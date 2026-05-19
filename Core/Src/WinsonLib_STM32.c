/*
 * WinsonLib_STM32.cpp
 * STM32 HAL port of WinsonLib v0.0.3
 */

#include "WinsonLib_STM32.h"

/* ========================================================================== */
/* Timing helpers                                                              */
/* ========================================================================== */

void WinsonLib_EnableDWT(void)
{
    /* Enable DWT cycle counter — call once in main() before using WCS */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(DWT_LAR_Access_Allow)
    DWT->LAR = 0xC5ACCE55; /* unlock on Cortex-M7 */
#endif
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t WinsonLib_Micros(void)
{
    /* Returns microseconds since DWT was enabled.
     * Works on Cortex-M3/M4/M7 (72–480 MHz typical).
     * For M0/M0+: replace with a TIM counter running at 1 MHz. */
    return DWT->CYCCNT / (SystemCoreClock / 1000000UL);
}

/* ========================================================================== */
/* Internal: Modbus CRC16                                                     */
/* ========================================================================== */

static uint16_t _CRC(uint16_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 0x0001) {
            crc >>= 1;
            crc ^= 0xA001;
        } else {
            crc >>= 1;
        }
    }
    return crc;
}

/* ========================================================================== */
/* Internal: ADC single-shot read                                             */
/* ========================================================================== */

static int16_t _ADC_Read(ADC_HandleTypeDef *hadc, uint32_t channel, uint32_t rank)
{
    ADC_ChannelConfTypeDef cfg = {0};
    cfg.Channel      = channel;
    cfg.Rank         = rank;
    cfg.SamplingTime = ADC_SAMPLETIME_28CYCLES; /* adjust for your MCU family */
    HAL_ADC_ConfigChannel(hadc, &cfg);
    HAL_ADC_Start(hadc);
    if (HAL_ADC_PollForConversion(hadc, 10) != HAL_OK) return 0;
    int16_t v = (int16_t)HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);
    return v;
}

/* ========================================================================== */
/* WSerial                                                                    */
/* ========================================================================== */

void WSerial_Init(WSerial_t *ws, UART_HandleTypeDef *huart)
{
    ws->_huart = huart;
    ws->_index = 0;
    ws->_uif   = false;
    memset(ws->_ubuff, 0, sizeof(ws->_ubuff));
}

void WSerial_Begin(WSerial_t *ws)
{
    /* Drain any stale bytes from the hardware RX FIFO / DR register */
    WSerial_ClearRxBuff(ws);
    HAL_Delay(1000);
}

void WSerial_ClearRxBuff(WSerial_t *ws)
{
    /* Read and discard all pending bytes without blocking */
    uint8_t dummy;
    while (HAL_UART_Receive(ws->_huart, &dummy, 1, 1) == HAL_OK) { /* drain */ }
    ws->_index = 0;
    ws->_uif   = false;
    memset(ws->_ubuff, 0, sizeof(ws->_ubuff));
}

int WSerial_Listen(WSerial_t *ws, int ms)
{
    ws->_index = 0;
    ws->_uif   = false;

    uint32_t deadline = HAL_GetTick() + (uint32_t)ms;
    while (HAL_GetTick() < deadline) {
        uint8_t b;
        if (HAL_UART_Receive(ws->_huart, &b, 1,
                             WINSONLIB_UART_BYTE_TIMEOUT_MS) == HAL_OK) {
            if (ws->_index < (int)(sizeof(ws->_ubuff) - 1)) {
                ws->_ubuff[ws->_index++] = b;
            }
            if (b == '\n') {
                ws->_uif = true;
                return ws->_index;
            }
        }
    }
    return ws->_index;
}

bool WSerial_Write(WSerial_t *ws, const char *text, char *rxOut, int rxOutLen)
{
    HAL_UART_Transmit(ws->_huart, (uint8_t *)text, (uint16_t)strlen(text), 1000);
    WSerial_ClearRxBuff(ws);
    WSerial_Listen(ws, 2000);
    if (ws->_uif) {
        if (rxOut && rxOutLen > 0) {
            WSerial_GetRxBuff(ws, rxOut, rxOutLen);
        }
        return true;
    }
    return false;
}

int32_t WSerial_WriteModbus(WSerial_t *ws,
                             uint8_t SlaveAddress,
                             uint8_t FunctionCode,
                             uint16_t DeviceAddress,
                             uint16_t RegisterNum)
{
    uint16_t crc = 0xFFFF;
    uint8_t  wr[8];

    wr[0] = SlaveAddress;
    wr[1] = FunctionCode;
    wr[2] = (uint8_t)(DeviceAddress >> 8);
    wr[3] = (uint8_t)(DeviceAddress & 0xFF);
    wr[4] = (uint8_t)(RegisterNum  >> 8);
    wr[5] = (uint8_t)(RegisterNum  & 0xFF);
    for (int i = 0; i < 6; i++) crc = _CRC(crc, wr[i]);
    wr[6] = (uint8_t)(crc & 0xFF);
    wr[7] = (uint8_t)(crc >> 8);

    HAL_UART_Transmit(ws->_huart, wr, 8, 1000);

    WSerial_ClearRxBuff(ws);
    WSerial_Listen(ws, 1000);

    uint8_t buf[20];
    int     len = ws->_index;
    memcpy(buf, ws->_ubuff, (size_t)len);

    /* Find slave address byte */
    int i = 0;
    bool found = false;
    for (; i < len; i++) {
        if (buf[i] == SlaveAddress) {
            crc = _CRC(0xFFFF, buf[i]);
            found = true;
            break;
        }
    }
    if (!found) return 0;

    /* Check function code */
    if (i + 1 >= len) return 0;
    if (buf[i + 1] != FunctionCode) return 0;
    crc = _CRC(crc, buf[i + 1]);

    if (FunctionCode == 0x06) {
        /* Echo check for write response */
        if (i + 7 >= len) return 0;
        for (int j = 0; j < 6; j++) {
            if (wr[2 + j] != buf[i + 2 + j]) return 0;
        }
        return (int32_t)RegisterNum;
    } else if (FunctionCode == 0x03) {
        if (i + 8 >= len) return 0;
        if (buf[i + 2] != 0x04) return 0;

        BytesArrayConverter conv;
        conv.array[3] = buf[i + 3];
        conv.array[2] = buf[i + 4];
        conv.array[1] = buf[i + 5];
        conv.array[0] = buf[i + 6];

        for (int j = 2; j < 7; j++) crc = _CRC(crc, buf[i + j]);
        if ((uint8_t)(crc & 0xFF) != buf[i + 7]) return 0;
        if ((uint8_t)(crc >> 8)   != buf[i + 8]) return 0;

        return conv.ToInt32;
    }
    return 0;
}

bool WSerial_CheckComplete(const WSerial_t *ws)
{
    return ws->_uif;
}

void WSerial_GetRxBuff(const WSerial_t *ws, char *out, int outLen)
{
    int n = (ws->_index < outLen - 1) ? ws->_index : outLen - 1;
    memcpy(out, ws->_ubuff, (size_t)n);
    out[n] = '\0';
}

/* ========================================================================== */
/* Internal: parse signed current from continuous-mode ASCII response         */
/*                                                                             */
/* Protocol: "~XXXX.X\r\n" = AC, "+XXXX.X\r\n" = DC positive,               */
/*           "-XXXX.X\r\n" = DC negative.                                     */
/* ========================================================================== */

static Wdata_t _ParseContinuous(WSerial_t *ws)
{
    Wdata_t d;
    d.Sign  = WINSON_NaN;
    d.Value = 0.0;

    if (!WSerial_CheckComplete(ws)) return d;

    char buf[32];
    WSerial_GetRxBuff(ws, buf, sizeof(buf));

    if (strchr(buf, '~')) {
        /* Remove '~', rest is numeric string */
        char *p = buf;
        while (*p) { if (*p == '~') memmove(p, p+1, strlen(p)); else p++; }
        d.Sign = WINSON_AC;
    } else if (strchr(buf, '+')) {
        char *p = buf;
        while (*p) { if (*p == '+') memmove(p, p+1, strlen(p)); else p++; }
        d.Sign = WINSON_DC;
    } else if (strchr(buf, '-')) {
        d.Sign = WINSON_DC;
    } else {
        return d;
    }

    /* Strip trailing \r\n */
    char *cr = strchr(buf, '\r'); if (cr) *cr = '\0';
    char *lf = strchr(buf, '\n'); if (lf) *lf = '\0';

    d.Value = strtod(buf, NULL);
    return d;
}

/* ========================================================================== */
/* WCM                                                                        */
/* ========================================================================== */

void WCM_Init(WCM_t *wcm, UART_HandleTypeDef *huart,
              GPIO_TypeDef *rstPort, uint16_t rstPin, WType_t type)
{
    WCM_InitAddr(wcm, huart, rstPort, rstPin, type, 0x01);
}

void WCM_InitAddr(WCM_t *wcm, UART_HandleTypeDef *huart,
                  GPIO_TypeDef *rstPort, uint16_t rstPin,
                  WType_t type, uint8_t address)
{
    WSerial_Init(&wcm->_serial, huart);
    wcm->_type    = type;
    wcm->_rstPort = rstPort;
    wcm->_rstPin  = rstPin;
    wcm->_addr    = (address == 0x00) ? 0x01 : address;
}

void WCM_Begin(WCM_t *wcm)
{
    HAL_GPIO_WritePin(wcm->_rstPort, wcm->_rstPin, GPIO_PIN_SET);
    WSerial_Begin(&wcm->_serial);
}

uint8_t WCM_GetAddr(const WCM_t *wcm) { return wcm->_addr; }

Wdata_t WCM_SignedCurrent(WCM_t *wcm)
{
    Wdata_t d;
    d.Sign  = WINSON_NaN;
    d.Value = 0.0;

    if (wcm->_type == WINSON_Modbus) {
        d.Value = WCM_mA_addr(wcm, wcm->_addr);
        return d;
    }

    WSerial_ClearRxBuff(&wcm->_serial);
    for (int i = 0; i < 5; i++) {
        if (WSerial_Listen(&wcm->_serial, 1000) == 8) break;
    }
    return _ParseContinuous(&wcm->_serial);
}

double WCM_mA(WCM_t *wcm)
{
    if (wcm->_type == WINSON_Modbus) return WCM_mA_addr(wcm, wcm->_addr);
    return WCM_SignedCurrent(wcm).Value;
}

double WCM_A(WCM_t *wcm) { return WCM_mA(wcm) / 1000.0; }

bool WCM_Reset(WCM_t *wcm)
{
    if (wcm->_type == WINSON_Modbus) return WCM_Reset_addr(wcm, wcm->_addr);
    HAL_GPIO_WritePin(wcm->_rstPort, wcm->_rstPin, GPIO_PIN_RESET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(wcm->_rstPort, wcm->_rstPin, GPIO_PIN_SET);
    return true;
}

double WCM_mA_addr(WCM_t *wcm, uint8_t addr)
{
    int32_t d = WSerial_WriteModbus(&wcm->_serial, addr, 0x03, 0x0002, 0x0002);
    return (double)d;
}

double WCM_A_addr(WCM_t *wcm, uint8_t addr)
{
    return WCM_mA_addr(wcm, addr) / 1000.0;
}

double WCM_oC(WCM_t *wcm)       { return WCM_oC_addr(wcm, wcm->_addr); }
double WCM_oF(WCM_t *wcm)       { return WCM_oC(wcm) * 1.8 + 32.0; }
double WCM_oF_addr(WCM_t *wcm, uint8_t addr) { return WCM_oC_addr(wcm, addr) * 1.8 + 32.0; }

double WCM_oC_addr(WCM_t *wcm, uint8_t addr)
{
    int32_t d = WSerial_WriteModbus(&wcm->_serial, addr, 0x03, 0x0004, 0x0002);
    return (double)d / 10.0;
}

bool WCM_Reset_addr(WCM_t *wcm, uint8_t addr)
{
    int32_t d = WSerial_WriteModbus(&wcm->_serial, addr, 0x06, 0x0000, 0x0100);
    return (d == 256 || addr == 0x00);
}

bool WCM_SetAddress(WCM_t *wcm, uint8_t newAddr)
{
    if (WCM_SetAddressFull(wcm, wcm->_addr, newAddr)) {
        wcm->_addr = newAddr;
        return true;
    }
    return false;
}

bool WCM_SetAddressFull(WCM_t *wcm, uint8_t oldAddr, uint8_t newAddr)
{
    if (wcm->_type != WINSON_Modbus || newAddr == 0x00) return false;
    int32_t d = WSerial_WriteModbus(&wcm->_serial, oldAddr, 0x06, 0x0010, newAddr);
    return (d == (int32_t)newAddr || oldAddr == 0x00);
}

bool WCM_FactoryReset(WCM_t *wcm) { return WCM_SetAddressFull(wcm, 0x00, 0x01); }

/* ========================================================================== */
/* DWCS                                                                       */
/* ========================================================================== */

void DWCS_Init(DWCS_t *d, UART_HandleTypeDef *huart,
               GPIO_TypeDef *rstPort, uint16_t rstPin, WType_t type)
{
    DWCS_InitAddr(d, huart, rstPort, rstPin, type, 0x01);
}

void DWCS_InitAddr(DWCS_t *d, UART_HandleTypeDef *huart,
                   GPIO_TypeDef *rstPort, uint16_t rstPin,
                   WType_t type, uint8_t address)
{
    WSerial_Init(&d->_serial, huart);
    d->_type    = type;
    d->_rstPort = rstPort;
    d->_rstPin  = rstPin;
    d->_addr    = address;
}

void DWCS_Begin(DWCS_t *d)
{
    if (d->_type == WINSON_DC || d->_type == WINSON_AC) {
        /* Continuous mode: RST pin used for hardware reset */
        if (d->_rstPort) HAL_GPIO_WritePin(d->_rstPort, d->_rstPin, GPIO_PIN_SET);
    }
    WSerial_Begin(&d->_serial);
}

uint8_t DWCS_GetAddr(const DWCS_t *d) { return d->_addr; }

static Wdata_t _DWCS_ReadNormal(DWCS_t *d)
{
    WSerial_ClearRxBuff(&d->_serial);
    for (int i = 0; i < 5; i++) {
        if (WSerial_Listen(&d->_serial, 1000) == 8) break;
    }
    Wdata_t r = _ParseContinuous(&d->_serial);
    /* DWCS continuous mode has no '-' prefix for DC; default sign to DC */
    if (r.Sign == WINSON_NaN && WSerial_CheckComplete(&d->_serial))
        r.Sign = WINSON_DC;
    return r;
}

static Wdata_t _DWCS_ReadPro(DWCS_t *d)
{
    Wdata_t r;
    r.Sign  = WINSON_NaN;
    r.Value = 0.0;

    char rx[32];
    if (!WSerial_Write(&d->_serial, "AT+MEAS\r\n", rx, sizeof(rx))) return r;

    if (strchr(rx, '~')) {
        char *p = rx;
        while (*p) { if (*p == '~') memmove(p, p+1, strlen(p)); else p++; }
        r.Sign = WINSON_AC;
    } else {
        r.Sign = WINSON_DC;
    }
    char *cr = strchr(rx, '\r'); if (cr) *cr = '\0';
    char *lf = strchr(rx, '\n'); if (lf) *lf = '\0';
    r.Value = strtod(rx, NULL);
    return r;
}

Wdata_t DWCS_SignedCurrent(DWCS_t *d)
{
    Wdata_t zero = {WINSON_NaN, 0.0};
    switch (d->_type) {
        case WINSON_AC:
        case WINSON_DC:      return _DWCS_ReadNormal(d);
        case WINSON_AT:      return _DWCS_ReadPro(d);
        default:             return zero;
    }
}

double DWCS_A(DWCS_t *d)
{
    switch (d->_type) {
        case WINSON_AC:
        case WINSON_DC:      return _DWCS_ReadNormal(d).Value;
        case WINSON_AT:      return _DWCS_ReadPro(d).Value;
        case WINSON_Modbus:  return DWCS_A_addr(d, d->_addr);
        default:             return 0.0;
    }
}

double DWCS_mA(DWCS_t *d) { return DWCS_A(d) * 1000.0; }

double DWCS_oC(DWCS_t *d)
{
    if (d->_type == WINSON_Modbus) return DWCS_oC_addr(d, d->_addr);
    char rx[32];
    WSerial_Write(&d->_serial, "AT+TEMP\r\n", rx, sizeof(rx));
    char *cr = strchr(rx, '\r'); if (cr) *cr = '\0';
    return strtod(rx, NULL);
}

double DWCS_oF(DWCS_t *d) { return DWCS_oC(d) * 1.8 + 32.0; }

bool DWCS_DC(DWCS_t *d)
{
    if (d->_type == WINSON_Modbus) return DWCS_DC_addr(d, d->_addr);
    char rx[16];
    WSerial_Write(&d->_serial, "AT+CURR,0\r\n", rx, sizeof(rx));
    return (strncmp(rx, "OK", 2) == 0);
}

bool DWCS_AC(DWCS_t *d)
{
    if (d->_type == WINSON_Modbus) return DWCS_AC_addr(d, d->_addr);
    char rx[16];
    WSerial_Write(&d->_serial, "AT+CURR,1\r\n", rx, sizeof(rx));
    return (strncmp(rx, "OK", 2) == 0);
}

bool DWCS_Reset(DWCS_t *d)
{
    switch (d->_type) {
        case WINSON_AC:
        case WINSON_DC:
            if (d->_rstPort) {
                HAL_GPIO_WritePin(d->_rstPort, d->_rstPin, GPIO_PIN_RESET);
                HAL_Delay(1000);
                HAL_GPIO_WritePin(d->_rstPort, d->_rstPin, GPIO_PIN_SET);
            }
            return true;
        case WINSON_AT: {
            char rx[16];
            WSerial_Write(&d->_serial, "AT+RST\r\n", rx, sizeof(rx));
            return (strncmp(rx, "OK", 2) == 0);
        }
        case WINSON_Modbus:
            return DWCS_Reset_addr(d, d->_addr);
        default:
            return false;
    }
}

/* Modbus-addressed DWCS variants */
double DWCS_mA_addr(DWCS_t *d, uint8_t addr)
{
    int32_t v = WSerial_WriteModbus(&d->_serial, addr, 0x03, 0x0002, 0x0002);
    return (double)v;
}

double DWCS_A_addr(DWCS_t *d, uint8_t addr)
{
    return DWCS_mA_addr(d, addr) / 1000.0;
}

double DWCS_oC_addr(DWCS_t *d, uint8_t addr)
{
    int32_t v = WSerial_WriteModbus(&d->_serial, addr, 0x03, 0x0004, 0x0002);
    return (double)v / 10.0;
}

double DWCS_oF_addr(DWCS_t *d, uint8_t addr)
{
    return DWCS_oC_addr(d, addr) * 1.8 + 32.0;
}

bool DWCS_Reset_addr(DWCS_t *d, uint8_t addr)
{
    int32_t v = WSerial_WriteModbus(&d->_serial, addr, 0x06, 0x0000, 0x0100);
    return (v == 256 || addr == 0x00);
}

bool DWCS_DC_addr(DWCS_t *d, uint8_t addr)
{
    int32_t v = WSerial_WriteModbus(&d->_serial, addr, 0x06, 0x0020, 0x0000);
    return (v == 0 || addr == 0x00);
}

bool DWCS_AC_addr(DWCS_t *d, uint8_t addr)
{
    int32_t v = WSerial_WriteModbus(&d->_serial, addr, 0x06, 0x0020, 0x0001);
    return (v == 1 || addr == 0x00);
}

bool DWCS_SetAddress(DWCS_t *d, uint8_t newAddr)
{
    if (DWCS_SetAddressFull(d, d->_addr, newAddr)) {
        d->_addr = newAddr;
        return true;
    }
    return false;
}

bool DWCS_SetAddressFull(DWCS_t *d, uint8_t oldAddr, uint8_t newAddr)
{
    if (d->_type != WINSON_Modbus || newAddr == 0x00) return false;
    int32_t v = WSerial_WriteModbus(&d->_serial, oldAddr, 0x06, 0x0010, newAddr);
    return (v == (int32_t)newAddr || oldAddr == 0x00);
}

bool DWCS_FactoryReset(DWCS_t *d) { return DWCS_SetAddressFull(d, 0x00, 0x01); }

/* ========================================================================== */
/* WCS                                                                        */
/* ========================================================================== */

void WCS_InitSingle(WCS_t *wcs, WCS_ADC_Channel ch, uint16_t mVperA)
{
    wcs->_mode        = WCS_SingleOutput;
    wcs->_ch1         = ch;
    wcs->_sensitivity = mVperA;
    wcs->_midPoint    = (WINSONLIB_ADC_MAX_STEPS + 1) / 2; /* ~half-scale */
}

void WCS_InitDifferential(WCS_t *wcs,
                           WCS_ADC_Channel ch1, WCS_ADC_Channel ch2,
                           uint16_t mVperA)
{
    wcs->_mode        = WCS_DifferentialOutput;
    wcs->_ch1         = ch1;
    wcs->_ch2         = ch2;
    wcs->_sensitivity = mVperA;
    wcs->_midPoint    = 0;
}

static void _WCS_ReadBuffer(WCS_t *wcs, int16_t *buf)
{
    /* Warm-up reads (discard) */
    _ADC_Read(wcs->_ch1.hadc, wcs->_ch1.channel, wcs->_ch1.rank);
    if (wcs->_mode == WCS_DifferentialOutput)
        _ADC_Read(wcs->_ch2.hadc, wcs->_ch2.channel, wcs->_ch2.rank);

    for (int i = 0; i < 120; i++) {
        wcs->_start = WinsonLib_Micros();
        if (wcs->_mode == WCS_SingleOutput) {
            buf[i] = _ADC_Read(wcs->_ch1.hadc, wcs->_ch1.channel, wcs->_ch1.rank);
        } else {
            int16_t a = _ADC_Read(wcs->_ch1.hadc, wcs->_ch1.channel, wcs->_ch1.rank);
            int16_t b = _ADC_Read(wcs->_ch2.hadc, wcs->_ch2.channel, wcs->_ch2.rank);
            buf[i] = a - b;
        }
        /* Maintain ~1200 Hz sample rate (829 µs between samples) */
        while (WinsonLib_Micros() - wcs->_start < 829U) { /* busy wait */ }
    }
}

void WCS_Reset(WCS_t *wcs)
{
    _WCS_ReadBuffer(wcs, wcs->_dataScaled);
    int32_t sum = 0;
    for (int i = 0; i < 120; i++) sum += wcs->_dataScaled[i];
    wcs->_midPoint = (int16_t)(sum / 120);
}

/*
 * ADC voltage per step: WINSONLIB_ADC_VREF_MV / WINSONLIB_ADC_MAX_STEPS (mV)
 * Current = (steps * Vref_mV / ADC_MAX) / sensitivity_mV  (Amperes)
 *         = steps * Vref_mV / (ADC_MAX * sensitivity_mV)
 */
double WCS_A_DC(WCS_t *wcs)
{
    int16_t steps;
    if (wcs->_mode == WCS_SingleOutput) {
        _ADC_Read(wcs->_ch1.hadc, wcs->_ch1.channel, wcs->_ch1.rank); /* stabilise */
        steps = _ADC_Read(wcs->_ch1.hadc, wcs->_ch1.channel, wcs->_ch1.rank)
                - wcs->_midPoint;
    } else {
        _ADC_Read(wcs->_ch1.hadc, wcs->_ch1.channel, wcs->_ch1.rank);
        _ADC_Read(wcs->_ch2.hadc, wcs->_ch2.channel, wcs->_ch2.rank);
        steps = (_ADC_Read(wcs->_ch1.hadc, wcs->_ch1.channel, wcs->_ch1.rank)
               - _ADC_Read(wcs->_ch2.hadc, wcs->_ch2.channel, wcs->_ch2.rank))
               - wcs->_midPoint;
    }
    return (double)steps
           * (double)WINSONLIB_ADC_VREF_MV
           / (double)WINSONLIB_ADC_MAX_STEPS
           / (double)wcs->_sensitivity;
}

double WCS_A_AC(WCS_t *wcs)
{
    _WCS_ReadBuffer(wcs, wcs->_dataScaled);
    double sum = 0.0;
    for (int i = 0; i < 120; i++) {
        double v = (double)(wcs->_dataScaled[i] - wcs->_midPoint)
                   * (double)WINSONLIB_ADC_VREF_MV
                   / (double)WINSONLIB_ADC_MAX_STEPS
                   / (double)wcs->_sensitivity;
        sum += v * v;
    }
    return sqrt(sum / 120.0);
}
