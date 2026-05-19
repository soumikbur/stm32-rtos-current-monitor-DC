/*
 * WinsonLib_STM32.h
 * STM32 HAL port of WinsonLib v0.0.3
 *
 * Platform changes from Arduino version:
 *   - SoftwareSerial replaced by UART_HandleTypeDef* (hardware UART)
 *   - analogRead() replaced by ADC_HandleTypeDef* + channel polling
 *   - delay() -> HAL_Delay()
 *   - micros() -> DWT-based counter (enable via WinsonLib_EnableDWT())
 *   - Arduino String -> char[] buffers
 *   - pinMode/digitalWrite -> HAL_GPIO_WritePin (GPIO init via CubeMX)
 *   - byte/word -> uint8_t/uint16_t
 *
 * ADC configuration (override in your project if needed):
 *   WINSONLIB_ADC_VREF_MV   — ADC reference voltage in millivolts (default 3300)
 *   WINSONLIB_ADC_MAX_STEPS — ADC full-scale count (default 4095 for 12-bit)
 */

#ifndef WINSONLIB_STM32_H
#define WINSONLIB_STM32_H

//#include "stm32_hal_include.h"  /* see note below */
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*
 * stm32_hal_include.h must exist in your project and contain the correct
 * family header, e.g.:
 *   #include "stm32f4xx_hal.h"   // F4
 *   #include "stm32g0xx_hal.h"   // G0
 * Create it alongside this file or replace the include above directly.
 */

/* ---------- ADC scaling constants ---------------------------------------- */
#ifndef WINSONLIB_ADC_VREF_MV
#  define WINSONLIB_ADC_VREF_MV   3300   /* millivolts */
#endif
#ifndef WINSONLIB_ADC_MAX_STEPS
#  define WINSONLIB_ADC_MAX_STEPS 4095   /* 12-bit */
#endif

/* ---------- UART timeout for serial comms (ms) ---------------------------- */
#ifndef WINSONLIB_UART_BYTE_TIMEOUT_MS
#  define WINSONLIB_UART_BYTE_TIMEOUT_MS  2
#endif

/* ---------- Micros timer -------------------------------------------------- */
/*
 * Call WinsonLib_EnableDWT() once in main() before using WCS.
 * Requires the DWT unit available on Cortex-M3/M4/M7.
 * On M0/M0+ (e.g. STM32G0/F0/L0) use a spare TIM instead:
 *   configure a 1 MHz free-running TIM and redefine WinsonLib_Micros()
 *   to return its counter register.
 */
void     WinsonLib_EnableDWT(void);
uint32_t WinsonLib_Micros(void);

/* ---------- Sensor type enum --------------------------------------------- */
typedef enum {
    WINSON_AC,
    WINSON_DC,
    WINSON_AT,
    WINSON_ACDC,
    WINSON_Modbus,
    WINSON_NaN
} WType_t;

/* ---------- Measurement result struct ------------------------------------- */
typedef struct {
    WType_t Sign;
    double  Value;
} Wdata_t;

/* ---------- Internal byte-array / int32 union ----------------------------- */
typedef union {
    uint8_t array[4];
    int32_t ToInt32;
} BytesArrayConverter;

/* ---------- ADC channel descriptor (for WCS) ------------------------------ */
typedef struct {
    ADC_HandleTypeDef *hadc;
    uint32_t           channel;   /* ADC_CHANNEL_x */
    uint32_t           rank;      /* ADC_REGULAR_RANK_1 etc. (family-specific) */
} WCS_ADC_Channel;

/* ========================================================================== */
/* WSerial — wraps a hardware UART_HandleTypeDef                              */
/* ========================================================================== */
typedef struct {
    UART_HandleTypeDef *_huart;
    uint8_t             _ubuff[32];
    int                 _index;
    bool                _uif;
} WSerial_t;

void     WSerial_Init    (WSerial_t *ws, UART_HandleTypeDef *huart);
void     WSerial_Begin   (WSerial_t *ws);                          /* flush RX */
int      WSerial_Listen  (WSerial_t *ws, int ms);                  /* returns bytes received */
bool     WSerial_Write   (WSerial_t *ws, const char *text,
                           char *rxOut, int rxOutLen);             /* returns true on success */
int32_t  WSerial_WriteModbus(WSerial_t *ws,
                              uint8_t SlaveAddress,
                              uint8_t FunctionCode,
                              uint16_t DeviceAddress,
                              uint16_t RegisterNum);
void     WSerial_ClearRxBuff(WSerial_t *ws);
bool     WSerial_CheckComplete(const WSerial_t *ws);
void     WSerial_GetRxBuff(const WSerial_t *ws, char *out, int outLen);

/* ========================================================================== */
/* WCM — current/temperature module (continuous or Modbus UART)              */
/* ========================================================================== */
typedef struct {
    WSerial_t        _serial;
    WType_t          _type;
    GPIO_TypeDef    *_rstPort;
    uint16_t         _rstPin;
    uint8_t          _addr;
} WCM_t;

/*
 * WCM constructor equivalents:
 *   WCM_Init(wcm, huart, rstPort, rstPin, type)           — address defaults to 0x01
 *   WCM_InitAddr(wcm, huart, rstPort, rstPin, type, addr) — explicit address
 */
void    WCM_Init    (WCM_t *wcm, UART_HandleTypeDef *huart,
                     GPIO_TypeDef *rstPort, uint16_t rstPin, WType_t type);
void    WCM_InitAddr(WCM_t *wcm, UART_HandleTypeDef *huart,
                     GPIO_TypeDef *rstPort, uint16_t rstPin,
                     WType_t type, uint8_t address);
void    WCM_Begin   (WCM_t *wcm);   /* call once in setup; replaces Init() */

Wdata_t WCM_SignedCurrent(WCM_t *wcm);
double  WCM_mA      (WCM_t *wcm);
double  WCM_A       (WCM_t *wcm);
bool    WCM_Reset   (WCM_t *wcm);

/* Modbus-addressed variants */
double  WCM_mA_addr (WCM_t *wcm, uint8_t addr);
double  WCM_A_addr  (WCM_t *wcm, uint8_t addr);
double  WCM_oC      (WCM_t *wcm);
double  WCM_oC_addr (WCM_t *wcm, uint8_t addr);
double  WCM_oF      (WCM_t *wcm);
double  WCM_oF_addr (WCM_t *wcm, uint8_t addr);
bool    WCM_Reset_addr    (WCM_t *wcm, uint8_t addr);
bool    WCM_SetAddress    (WCM_t *wcm, uint8_t newAddr);
bool    WCM_SetAddressFull(WCM_t *wcm, uint8_t oldAddr, uint8_t newAddr);
bool    WCM_FactoryReset  (WCM_t *wcm);
uint8_t WCM_GetAddr       (const WCM_t *wcm);

/* ========================================================================== */
/* DWCS — digital current sensor (continuous / AT-command / Modbus UART)     */
/* ========================================================================== */
typedef struct {
    WSerial_t        _serial;
    WType_t          _type;
    /* In continuous (DC/AC) mode: rstPort/rstPin drive the RST line.
       In AT/Modbus mode: no external RST needed; rstPort may be NULL. */
    GPIO_TypeDef    *_rstPort;
    uint16_t         _rstPin;
    uint8_t          _addr;
} DWCS_t;

void    DWCS_Init    (DWCS_t *d, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *rstPort, uint16_t rstPin, WType_t type);
void    DWCS_InitAddr(DWCS_t *d, UART_HandleTypeDef *huart,
                      GPIO_TypeDef *rstPort, uint16_t rstPin,
                      WType_t type, uint8_t address);
void    DWCS_Begin   (DWCS_t *d);

Wdata_t DWCS_SignedCurrent(DWCS_t *d);
double  DWCS_A      (DWCS_t *d);
double  DWCS_mA     (DWCS_t *d);
double  DWCS_oC     (DWCS_t *d);
double  DWCS_oF     (DWCS_t *d);
bool    DWCS_Reset  (DWCS_t *d);
bool    DWCS_DC     (DWCS_t *d);
bool    DWCS_AC     (DWCS_t *d);

/* Modbus-addressed variants */
double  DWCS_mA_addr (DWCS_t *d, uint8_t addr);
double  DWCS_A_addr  (DWCS_t *d, uint8_t addr);
double  DWCS_oC_addr (DWCS_t *d, uint8_t addr);
double  DWCS_oF_addr (DWCS_t *d, uint8_t addr);
bool    DWCS_Reset_addr    (DWCS_t *d, uint8_t addr);
bool    DWCS_DC_addr       (DWCS_t *d, uint8_t addr);
bool    DWCS_AC_addr       (DWCS_t *d, uint8_t addr);
bool    DWCS_SetAddress    (DWCS_t *d, uint8_t newAddr);
bool    DWCS_SetAddressFull(DWCS_t *d, uint8_t oldAddr, uint8_t newAddr);
bool    DWCS_FactoryReset  (DWCS_t *d);
uint8_t DWCS_GetAddr       (const DWCS_t *d);

/* ========================================================================== */
/* WCS — analog Hall-effect current sensor                                    */
/* ========================================================================== */

/* Sensitivity defines (mV/A) — unchanged from original */
#define _WCS38A25  7000
#define _WCS37A50  3500
#define _WCS2801   2000
#define _WCS2702   1000
#define _WCS2705    260
#define _WCS2810    135
#define _WCS2720     65
#define _WCS2750     32
#define _WCS3740     32
#define _WCS2201   4200
#define _WCS2202   1120
#define _WCS2210    280
#define _WCS2800     70
#define _WCS6800     65
#define _WCS1800     66
#define _WCS1700     33
#define _WCS1600     22
#define _WCS1500     11
#define _WCS2200    140

typedef enum { WCS_SingleOutput, WCS_DifferentialOutput } WCS_Mode_t;

typedef struct {
    WCS_Mode_t       _mode;
    WCS_ADC_Channel  _ch1;        /* primary channel */
    WCS_ADC_Channel  _ch2;        /* differential second channel (same hadc OK) */
    uint16_t         _sensitivity; /* mV per Ampere */
    int16_t          _midPoint;
    int16_t          _dataScaled[120];
    uint32_t         _start;
} WCS_t;

/*
 * WCS_InitSingle(wcs, ch, mVperA)         — single-ended mode
 * WCS_InitDifferential(wcs, ch1, ch2, mVperA) — differential mode
 */
void   WCS_InitSingle      (WCS_t *wcs, WCS_ADC_Channel ch, uint16_t mVperA);
void   WCS_InitDifferential(WCS_t *wcs, WCS_ADC_Channel ch1, WCS_ADC_Channel ch2, uint16_t mVperA);
void   WCS_Reset (WCS_t *wcs);   /* calibrate zero-current midpoint */
double WCS_A_DC  (WCS_t *wcs);
double WCS_A_AC  (WCS_t *wcs);

#endif /* WINSONLIB_STM32_H */
