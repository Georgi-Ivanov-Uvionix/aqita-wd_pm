
/**
  ******************************************************************************
  * @file    uvx_comm_bq.h
  * @brief   Header file for microcontroller-to-microcontroller (BQ) communication.
  *
  * This file defines constants and includes required headers for implementing
  * a communication protocol between two microcontrollers. The protocol uses
  * a start byte, control codes, and a maximum packet size for data exchange.
  *
  * - UVX_START_BYTE: Start-of-packet marker for framing.
  * - UVX_CTRL_READ:  Example control code for read operations.
  * - UVX_MAX_PACKET: Maximum allowed packet size in bytes.
  *
  *   * Protocol Packet Structure:
  * ----------------------------------------------------------------------------
  * | Byte Index |    Name        | Size |         Description                 |
  * |------------|--------------- |------|-------------------------------------|
  * |    0       | Start Byte     |  1   | Fixed value to mark packet start    |
  * |            |                |      | (0x02)                              |
  * |    1       | Control Byte   |  1   | Command type:                       |
  * |            |                |      |   0x00 = Write                      |
  * |            |                |      |   0x01 = Read                       |
  * |            |                |      |   0x02 = Send                       |
  * |            |                |      |   0x03 = Send Error                 |
  * |    2       | Command Byte   |  1   | Specifies the action or register    |
  * |  3 - 4     | Payload Size   |  2   | Payload size (number of bytes)      |
  * |  5 ...     | Payload        |  N   | Optional data depending on command  |
  * | 5+N        | CRC Byte       |  1   | XOR or CRC-8 over all previous bytes|
  * ----------------------------------------------------------------------------
  *
  ******************************************************************************
  */
#ifndef __UVX_COMM_BQ_H
#define __UVX_COMM_BQ_H

/* Includes ------------------------------------------------------------------*/
#include "uvx_universal_types.h"
#include <stdint.h>
#include "main.h"
#include "uvx_i2c.h"

#define BUFF_SIZE_TX_BQ			100
#define BUFF_SIZE_RX_BQ			100
#define UVX_BQ_I2C_TX_STAGING_SIZE 20

#define UVX_PACKET_SIZE     1000  // Maximum packet size
#define UVX_START_BYTE      0x02  // Start byte for framing packets

#define BYTE_CTRL       1 // Control byte index in the packet
#define BYTE_CMD        2 // Command byte index in the packet
#define BYTE_SIZE_MSB   3 // Payload size MSB index in the packet
#define BYTE_SIZE_LSB   4 // Payload size LSB index in the packet
#define BYTE_DATA       5 // Data byte index in the packet

#define BQ_H_I2C_ADDRESS    0x0B // I2C address for BQ communication
#define BQ_L_I2C_ADDRESS    0x0D // I2C address for BQ communication

#define BQ_DEVICES          2
#define BQ_MAX_NO_RESPONSE  1000 

#define SRAM1 __attribute__((section(".sram1")))

typedef union MANUFACTURE_STATUS_U
{
    uint16_t data;
    struct {
        /* Bits 0–7 */
        uint16_t PCHG_EN   : 1;  // Bit 0 : Pre-charge enable
        uint16_t CHG_EN    : 1;  // Bit 1 : Charge enable
        uint16_t DSG_EN    : 1;  // Bit 2 : Discharge enable
        uint16_t GAUGE_EN  : 1;  // Bit 3 : Gauging enable
        uint16_t FET_EN    : 1;  // Bit 4 : FET enable
        uint16_t LF_EN     : 1;  // Bit 5 : Lifetime data enable
        uint16_t PF_EN     : 1;  // Bit 6 : Permanent failure enable
        uint16_t BBR_EN    : 1;  // Bit 7 : Black-box recorder enable

        /* Bits 8–15 */
        uint16_t FUSE_EN   : 1;  // Bit 8  : Fuse enable
        uint16_t LED_EN    : 1;  // Bit 9  : LED enable
        uint16_t RSVD10    : 1;  // Bit 10 : Reserved
        uint16_t RSVD11    : 1;  // Bit 11 : Reserved
        uint16_t RSVD12    : 1;  // Bit 12 : Reserved
        uint16_t PDSG_EN   : 1;  // Bit 13 : Permanent discharge enable
        uint16_t LT_TEST   : 1;  // Bit 14 : Lifetime test enable
        uint16_t CAL_EN    : 1;  // Bit 15 : Calibration enable
    } bits;
} MANUFACTURE_STATUS_U;

typedef struct MANUFACTURE_STATUS_S
{
    uint8_t raw[3];
    MANUFACTURE_STATUS_U reg;
} MANUFACTURE_STATUS_S;

typedef union OPERATION_STATUS_U
{
    uint32_t data;
    struct {
        /* Bits 0–7 */
        uint32_t PRES      : 1;  // Bit 0  : System present (active low)
        uint32_t DSG       : 1;  // Bit 2  : Discharge
        uint32_t CHG       : 1;  // Bit 1  : Charge
        uint32_t PCHG      : 1;  // Bit 3  : Pre-charge
        uint32_t PDSG      : 1;  // Bit 4  : Pre-discharge
        uint32_t FUSE      : 1;  // Bit 5  : Fuse status
        uint32_t RSVD6     : 1;  // Bit 6  : Reserved
        uint32_t BTP_INT   : 1;  // Bit 7  : Battery trip interrupt

        /* Bits 8–15 */
        uint32_t SEC0      : 1;  // Bit 8
        uint32_t SEC1      : 1;  // Bit 9
        uint32_t SDV       : 1;  // Bit 10
        uint32_t SS        : 1;  // Bit 11
        uint32_t PF        : 1;  // Bit 12
        uint32_t XDSG      : 1;  // Bit 13
        uint32_t XCHG      : 1;  // Bit 14
        uint32_t SLEEP     : 1;  // Bit 15

        /* Bits 16–23 */
        uint32_t SDM       : 1;  // Bit 16 : Shutdown via command
        uint32_t LED       : 1;  // Bit 17 : LED display
        uint32_t AUTH      : 1;  // Bit 18 : Authentication in progress
        uint32_t AUTOCALM  : 1;  // Bit 19 : Auto CC Offset calibration
        uint32_t CAL       : 1;  // Bit 20 : Calibration output valid
        uint32_t CAL_OFFSET: 1;  // Bit 21 : Raw CC offset calibration valid
        uint32_t XL        : 1;  // Bit 22 : 400-kHz SMBus mode
        uint32_t SLEEPM    : 1;  // Bit 23 : Sleep via command

        /* Bits 24–31 */
        uint32_t INIT      : 1;  // Bit 24 : Initialization after reset
        uint32_t SMBLCAL   : 1;  // Bit 25 : Auto CC calibration on SMBus low
        uint32_t SLPAD     : 1;  // Bit 26 : ADC active in sleep
        uint32_t SLPCC     : 1;  // Bit 27 : CC active in sleep
        uint32_t CB        : 1;  // Bit 28 : Cell balancing active
        uint32_t EMSHUT    : 1;  // Bit 29 : Emergency FET shutdown
        uint32_t RSVD30    : 1;  // Bit 30 : Reserved
        uint32_t IATA_CTERM: 1;  // Bit 31 : IATA charge control
    } bits;
} OPERATION_STATUS_U;

typedef struct OPERATION_STATUS_S
{
    uint8_t raw[5];
    OPERATION_STATUS_U reg;
} OPERATION_STATUS_S;

typedef union GAUGING_STATUS_U
{
    uint32_t data;
    struct {
        /* Bits 0–7 */
        uint32_t FD        : 1;  // Bit 0  : Full Discharge detected
        uint32_t FC        : 1;  // Bit 1  : Full Charge detected
        uint32_t TD        : 1;  // Bit 2  : Terminate Discharge
        uint32_t TC        : 1;  // Bit 3  : Terminate Charge
        uint32_t BAL_EN    : 1;  // Bit 4  : Cell balancing enabled
        uint32_t EDV       : 1;  // Bit 5  : End-of-discharge detected
        uint32_t DSG       : 1;  // Bit 6  : Discharge detected
        uint32_t CF        : 1;  // Bit 7  : Charge fault

        /* Bits 8–15 */
        uint32_t REST      : 1;  // Bit 8  : Rest detected
        uint32_t RSVD9     : 1;  // Bit 9  : Reserved
        uint32_t R_DIS     : 1;  // Bit 10 : Resistance discharge
        uint32_t VOK       : 1;  // Bit 11 : Voltage OK
        uint32_t QEN       : 1;  // Bit 12 : QMax updates enabled
        uint32_t SLP_QMAX  : 1;  // Bit 13 : QMax update in progress
        uint32_t RSVD      : 1;  // Bit 14 : Reserved

        /* Bits 16–23 */
        uint32_t NSFM      : 1;  // Bit 15 : QMax status
        uint32_t VDQ       : 1;  // Bit 16 : Resistance update active
        uint32_t QMAX      : 1;  // Bit 17 : Load mode
        uint32_t RX        : 1;  // Bit 18 : Fast relax
        uint32_t LDMD      : 1;  // Bit 19 : Open-circuit voltage update
        uint32_t OCV_FR    : 1;  // Bit 20 : Reserved
        uint32_t RSVD21    : 1;  // Bit 21 : Reserved
        uint32_t RSVD22    : 1;  // Bit 22 : Reserved
        uint32_t RSVD23    : 1;  // Bit 23 : Reserved
        uint32_t RSVD24    : 1;  // Bit 24 : Reserved
        uint32_t RSVD25    : 1;  // Bit 25 : Reserved
        uint32_t RSVD26    : 1;  // Bit 26 : Reserved
        uint32_t RSVD27    : 1;  // Bit 27 : Reserved
        uint32_t RSVD28    : 1;  // Bit 28 : Reserved
        uint32_t RSVD29    : 1;  // Bit 29 : Reserved
        uint32_t RSVD30    : 1;  // Bit 30 : Reserved
        uint32_t RSVD31    : 1;  // Bit 31 : Reserved
    } bits;
} GAUGING_STATUS_U;

typedef struct GAUGING_STATUS_S
{
    uint8_t raw[4];
    GAUGING_STATUS_U reg;
} GAUGING_STATUS_S;

typedef struct UVX_BQ_DATA
{
    uint16_t temperature;               // Temperature in 0.1 Kelvin
    uint16_t voltage;                   // Voltage in millivolts    
    int16_t  current;                   // Current in milliamps
    uint16_t relative_state_of_charge;  // Relative State of Charge in percentage (0-100%)
    uint16_t absolute_state_of_charge;  // Absolute State of Charge in percentage (0-100%)
    uint16_t remaining_capacity;        // Remaining Capacity in mAh
    uint16_t full_charge_capacity;      // Full Charge Capacity in mAh
    uint16_t average_time_to_empty;     // Average Time to Empty in minutes
    uint16_t average_time_to_full;      // Average Time to Full in minutes
    uint8_t  SOH;                       // State of Health in percentage (0-100%)
    uint16_t  cycle_count;               // Cycle Count
    int16_t  charging_current;          // Charging Current in milliamps
    uint16_t charging_voltage;          // Charging Voltage in millivolts
    uint16_t design_capacity;           // Design Capacity in mAh
    uint16_t design_voltage;            // Design Voltage in millivolts 
    uint16_t specification_info;       // Specification Information
    uint16_t manufacturer_date;        // Manufacturer Date
    uint16_t serial_number;            // Serial Number
    uint16_t voltage_per_cell;         // Voltage per cell in millivolts batt voltage / cell count

    //===========0x0071 DAStatus1======================================
    uint8_t  dastatus1;
    uint8_t  dastatus1_size;
    uint16_t cell_voltage_1;          // Cell Voltage 1 in millivolts
    uint16_t cell_voltage_2;          // Cell Voltage 2 in millivolts
    uint16_t cell_voltage_3;          // Cell Voltage 3 in millivolts
    uint16_t cell_voltage_4;          // Cell Voltage 4 in millivolts
    uint16_t bat_voltage;              // Battery Voltage in millivolts
    uint16_t pack_voltage;              // pack Voltage in millivolts
    uint16_t cell_current_1;              // Cell Current 1 in milliamps  
    uint16_t cell_current_2;              // Cell Current 2 in milliamps  
    uint16_t cell_current_3;              // Cell Current 3 in milliamps  
    uint16_t cell_current_4;              // Cell Current 4 in milliamps  
    uint16_t cell_power_1;              // Cell Power 1 in milliwatts  
    uint16_t cell_power_2;              // Cell Power 2 in milliwatts  
    uint16_t cell_power_4;              // Cell Power 4 in milliwatts  
    uint16_t cell_power_3;              // Cell Power 3 in milliwatts  
    int16_t power;                  //Power calculated by Voltage() × Current()
    uint16_t power_avg;                  //Average Power

    //===========0x0072 DAStatus2======================================
    uint8_t  dastatus2;
    uint8_t  dastatus2_size;
    uint16_t int_temperature;    
    uint16_t ts1_temperature;    
    uint16_t ts2_temperature;    
    uint16_t ts3_temperature;    
    uint16_t ts4_temperature;    
    uint16_t cell_temperature;   
    uint16_t fet_temperature;    
    uint16_t gauging_temperature;

    //===========0x0074 GAUGING_STATUS_2======================================
    uint8_t gauging_status_2;
    uint8_t gauging_status_2_size;
    uint8_t pack_grid;          // Active pack grid point (minimum of CellGrid0 to Cell Grid3). This data is only valid during DISCHARGE mode when [R_DIS] = 0. If [R_DIS] = 1 or not discharging, this value is not updated.
    uint8_t ls_status;          // LStatus—Learned status of resistance table
    uint8_t cell_grid_1;        // Active grid point of Cell 1. This data is only valid during DISCHARGE mode when [R_DIS] = 0. If [R_DIS] = 1 or not discharging, this value is not updated.
    uint8_t cell_grid_2;        // Active grid point of Cell 2. This data is only valid during DISCHARGE mode when [R_DIS] = 0. If [R_DIS] = 1 or not discharging, this value is not updated.
    uint8_t cell_grid_3;        // Active grid point of Cell 3. This data is only valid during DISCHARGE mode when [R_DIS] = 0. If [R_DIS] = 1 or not discharging, this value is not updated.
    uint8_t cell_grid_4;        // Active grid point of Cell 4. This data is only valid during DISCHARGE mode when [R_DIS] = 0. If [R_DIS] = 1 or not discharging, this value is not updated.
    uint8_t state_time_s[4];    // Time passed since last state change (DISCHARGE, CHARGE, REST)
    uint16_t DOD0_0;            // Depth of discharge for Cell 1
    uint16_t DOD0_1;            // Depth of discharge for Cell 2
    uint16_t DOD0_2;            // Depth of discharge for Cell 3
    uint16_t DOD0_3;            // Depth of discharge for Cell 4
    uint16_t DOD0_Q_mAh;        // Passed Q. Passed capacity since the last DOD0 update, mAh
    uint16_t DOD0_E_cWh;        // Passed E. Passed capacity since the last DOD0 update, cWh
    uint16_t DOD0_time_hr16;    // Passed time since the last DOD0 update, hours/16
    uint16_t DOD0_EOC_0;        // Depth of discharge at end of charge of Cell 1
    uint16_t DOD0_EOC_1;        // Depth of discharge at end of charge of Cell 2
    uint16_t DOD0_EOC_2;        // Depth of discharge at end of charge of Cell 3
    uint16_t DOD0_EOC_3;        // Depth of discharge at end of charge of Cell 4

    //===========0x0075 GAUGING_STATUS_3======================================
    uint8_t gauging_status_3;
    uint8_t gauging_status_3_size;
    uint16_t Qmax_cell_1;       //mAh
    uint16_t Qmax_cell_2;       //mAh
    uint16_t Qmax_cell_3;       //mAh
    uint16_t Qmax_cell_4;       //mAh
    uint16_t DOD_cell_1;        // Depth of discharge for Cell 1 
    uint16_t DOD_cell_2;        // Depth of discharge for Cell 2 
    uint16_t DOD_cell_3;        // Depth of discharge for Cell 3 
    uint16_t DOD_cell_4;        // Depth of discharge for Cell 4 
    uint16_t Qmax_passed;       // Pass capacity since last QMax DOD value is saved. mAh
    uint16_t Qmax_time;         // Time passed since last QMax DOD value is saved. hr/16
    uint16_t temp_k;            // Thermal Model temperature factor
    uint16_t temp_a;            // Thermal Model temperature
    uint16_t DOD0_raw_1;        // Raw Depth of discharge for Cell 1
    uint16_t DOD0_raw_2;        // Raw Depth of discharge for Cell 2
    uint16_t DOD0_raw_3;        // Raw Depth of discharge for Cell 3
    uint16_t DOD0_raw_4;        // Raw Depth of discharge for Cell 4

    //===========0x0076 CBSTATUS======================================
    uint8_t  CBSTATUS;            //alling status bitfield (1 byte) + 30 bytes of cell balancing time and DoD data
    uint8_t  CBSTATUS_size;
    uint16_t cell_bal_time_1;     // Cell balancing time for cell 1 in minutes   
    uint16_t cell_bal_time_2;     // Cell balancing time for cell 2 in minutes   
    uint16_t cell_bal_time_3;     // Cell balancing time for cell 3 in minutes 
    uint16_t cell_bal_time_4;     // Cell balancing time for cell 4 in minutes   
    uint16_t cell_bal_time_5;     // Cell balancing time for cell 5 in minutes   
    uint16_t cell_bal_time_6;     // Cell balancing time for cell 6 in minutes   
    uint16_t cell_bal_time_7;     // Cell balancing time for cell 7 in minutes   
    uint16_t cell_dod_1;          // Cell depth of discharge for cell 1 in percentage 
    uint16_t cell_dod_2;          // Cell depth of discharge for cell 2 in percentage 
    uint16_t cell_dod_3;          // Cell depth of discharge for cell 3 in percentage 
    uint16_t cell_dod_4;          // Cell depth of discharge for cell 4 in percentage 
    uint16_t cell_dod_5;          // Cell depth of discharge for cell 5 in percentage 
    uint16_t cell_dod_6;          // Cell depth of discharge for cell 6 in percentage 
    uint16_t cell_dod_7;          // Cell depth of discharge for cell 7 in percentage 
    uint16_t cell_dod_total;      // Total depth of discharge in percentage


    //===========0x007B DAStatus3======================================
    uint8_t  dastatus3;
    uint8_t  dastatus3_size;
    uint16_t cell_voltage_5;    
    uint16_t cell_current_5;    
    uint16_t cell_power_5;    
    uint16_t cell_voltage_6;    
    uint16_t cell_current_6;    
    uint16_t cell_power_6;   
    uint16_t cell_voltage_7;    
    uint16_t cell_current_7;    
    uint16_t cell_power_7;

    uint8_t No_response : 1; // Flag to indicate no response from BQ

    MANUFACTURE_STATUS_S manufacturing_status;
    OPERATION_STATUS_S operation_status; 
    GAUGING_STATUS_S gauging_status;
    uint8_t cells_count;               // Detected cells count for this BQ pack
}UVX_BQ_DATA;

typedef enum 
{
    /* --- Basic / Identification --- */
    BQ_MA_CHEMICAL_ID                     = 0x0006,
    BQ_MA_STATIC_CHEM_DF_SIGNATURE        = 0x0008,
    BQ_MA_ALL_DF_SIGNATURE                = 0x0009,

    /* --- Power / Modes --- */
    BQ_MA_SHUTDOWN_MODE                   = 0x0010,
    BQ_MA_SLEEP_MODE                      = 0x0011,
    BQ_MA_AUTO_CC_OFFSET                  = 0x0013,

    /* --- FET / Protection Control --- */
    BQ_MA_PDSG_FET_TOGGLE                 = 0x001C,
    BQ_MA_FUSE_TOGGLE                     = 0x001D,
    BQ_MA_PCHG_FET_TOGGLE                 = 0x001E,
    BQ_MA_CHG_FET_TOGGLE                  = 0x001F,
    BQ_MA_DSG_FET_TOGGLE                  = 0x0020,
    BQ_MA_GAUGING                         = 0x0021,
    BQ_MA_FET_CONTROL                     = 0x0022,

    /* --- Data / Logging --- */
    BQ_MA_LIFETIME_DATA_COLLECTION        = 0x0023,
    BQ_MA_PERMANENT_FAILURE               = 0x0024,
    BQ_MA_BLACK_BOX_RECORDER              = 0x0025,
    BQ_MA_FUSE                            = 0x0026,

    /* --- LED / UI --- */
    BQ_MA_LED_DISPLAY_ENABLE              = 0x0027,
    BQ_MA_LIFETIME_DATA_RESET             = 0x0028,
    BQ_MA_PF_DATA_RESET                   = 0x0029,
    BQ_MA_BLACK_BOX_RESET                 = 0x002A,
    BQ_MA_LED_TOGGLE                      = 0x002B,
    BQ_MA_LED_DISPLAY_PRESS               = 0x002C,

    /* --- Calibration / Test --- */
    BQ_MA_CALIBRATION_MODE                = 0x002D,
    BQ_MA_LIFETIME_DATA_FLUSH             = 0x002E,
    BQ_MA_LIFETIME_DATA_SPEED_UP          = 0x002F,

    /* --- Security --- */
    BQ_MA_SEAL_DEVICE                     = 0x0030,
    BQ_MA_SECURITY_KEYS                   = 0x0035,
    BQ_MA_AUTHENTICATION_KEY              = 0x0037,

    /* --- Reset --- */
    BQ_MA_DEVICE_RESET                    = 0x0041,

    /* --- Status / Alerts --- */
    BQ_MA_SAFETY_ALERT                    = 0x0050,
    BQ_MA_SAFETY_STATUS                   = 0x0051,
    BQ_MA_PF_ALERT                        = 0x0052,
    BQ_MA_PF_STATUS                       = 0x0053,
    BQ_MA_OPERATION_STATUS                = 0x0054,
    BQ_MA_CHARGING_STATUS                 = 0x0055,
    BQ_MA_GAUGING_STATUS                  = 0x0056,
    BQ_MA_MANUFACTURING_STATUS            = 0x0057,
    BQ_MA_AFE_REGISTER                    = 0x0058,
    BQ_MA_NO_LOAD_REM_CAP                 = 0x005A,

    /* --- Lifetime Data Blocks --- */
    BQ_MA_LIFETIME_DATA_BLOCK1             = 0x0060,
    BQ_MA_LIFETIME_DATA_BLOCK2             = 0x0061,
    BQ_MA_LIFETIME_DATA_BLOCK3             = 0x0062,
    BQ_MA_LIFETIME_DATA_BLOCK4             = 0x0063,
    BQ_MA_LIFETIME_DATA_BLOCK5             = 0x0064,

    /* --- Manufacturer Info / DA --- */
    BQ_MA_MANUFACTURER_INFO               = 0x0070,
    BQ_MA_DA_STATUS1                      = 0x0071,
    BQ_MA_DA_STATUS2                      = 0x0072,
    BQ_MA_DA_STATUS3                      = 0x007B,

    /* --- Gauging Status Extended --- */
    BQ_MA_GAUGING_STATUS1                 = 0x0073,
    BQ_MA_GAUGING_STATUS2                 = 0x0074,
    BQ_MA_GAUGING_STATUS3                 = 0x0075,
    BQ_MA_CB_STATUS                      = 0x0076,
    BQ_MA_STATE_OF_HEALTH                = 0x0077,
    BQ_MA_FILTER_CAPACITY                = 0x0078,
    BQ_MA_RSOC_WRITE                     = 0x0079,
    BQ_MA_MANUFACTURER_INFO_B            = 0x007A,
    BQ_MA_GAUGING_STATUS4                = 0x007C,
    BQ_MA_GAUGING_STATUS5                = 0x007D,

    /* --- Manufacturer Info Extended --- */
    BQ_MA_MANUFACTURER_INFO_C            = 0x0080,
    BQ_MA_MANUFACTURER_INFO_D            = 0x0081,
    BQ_MA_CURRENT_LONG                  = 0x0082,

    /* --- IATA --- */
    BQ_MA_IATA_SHUTDOWN                  = 0x00F0,
    BQ_MA_IATA_RM                        = 0x00F1,
    BQ_MA_IATA_FCC                       = 0x00F2,
    BQ_MA_IATA_CHARGE                   = 0x00F3,

    /* --- ROM / DF / Calibration Output --- */
    BQ_MA_ROM_MODE                      = 0x0F00,
    BQ_MA_WRITE_TEMP                    = 0x3008,
    BQ_MA_DATAFLASH_ACCESS_START        = 0x4000,  // 0x4000–0x5FFF
    BQ_MA_EXIT_CAL_OUTPUT_MODE          = 0xF080,
    BQ_MA_OUTPUT_CADC_CAL               = 0xF081,
    BQ_MA_OUTPUT_SHORTED_CCADC_CAL      = 0xF082,
    BQ_MA_OUTPUT_CCADC_CAL              = 0xF083

} UVX_BQ_MA_REGISTERS;


typedef enum
{
    /* 0x00–0x1F: Standard BQ Commands */
    MANUFACTURER_ACCESS            = 0x00,
    REMAINING_CAPACITY_ALARM       = 0x01,
    REMAINING_TIME_ALARM           = 0x02,
    BATTERY_MODE                   = 0x03,
    AT_RATE                        = 0x04,
    AT_RATE_TIME_TO_FULL           = 0x05,
    AT_RATE_TIME_TO_EMPTY          = 0x06,
    AT_RATE_OK                     = 0x07,
    TEMPERATURE                    = 0x08,
    VOLTAGE                        = 0x09,
    CURRENT                        = 0x0A,
    AVERAGE_CURRENT                = 0x0B,
    MAX_ERROR                      = 0x0C,
    RELATIVE_STATE_OF_CHARGE       = 0x0D,
    ABSOLUTE_STATE_OF_CHARGE       = 0x0E,
    REMAINING_CAPACITY             = 0x0F,
    FULL_CHARGE_CAPACITY           = 0x10,
    RUN_TIME_TO_EMPTY              = 0x11,
    AVERAGE_TIME_TO_EMPTY          = 0x12,
    AVERAGE_TIME_TO_FULL           = 0x13,
    CHARGING_CURRENT               = 0x14,
    CHARGING_VOLTAGE               = 0x15,
    BATTERY_STATUS                 = 0x16,
    CYCLE_COUNT                    = 0x17,
    DESIGN_CAPACITY                = 0x18,
    DESIGN_VOLTAGE                 = 0x19,
    SPECIFICATION_INFO             = 0x1A,
    MANUFACTURER_DATE              = 0x1B,
    SERIAL_NUMBER                  = 0x1C,

    /* 0x20–0x2F: BQ String / Extended */
    MANUFACTURER_NAME              = 0x20,
    DEVICE_NAME                    = 0x21,
    DEVICE_CHEMISTRY               = 0x22,
    MANUFACTURER_DATA              = 0x23,
    RESERVED_24                    = 0x24,
    RESERVED_25                    = 0x25,
    RESERVED_26                    = 0x26,
    RESERVED_27                    = 0x27,
    RESERVED_28                    = 0x28,
    RESERVED_29                    = 0x29,
    RESERVED_2A                    = 0x2A,
    RESERVED_2B                    = 0x2B,
    RESERVED_2C                    = 0x2C,
    RESERVED_2D                    = 0x2D,
    RESERVED_2E                    = 0x2E,
    AUTHENTICATE                   = 0x2F,

    /* 0x30–0x3F: TI Extensions / Cell Voltages */
    CELL_VOLTAGE5                  = 0x30,
    CELL_VOLTAGE6                  = 0x31,
    CELL_VOLTAGE7                  = 0x32,
    CELL_VOLTAGE8                  = 0x33,
    CELL_VOLTAGE9                  = 0x34,
    CELL_VOLTAGE10                 = 0x35,
    CELL_VOLTAGE11                 = 0x36,
    CELL_VOLTAGE12                 = 0x37,
    CELL_VOLTAGE13                 = 0x38,
    CELL_VOLTAGE14                 = 0x39,
    CELL_VOLTAGE15                 = 0x3A,
    CELL_VOLTAGE16                 = 0x3B,
    CELL_VOLTAGE4                  = 0x3C,
    CELL_VOLTAGE3                  = 0x3D,
    CELL_VOLTAGE2                  = 0x3E,
    CELL_VOLTAGE1                  = 0x3F,

    /* 0x40–0x4F: Manufacturer / GPIO */
    MANUFACTURER_BLOCK_ACCESS      = 0x44,
    GPIO_WRITE                     = 0x49,
    GPIO_READ                      = 0x4A,
    STATE_OF_HEALTH                = 0x4F,

    /* 0x50–0x5F: Status / Alerts (per TI documentation) */
    SAFETY_ALERT                  = 0x50,
    SAFETY_STATUS                 = 0x51,
    PF_ALERT                      = 0x52,
    PF_STATUS                     = 0x53,
    OPERATION_STATUS              = 0x54,
    CHARGING_STATUS               = 0x55,
    GAUGING_STATUS                = 0x56,
    MANUFACTURING_STATUS          = 0x57,
    AFE_REGISTERS                 = 0x58,
    RESERVED_59                    = 0x59,
    RESERVED_5A                    = 0x5A,
    RESERVED_5B                    = 0x5B,
    RESERVED_5C                    = 0x5C,
    RESERVED_5D                    = 0x5D,
    RESERVED_5E                    = 0x5E,
    RESERVED_5F                    = 0x5F,

    /* 0x60–0x82: Reserved / Manufacturer Specific */
    LIFETIME_DATA_BLOCK1          = 0x60,
    LIFETIME_DATA_BLOCK2          = 0x61,
    LIFETIME_DATA_BLOCK3          = 0x62,
    LIFETIME_DATA_BLOCK4          = 0x63,
    LIFETIME_DATA_BLOCK5          = 0x64,
    LIFETIME_DATA_BLOCK6          = 0x65,
    LIFETIME_DATA_BLOCK7          = 0x66,
    LIFETIME_DATA_BLOCK8          = 0x67,
    RESERVED_68                    = 0x68,
    RESERVED_69                    = 0x69,
    RESERVED_6A                    = 0x6A,
    RESERVED_6B                    = 0x6B,
    RESERVED_6C                    = 0x6C,
    RESERVED_6D                    = 0x6D,
    RESERVED_6E                    = 0x6E,
    RESERVED_6F                    = 0x6F,
    RESERVED_70                    = 0x70,
    DA_STATUS1                     = 0x71,
    DA_STATUS2                     = 0x72,
    RESERVED_73                    = 0x73,
    GAUGING_STATUS_2               = 0x74,
    GAUGING_STATUS_3               = 0x75,
    CBSTATUS                       = 0x76,
    RESERVED_77                    = 0x77,
    RESERVED_78                    = 0x78,
    RESERVED_79                    = 0x79,
    RESERVED_7A                    = 0x7A,
    DA_STATUS3                     = 0x7B,
    RESERVED_7C                    = 0x7C,
    RESERVED_7D                    = 0x7D,
    RESERVED_7E                    = 0x7E,
    RESERVED_7F                    = 0x7F,
    RESERVED_80                    = 0x80,
    RESERVED_81                    = 0x81,
    RESERVED_82                    = 0x82,
    END_REGISTER                   = 0xFF
}UVX_BQ_REGISTERS;

typedef struct 
{
  UVX_BQ_REGISTERS reg_addr;
  uint8_t size_data;
  uint8_t *p_data;
} UVX_BQ_REGISTER;

typedef struct UVX_COMM_BQ
{
	  uint8_t 		            ID;		  // I2C ID 
    UVX_I2C_HAL*            p_hal_i2c; // HAL I2C structure
    UVX_I2C*                i2c; // I2C structure
    uint8_t                 addr_i2c; // I2C device address
    UVX_BQ_REGISTER*       p_register_list; // Pointer to the register list

    uint8_t Enable 		        : 1; // Flag to indicate if I2C is enabled
    uint8_t is_Initilized 	    : 1; // Flag to indicate if HAL timer is initialied
    uint8_t Error 		        : 1; // Error flag
    uint8_t RX_Ready 	        : 1; // RX byte ready
    uint8_t TX_Ready 	        : 1; // TX byte ready
    uint8_t TX_Ready_Buffer     : 1; // TX buffer ready
    uint8_t RX_Ready_Buffer     : 1; // RX buffer ready        
    uint8_t Force_balance       : 1; // Reserved for future use
    uint8_t Force_balance_old   : 1; // Reserved for future use
    uint8_t owns_i2c_lock       : 1; // This BQ transaction owns the shared I2C lock

    uint16_t                buff_size_rx; // RX buffer size
    uint16_t                buff_size_tx; // TX buffer size
    uint16_t                buff_tx_cnt;  // RX buffer count
    uint16_t                buff_rx_cnt;  // TX buffer count
    uint8_t*                p_buff_tx_start; // Pointer to the start of the TX buffer
    uint8_t*                p_buff_rx_start; // Pointer to the start of the RX buffer
    uint8_t*                p_buff_tx; // Pointer to the TX buffer
    uint8_t*                p_buff_rx; // Pointer to the RX buffer  
    uint8_t                 i2c_tx_staging[UVX_BQ_I2C_TX_STAGING_SIZE]; // Must persist for HAL_I2C_Mem_Write_IT
    volatile uint8_t        busy_reason; // UVX_BQ_BUSY_REASON debugger diagnostic
    uint16_t                size_frame;
    uint16_t                size_payload;

}UVX_COMM_BQ;

typedef enum
{
  UVX_BQ_BUSY_NONE = 0,
  UVX_BQ_BUSY_OWNS_PREVIOUS_TRANSFER,
  UVX_BQ_BUSY_SHARED_I2C_LOCK,
  UVX_BQ_BUSY_I2C_TRANSFER_NOT_IDLE
} UVX_BQ_BUSY_REASON;

 typedef enum
 {
   UVX_BQ_OK = 0x00,
   UVX_BQ_ERROR,
   UVX_BQ_ERROR_INIT,
   UVX_BQ_ERROR_INIT_I2C,
   UVX_BQ_ERROR_BUSY,
   UVX_BQ_ERROR_UNKNOWN_CMD,
   UVX_BQ_ERROR_CRC,
   UVX_BQ_REG_END,
   UVX_BQ_TIMEOUT
 } UVX_COMM_BQ_STATE;

typedef enum 
{
    MODE_IDLE = 0x00,
    MODE_READ_DATA_ESC,
    MODE_READ_DATA_JMB,
    MODE_WRITE,    
    MODE_SEND,
    MODE_SEND_ERROR,
    MODE_WAIT_RESPONSE
} UVX_COMM_MODE;

typedef struct 
{
  UVX_COMM_MODE state_previous; // Previous state of the BQ communication
  UVX_COMM_MODE state_current;  // Current state of the BQ communication
  UVX_COMM_MODE state_next; // Next state of the BQ communication
} UVX_COMM_BQ_STATE_MACHINE;

extern SRAM1 UVX_BQ_DATA bq_data_l;
extern SRAM1 UVX_BQ_DATA bq_data_h;
extern SRAM1 UVX_COMM_BQ comm_bq_l; // BQ communication structure
extern SRAM1 UVX_COMM_BQ comm_bq_h; // BQ communication structure
extern SRAM1 UVX_COMM_BQ_STATE_MACHINE comm_bq_state;
extern UVX_COMM_BQ_STATE_MACHINE comm_state; // BQ communication state machine

extern uint8_t buff_tx_bq[BUFF_SIZE_TX_BQ]; // TX buffer for BQ communication
extern uint8_t buff_rx_bq[BUFF_SIZE_RX_BQ]; // RX buffer for BQ communication
extern UVX_BQ_REGISTER bq_l_register_list_read[]; // Array of BQ registers for read operations
extern UVX_BQ_REGISTER bq_l_register_list_read_once[]; // Array of BQ registers for one-time read operations
extern UVX_BQ_REGISTER bq_h_register_list_read[]; // Array of BQ registers for read operations
extern UVX_BQ_REGISTER bq_h_register_list_read_once[]; // Array of BQ registers for one-time read operations
extern UVX_BQ_DATA bq_data[BQ_DEVICES]; // BQ data structure
/* Function prototypes */
UVX_COMM_BQ_STATE uvx_comm_bq_init(UVX_COMM_BQ* p_comm_bq, UVX_I2C* i2c, uint8_t i2c_addr, UVX_BQ_REGISTER* p_register_list);
UVX_COMM_BQ_STATE uvx_comm_bq_change_list(UVX_COMM_BQ* p_comm_bq, UVX_BQ_REGISTER* p_register_list);
UVX_COMM_BQ_STATE uvx_comm_bq_read_list(UVX_COMM_BQ* p_comm_bq, uint16_t reg_index);
UVX_COMM_BQ_STATE uvx_comm_bq_read_register (UVX_COMM_BQ* p_comm_bq, UVX_BQ_REGISTERS reg_addr);
UVX_COMM_BQ_STATE uvx_comm_bq_read_ma_register(UVX_COMM_BQ* p_comm_bq, UVX_BQ_REGISTERS reg_addr);
UVX_COMM_BQ_STATE uvx_comm_bq_get_index_register(UVX_BQ_REGISTER *list, UVX_BQ_REGISTERS reg_addr, uint8_t* p_index);
UVX_COMM_BQ_STATE uvx_comm_bq_write_register(UVX_COMM_BQ* p_comm_bq, uint16_t reg_addr, uint8_t* data, uint16_t size) ;
UVX_COMM_BQ_STATE uvx_comm_bq_write_mba_register(UVX_COMM_BQ* p_comm_bq, UVX_BQ_MA_REGISTERS reg_addr, uint8_t* data, uint16_t size);
UVX_COMM_BQ_STATE uvx_comm_bq_send(uint8_t cmd_echo, uint8_t* data, uint16_t size);
UVX_COMM_BQ_STATE uvx_comm_bq_send_error(uint8_t data);
UVX_COMM_BQ_STATE uvx_comm_bq_process_rx(uint8_t byte_rx);
UVX_COMM_BQ_STATE uvx_comm_bq_process_rx_data(void);
UVX_COMM_BQ_STATE uvx_comm_bq_force_balance(UVX_COMM_BQ* p_comm_bq, uint8_t enable);
UVX_COMM_BQ_STATE uvx_comm_bq_charge_fet(UVX_COMM_BQ* p_comm_bq, uint8_t state);
uint8_t uvx_comm_bq_swap_u8(uint8_t* v);
uint16_t uvx_comm_bq_swap_u16_pointer(uint16_t* v);
uint16_t uvx_comm_bq_swap_u16_value(uint16_t v);
uint32_t uvx_comm_bq_swap_u32_pointer(uint32_t* v);
uint32_t uvx_comm_bq_swap_u32_value(uint32_t v);


#endif /* __UVX_COMM_BQ_H */
