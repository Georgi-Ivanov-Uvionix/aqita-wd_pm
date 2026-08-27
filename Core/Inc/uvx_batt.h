
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UVX_BATT
#define __UVX_BATT

#include "uvx_crc8.h"
#include "uvx_comm_bq.h"
#include "main.h"

#define	TIMERS_PRESCALER_FOR_SLEEP_MODE				50-1	
#define	TIMERS_PRESCALER_FOR_RUN_MODE				40000-1
#define TIM7_PRESCALER_FOR_RUN_MODE					800-1

#define TIM7_PERIOD									50000

#define BATTERIES_COMMUNICATION_PERIOD				20	// Timer autoreload value defining the time interval between battery communication packets
#define BATTERIES_COMMUNICATION_DT_MSEC				( ( BATTERIES_COMMUNICATION_PERIOD * 1000 ) / ( SystemCoreClock / ( TIMERS_PRESCALER_FOR_RUN_MODE + 1 ) ) )

#define CURRENT_FOR_ONE_VOLT						39.877f	
#define BATTERY_VOLTAGE_MIN							20.0f	// For all four batteries combined
#define	BATTERY_VOLTAGE_MAX							34.0f	// For all four batteries combined

#define BATT_MIN_DATA_VALID_CNT						3

#define BATT_SENT_DATA_PACKET_SIZE 					8
#define BATT_RCVD_DATA_PACKET_SIZE 					8

#define BATT_LED_MODE_SOLID_COLOR					0
#define BATT_LED_MODE_BREATHING						1

#define BATT_ESTIMATED_CELL_CAPACITY_ASEC			10800
#define BATT_ESTIMATED_CELL_RESISTANCE				0.07f
#define BATT_OCV_TO_SOC_POLY_COEFFS					11
#define BATT_SOC_TO_OCV_POLY_COEFFS					12
#define BATT_OCV_FILTER_TIME_CNST_MSEC				3000.0f
#define BATT_SOC_EST_FILTER_TIME_CNST_MSEC			3000.0f
#define BATT_SOC_EST_GAIN							6.25f

// Battery cell types unique identifiers
#define BATT_CELL_TYPE_UNDEFINED					0
#define BATT_CELL_TYPE_LGINR18650MJ1				1

#define BATT_CELLS_MAX                              10
#define BATT_EXPECTED_CELLS                         9
#define BATT_CELL_MIN_DETECT_VOLTAGE_MV             1500
#define BATT_CELL_MIN_VOLTAGE                       2200 //mv
#define BATT_CELL_MAX_VOLTAGE                       4200 //mv
#define BATT_CELL_CHARGE_RESUME_VOLTAGE             4100 //mv
#define BATT_CELL_DETECT_THRESHOLD_MV               1500 //mv
#define BATT_CELL_VOLTAGE_DIFF                      10  //mv
#define BATT_DELTA_VOLTAGE                          500  //mv
#define BATT_ADC_PACK_V_LOW_MIN_MS                  5000U

#define BATT_SUPPLY_STATUS_PWR_FET_BIT                 0
#define BATT_SUPPLY_STATUS_CHG_FET_BIT                 1
#define BATT_SUPPLY_STATUS_TC_BIT                      2
#define BATT_SUPPLY_STATUS_BALANCE_BIT                 3
#define BATT_SUPPLY_STATUS_FORCE_BALANCE_L_BIT         4
#define BATT_SUPPLY_STATUS_FORCE_BALANCE_H_BIT         5
#define BATT_SUPPLY_STATUS_STABLE_L_BIT                6
#define BATT_SUPPLY_STATUS_STABLE_H_BIT                7

#define BATT_ERROR_BQ_1_NO_RESPONSE_BIT                0
#define BATT_ERROR_BQ_2_NO_RESPONSE_BIT                1
#define BATT_ERROR_BATT_MAX_TEMP_BIT                   2
#define BATT_ERROR_CHARGE_OVERVOLTAGE_BIT              3
#define BATT_ERROR_CHARGE_CELL_COUNT_ERROR_BIT         4

#define MAX_CELL_TEMPERATURE                        650
#define MAX_HIS_CELL_TEMPERATURE                    600 //max histeresis cell temperature for power on after high temp cutoff
#define MIN_HIS_CELL_TEMPERATURE                    500 //min histeresis cell temperature for power on after high temp cutoff
#define SOC_START_LOW_POWER                         95

#define KELVIN_TO_DEG_C                             2732

#define SRAM1 __attribute__((section(".sram1")))
#define SRAM2 __attribute__((section(".sram2")))

#define CELL_1_INTERCONNECT_RESISTANCE 15 //mOhm
#define CELL_2_INTERCONNECT_RESISTANCE 0 //mOhm
#define CELL_3_INTERCONNECT_RESISTANCE 0 //mOhm
#define CELL_4_INTERCONNECT_RESISTANCE 0 //mOhm
#define CELL_5_INTERCONNECT_RESISTANCE 1 //mOhm
#define CELL_6_INTERCONNECT_RESISTANCE 12 //mOhm
#define CELL_7_INTERCONNECT_RESISTANCE 0 //mOhm
#define CELL_8_INTERCONNECT_RESISTANCE 0 //mOhm
#define CELL_9_INTERCONNECT_RESISTANCE 0 //mOhm
#define CELL_10_INTERCONNECT_RESISTANCE 0 //mOhm


typedef packed_struct
{
    uint8_t  SOH;                       // State of Health in percentage (0-100%)       byte 1
    uint16_t batt_voltage;              // Battery Voltage in millivolts                bytes 2-3
    int16_t  current;                   // Current in milliamps                         bytes 4-5
    uint16_t remaining_capacity;        // Remaining Capacity in mAh                      bytes 6-7
    uint16_t full_charge_capacity;      // Full Charge Capacity in mAh                  bytes 8-9
    uint16_t design_capacity;           // Design Capacity in mAh                       bytes 10-11
    uint8_t  relative_state_of_charge;  // Relative State of Charge in percentage (0-100%) bytes 12-13
    uint8_t  supply_status;             //bit 0 - pwr fet byte 14
                                        //bit 1 - chg fet
                                        //bit 2 - tc
                                        //bit 3 - balance
                                        //bit 4 - force balance l
                                        //bit 5 - force balance h
                                        //bit 6 - stable l
                                        //bit 7 - stable h
    uint16_t cell_voltage_1;            // Cell Voltage 1 in millivolts
    uint16_t cell_voltage_2;            // Cell Voltage 2 in millivolts
    uint16_t cell_voltage_3;            // Cell Voltage 3 in millivolts byte 20
    uint16_t cell_voltage_4;            // Cell Voltage 4 in millivolts
	uint16_t cell_voltage_5;            // Cell Voltage 5 in millivolts
	uint16_t cell_voltage_6;            // Cell Voltage 6 in millivolts
	uint16_t cell_voltage_7;            // Cell Voltage 7 in millivolts
	uint16_t cell_voltage_8;            // Cell Voltage 8 in millivolts  byte 30
	uint16_t cell_voltage_9;            // Cell Voltage 9 in millivolts
	uint16_t cell_voltage_10;           // Cell Voltage 10 in millivolts            
    uint16_t temperature_cell_1;    // Temperature of cell 1 in tenths of degrees Celsius
    uint16_t temperature_cell_2;    
    uint16_t temperature_cell_3;    //byte 40
    uint16_t temperature_cell_4; 
    uint16_t temperature_cell_5;    
    uint16_t temperature_cell_6;    
    uint16_t temperature_cell_7;    
    uint16_t temperature_cell_8;             //byte 50    
    uint16_t temperature_l_int;       
    uint16_t temperature_h_int;       
    int16_t  voltage_diff_pack;         // voltage difference between two packs in millivolts
    int16_t  voltage_delta_cell;        // voltage difference between cells in millivolts
    int16_t  voltage_min_cell;          // voltage difference between cells in millivolts byte 60
    int16_t  voltage_max_cell;          // voltage difference between cells in millivolts      
    uint16_t cell_bal_time_1;           // Cell balancing time for cell 1 in minutes
    uint16_t cell_bal_time_2;           // Cell balancing time for cell 2 in minutes
    uint16_t cell_bal_time_3;           // Cell balancing time for cell 3 in minutes
    uint16_t cell_bal_time_4;           // Cell balancing time for cell 4 in minutes byte 70
    uint16_t cell_bal_time_5;           // Cell balancing time for cell 5 in minutes
    uint16_t cell_bal_time_6;           // Cell balancing time for cell 6 in minutes
    uint16_t cell_bal_time_7;           // Cell balancing time for cell 7 in minutes
    uint16_t cell_bal_time_8;           // Cell balancing time for cell 8 in minutes
    uint16_t cell_bal_time_9;           // Cell balancing time for cell 9 in minutes byte 80
    uint16_t cell_bal_time_10;          // Cell balancing time for cell 10 in minutes    
    uint8_t absolute_state_of_charge;  // Absolute State of Charge in percentage (0-100%)
    int32_t  power;                     //Power calculated by Voltage() × Current()                                  
    uint16_t  cycle_count;               // Number of charge-discharge cycles byte 89
    uint16_t adc_pack_v;                // measured pack voltage in millivolts byte 90 - 91
    uint16_t avg_time_to_empty_m;       // Average Time to Empty in minutes
    uint16_t avg_time_to_full_m;        // Average Time to Full in minutes    
    uint16_t state_time_l_m;            // Time passed since last state change (DISCHARGE, CHARGE, REST)  
    uint16_t state_time_h_m;            // Time passed since last state change (DISCHARGE, CHARGE, REST)        byte 99
    uint8_t  error;                     //bit 0 - BQ 1 no response      byte 100
                                        //bit 1 - BQ 2 no response
                                        //bit 2 - reserved
                                        //bit 3 - reserved
                                        //bit 4 - reserved
                                        //bit 5 - reserved
                                        //bit 6 - reserved
                                        //bit 7 - reserved
    int16_t  Qmax_passed_BQ_1;              // Qmax value from BQ 1 in mAh byte 102                                        
    int16_t  Qmax_passed_BQ_2;              // Qmax value from BQ 2 in mAh byte 104                                       
} BATT_DATA_PAYLOAD;

typedef struct
{
    uint8_t  adc_pack_v_stable_high;
    uint8_t  adc_pack_v_stable_low;     
    uint8_t  cells_count;
    uint16_t pwr_min_voltage;
    uint16_t pwr_max_voltage;

    uint8_t  SOH;                       // State of Health in percentage (0-100%)       byte 1
    uint16_t batt_voltage;              // Battery Voltage in millivolts                bytes 2-3
    int16_t  current;                   // Current in milliamps                         bytes 4-5
    int32_t  power;                     //Power calculated by Voltage() × Current()  
    uint16_t remaining_capacity;        // Remaining Capacity in mAh                      bytes 6-7
    uint8_t relative_state_of_charge;  // Relative State of Charge in percentage (0-100%) bytes 12-13    
    uint16_t full_charge_capacity;      // Full Charge Capacity in mAh                  bytes 8-9
    uint16_t design_capacity;           // Design Capacity in mAh                       bytes 10-11
    uint8_t absolute_state_of_charge;  // Absolute State of Charge in percentage (0-100%)                   
    uint8_t  supply_status;             //bit 0 - pwr fet byte 14
                                        //bit 1 - chg fet
                                        //bit 2 - tc
                                        //bit 3 - balance
                                        //bit 4 - force balance l
                                        //bit 5 - force balance h
                                        //bit 6 - stable l
                                        //bit 7 - stable h
    uint16_t cell_voltage_1;            // Cell Voltage 1 in millivolts
    uint16_t cell_voltage_2;            // Cell Voltage 2 in millivolts
    uint16_t cell_voltage_3;            // Cell Voltage 3 in millivolts byte 20
    uint16_t cell_voltage_4;            // Cell Voltage 4 in millivolts
	uint16_t cell_voltage_5;            // Cell Voltage 5 in millivolts
	uint16_t cell_voltage_6;            // Cell Voltage 6 in millivolts
	uint16_t cell_voltage_7;            // Cell Voltage 7 in millivolts
	uint16_t cell_voltage_8;            // Cell Voltage 8 in millivolts  byte 30
	uint16_t cell_voltage_9;            // Cell Voltage 9 in millivolts
	uint16_t cell_voltage_10;           // Cell Voltage 10 in millivolts            
    uint16_t temperature_cell_1;    // Temperature of cell 1 in tenths of degrees Celsius
    uint16_t temperature_cell_2;    
    uint16_t temperature_cell_3;    //byte 40
    uint16_t temperature_cell_4; 
    uint16_t temperature_cell_5;    
    uint16_t temperature_cell_6;    
    uint16_t temperature_cell_7;    
    uint16_t temperature_cell_8;             //byte 50    
    uint16_t temperature_l_int;       
    uint16_t temperature_h_int;       
    int16_t  voltage_diff_pack;         // voltage difference between two packs in millivolts
    int16_t  voltage_delta_cell;        // voltage difference between cells in millivolts
    int16_t  voltage_min_cell;          // voltage difference between cells in millivolts byte 60
    int16_t  voltage_max_cell;          // voltage difference between cells in millivolts      
    uint16_t cell_bal_time_1;           // Cell balancing time for cell 1 in minutes
    uint16_t cell_bal_time_2;           // Cell balancing time for cell 2 in minutes
    uint16_t cell_bal_time_3;           // Cell balancing time for cell 3 in minutes
    uint16_t cell_bal_time_4;           // Cell balancing time for cell 4 in minutes byte 70
    uint16_t cell_bal_time_5;           // Cell balancing time for cell 5 in minutes
    uint16_t cell_bal_time_6;           // Cell balancing time for cell 6 in minutes
    uint16_t cell_bal_time_7;           // Cell balancing time for cell 7 in minutes
    uint16_t cell_bal_time_8;           // Cell balancing time for cell 8 in minutes
    uint16_t cell_bal_time_9;           // Cell balancing time for cell 9 in minutes byte 80
    uint16_t cell_bal_time_10;          // Cell balancing time for cell 10 in minutes                     
    uint16_t  cycle_count;               // Number of charge-discharge cycles byte 89
    uint16_t adc_pack_v;                // measured pack voltage in millivolts byte 90 - 91
    uint16_t avg_time_to_empty_m;       // Average Time to Empty in minutes
    uint16_t avg_time_to_full_m;        // Average Time to Full in minutes    
    uint16_t state_time_l_m;            // Time passed since last state change (DISCHARGE, CHARGE, REST)  
    uint16_t state_time_h_m;            // Time passed since last state change (DISCHARGE, CHARGE, REST)        byte 99
    uint8_t  error;                     //bit 0 - BQ 1 no response      byte 100
                                        //bit 1 - BQ 2 no response
                                        //bit 2 - reserved
                                        //bit 3 - reserved
                                        //bit 4 - reserved
                                        //bit 5 - reserved
                                        //bit 6 - reserved
                                        //bit 7 - reserved
    int16_t  Qmax_passed_BQ_1;              // Qmax value from BQ 1 in mAh byte 102                                        
    int16_t  Qmax_passed_BQ_2;              // Qmax value from BQ 2 in mAh byte 104  

//--------------------------------DATA FOR JETSON-------------------------------------------    
    BATT_DATA_PAYLOAD payload;                                    
    uint16_t size_payload;
//-----------------------------------------------------------------------------------
    uint16_t temperature_cell_l;    //lowest cell temperature in 0.1 degree Celsius
    uint16_t temperature_cell_h;    //highest cell temperature in 0.1 degree Celsius  
    uint16_t design_voltage;      
    int16_t cell_current_1;          // Cell Current 1 in milliamps
    int16_t cell_current_2;          // Cell Current 2 in milliamps
    int16_t cell_current_3;          // Cell Current 3 in milliamps
    int16_t cell_current_4;          // Cell Current 4 in milliamps
	int16_t cell_current_5;          // Cell Current 5 in milliamps
	int16_t cell_current_6;          // Cell Current 6 in milliamps
	int16_t cell_current_7;          // Cell Current 7 in milliamps
	int16_t cell_current_8;          // Cell Current 8 in milliamps
	int16_t cell_current_9;          // Cell Current 9 in milliamps
	int16_t cell_current_10;         // Cell Current 10 in milliamps

    int16_t cell_power_1;          // Cell Power 1 in milliwatts
    int16_t cell_power_2;          // Cell Power 2 in milliwatts
    int16_t cell_power_3;          // Cell Power 3 in milliwatts
    int16_t cell_power_4;          // Cell Power 4 in milliwatts
	int16_t cell_power_5;          // Cell Power 5 in milliwatts
	int16_t cell_power_6;          // Cell Power 6 in milliwatts
	int16_t cell_power_7;          // Cell Power 7 in milliwatts
	int16_t cell_power_8;          // Cell Power 8 in milliwatts
	int16_t cell_power_9;          // Cell Power 9 in milliwatts
	int16_t cell_power_10;         // Cell Power 10 in milliwatts


    uint16_t cell_dod_1;               // Cell depth of discharge for cell 1 in percentage
    uint16_t cell_dod_2;               // Cell depth of discharge for cell 2 in percentage
    uint16_t cell_dod_3;               // Cell depth of discharge for cell 3 in percentage
    uint16_t cell_dod_4;               // Cell depth of discharge for cell 4 in percentage
    uint16_t cell_dod_5;               // Cell depth of discharge for cell 5 in percentage
    uint16_t cell_dod_6;               // Cell depth of discharge for cell 6 in percentage
    uint16_t cell_dod_7;               // Cell depth of discharge for cell 7 in percentage
    uint16_t cell_dod_8;               // Cell depth of discharge for cell 8 in percentage
    uint16_t cell_dod_9;               // Cell depth of discharge for cell 9 in percentage
    uint16_t cell_dod_10;              // Cell depth of discharge for cell 10 in percentage


    uint32_t CHG_fet_en   : 1;
    uint32_t CHG_fet_stat : 1;
    uint32_t init     : 1;
    uint32_t tc       : 1; 
    uint32_t cell_ball_h : 1;
    uint32_t cell_ball_l : 1;
    uint32_t batt_max_temp : 1;
   
    uint16_t specification_info;       // Specification Information
    uint16_t manufacturer_date;        // Manufacturer Date
    uint16_t serial_number;            // Serial Number
    uint32_t cnt_no_response;
        
}UVX_BATT_DATA;

 typedef enum
 {
   UVX_BATT_OK = 0x00,
   UVX_BATT_ERROR,
   UVX_BATT_ERROR_INIT,
   UVX_BATT_ERROR_INIT_I2C,
   UVX_BATT_ERROR_BUSY,
   UVX_BATT_ERROR_UNKNOWN_CMD,
   UVX_BATT_ERROR_CRC,
   UVX_BATT_REG_END,
   UVX_BATT_TIMEOUT
 } UVX_BATT_STATE;



typedef enum 
{
    BATT_MODE_INIT = 0x00,
    BATT_MODE_READ_ONCE_BQ_1,
	BATT_MODE_READ_ONCE_BQ_2,
    BATT_MODE_INIT_BALANCE_L,
    BATT_MODE_INIT_BALANCE_H,    
    BATT_MODE_OFF_BALANCE_L,    
    BATT_MODE_OFF_BALANCE_H,     
    BATT_MODE_READ_BQ_1,
	BATT_MODE_READ_BQ_2,	
    BATT_MODE_CHECK_STATUS,
	BATT_MODE_READ_CHECK_PACK_V,	
	BATT_MODE_READ_CHECK_PACK_V_STABLE,	
    BATT_MODE_STOP,
    BATT_MODE_WAIT_RESPONSE
} UVX_BATT_MODE;

typedef struct 
{
  UVX_BATT_MODE state_previous; // Previous state of the M2JMB communication
  UVX_BATT_MODE state_current;  // Current state of the M2JMB communication
  UVX_BATT_MODE state_next; // Next state of the M2JMB communication
} UVX_BATT_STATE_MACHINE;

extern UVX_BATT_STATE_MACHINE batt_state;
extern SRAM1 UVX_BATT_DATA batt_data;

UVX_BATT_STATE uvx_batt_parse_data(void);
UVX_BATT_STATE uvx_batt_read_data(uint16_t reg_index);
UVX_BATT_STATE uvx_batt_read_pack_v(void);
UVX_BATT_STATE uvx_batt_detect_cells(void);
UVX_BATT_STATE uvx_batt_search_cell_diff(void);
UVX_BATT_STATE uvx_batt_learn(void);

#endif

