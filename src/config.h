#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <DallasTemperature.h>

// #define TIME_ZONE_OFFSET_HRS            (-7)  /* Ensenada, México */
#define TIME_ZONE_OFFSET_HRS            (+8)   /* Taiping, Malaysia */

// temperature acquisition filter 
#define HIGH_TEMP_LIMIT 60
#define LOW_TEMP_LIMIT -40

// setting PWM properties  
#define AIR_PIN     0  

//------------ IO's    -------------------------------------------------------------------->
#define STAGE_1_IO  0   
#define STAGE_2_IO  1   
#define STAGE_3_IO  2   
#define VALVE_IO    3   
#define FAN_IO      4   

#define A0_5  27                            

#define STOP_IO     0    
#define DLY_S_IO    1    
#define START_IO    2    

#define A0    13 //ONE_WIRE_BUS  

#define TA_AI       0
#define TS_AI       1
#define TC_AI       2

#define BUFFER_SIZE 60 

// Temperature Sensors settings          -------------------------------------------------------------------->
#define ONE_WIRE_BUS 13// Data wire is plugged into port 2 on the Arduin
#define TEMPERATURE_PRECISION 12

#define TIME_ACQ_DELAY 10000 //in ms the delay between temperature value refresh
#define AVG_RESOLUTION 1000   //in ms the sampling for the Ts measure

DeviceAddress ADDRESS_TA = { 0x28, 0x8C, 0x4B, 0xAD, 0x27, 0x19, 0x01, 0xCA }; // Ta
DeviceAddress ADDRESS_TS = { 0x28, 0x78, 0x98, 0x8B, 0x0B, 0x00, 0x00, 0x22 }; // Ts
DeviceAddress ADDRESS_TC = { 0x28, 0xDA, 0xB6, 0xF7, 0x3A, 0x19, 0x01, 0x85 }; // Tc 
DeviceAddress ADDRESS_TI = { 0x28, 0x5A, 0xD3, 0x2A, 0x0D, 0x00, 0x00, 0x94 }; // Ti

#define IR_SENSOR_ADDRESS 0x5A
#define READ_TEMPERATURE 0x07

//---- timing settings -----////////////////////////////////////////////////////////////////////////////////

#define MINS 60000


//----- RTD's settings -----////////////////////////////////////////////////////////////////////////////////
#define RREF      400.0
// The 'nominal' 0-degrees-C resistance of the sensor
// 100.0 for PT100, 1000.0 for PT1000
#define RNOMINAL  100.0


#define TEMPERATURE_MIN  -50 // Minimum temperature value (in Celsius)
#define TEMPERATURE_MAX  150
#define ADC__RESOLUTION  65535 
#define REFERENCE 3.3

const float voltage_per_step = REFERENCE / ADC__RESOLUTION;
const int16_t range = TEMPERATURE_MAX - TEMPERATURE_MIN;
const double temperature_per_step = range / REFERENCE;

//------------ structure definitions an flags -------------------------------------------------------->
// Fan F1 and sprinkler S1 value
typedef struct { float M_F1; }                            data_F1;

typedef struct { float M_S1; }                            data_S1;

//stage
typedef struct { float stage; }                           data_stage;

// A and B variables
typedef struct { float N_A; float N_B; }                  data_SP;

// PID variables
typedef struct { float PID_output; }                      data_PIDO;

typedef struct { float PID_setpoint; }                    data_setpoint;

typedef struct { float N_P; float N_I; float N_D; }       data_PID;

// Ts and Tc target value
typedef struct { float N_ts_set; float N_tc_set; }        data_tset;

// fan (F1) STAGE 1 on and off time 
typedef struct { float N_f1_st1_ontime; float N_f1_st1_offtime; }                 data_st1;

// RTC
typedef struct { float N_hours; float N_minutes; float N_day; float N_month; }    data_rtc;

// temperature measures
typedef struct { float Ta_N; float Ts_N; float Tc_N; float Ti_N; float AvgTs_N; } data_s;


// fan (F1) and sprinklers (S1) STAGE 2 on and off time 
typedef struct { float N_f1_st2_ontime; float N_f1_st2_offtime; float N_s1_st2_ontime; float N_s1_st2_offtime; } data_st2;

// fan (F1) and sprinklers (S1) STAGE 3 on and off time 
typedef struct { float N_f1_st3_ontime; float N_f1_st3_offtime; float N_s1_st3_ontime; float N_s1_st3_offtime; } data_st3;

#endif
