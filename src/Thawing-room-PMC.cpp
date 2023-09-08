
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "WIFIUnified.h"
#include "RTClib.h"
#include "config.h"
#include <PID_v1.h>
#include <OneWire.h>
#include "secrets.h"
#include "MqttClient.h"
#include <Adafruit_ADS1X15.h>
#include <DallasTemperature.h>
#include <Arduino_MachineControl.h>

using namespace rtos;
using namespace machinecontrol;

data_rtc N_rtc;  // structure data_rtc from the config file is renamed N_rtc
data_st1 N_st1;  // fan (F1) STAGE 1 on and off time
data_st2 N_st2;  // fan (F1) and sprinklers (S1) STAGE 2 on and off time
data_st3 N_st3;  // fan (F1) and sprinklers (S1) STAGE 3 on and off time


// State of SPRINKLER 1
bool S1_state = 0;

// A & B variables
data_SP N_SP;

float A = 0;
float B = 0;
bool R_A = 0;
bool R_B = 0;

// PID variables
data_PIDO PID_data;           // value of the PID output
data_setpoint setpoint_data;  // value of the Setpoint

bool R_P = 0;
bool R_I = 0;
bool R_D = 0;

double Output;    // PWM signal and converter
double PIDinput;  // temp sensor
double Setpoint;  // will be the desired value

// PID parameters
double Kp = 0, Ki = 10, Kd = 0;

double coefOutput = 0;  // Output for the infeed (New Analog Output that will be sent to S1
uint8_t coefPID = 100;
float Output_float = 0.0;
uint8_t Converted_Output = 0;

// Temperatures measures of Ta, Ts, Tc, Ti & AvgTs
data_s temp_data;

// Ts & Tc target value
data_tset N_tset;
bool stop_temp1 = 0;
bool stop_temp2 = 0;

// Fan F1 value (1 parameter)
data_F1 F1_data;

// Sprinkler S1 value (1 parameter)
data_S1 S1_data;

// Start, delayed start, stop, and choose Ts
uint8_t N_stop = 0;
uint8_t N_start = 0;
uint8_t N_d_start = 0;
uint8_t N_chooseTs = 0;

bool STOP = 0;
bool START = 0;

bool START1 = 0;     // delayed start bttn
bool START2 = 0;     // start bttn
bool C1_state = 0;   // State of Stage 1
bool C2_state = 0;   // State of Stage 2
bool C3_state = 0;   // State of Stage 3
bool MTR_State = 0;  // State of the motor that control the Fan F1

// State of the Stage (data = 1, 2 or 3)
data_stage stage_data;
uint8_t stage = 0;

// Parameters of Stage 2
uint8_t Stage2_hour = 0;
uint8_t Stage2_minute = 0;
uint8_t Stage2_day = 0;
uint8_t Stage2_month = 0;

bool Stage2_RTC_set = 0;
bool Stage2_started = 0;
bool Stage3_started = 0;

// ########################### Timers ##########################
uint32_t F1_timer = 0UL;               // fan F1 timing
uint32_t pid_computing_timer = 0UL;    // PID computing timing
uint32_t F1_stg_2_timmer = 0UL;        // F1 stage 2 timing
uint32_t S1_stg_2_timer = 0UL;         // S1 stage 2 timing
uint32_t F1_stg_3_timer = 0UL;         // F1 stage 3 timing
uint32_t S1_stg_3_timer = 0UL;         // S1 stage 3 timing
uint32_t get_temp_timer = 0UL;         // temperature acquisition
uint32_t ts_avg_timer = 0UL;           // Ts average timing
uint32_t stg_2_pid_timer = 0UL;        // stage 2 PID
uint32_t turn_on_pid_timer = 0UL;      // stage 2 PID ON
uint32_t turn_off_pid_timer = 0UL;     // stage 2 PID OFF
uint32_t address_sending_timer = 0UL;  // Address Sending
uint32_t A_B_timer = 0UL;              // Stage ON/OFF and A&B PUBLISH


// ######################## Temperature ########################
float TA = 0, TA_F = 0;  //Ta
float TS = 0, TS_F = 0;  //Ts
float TC = 0, TC_F = 0;  //Tc
float TI = 0, TI_F = 0;  //Ti optional

// ########################### Buffer ##########################
float avg_ts = 0.0;              // average surface temperature
float buffer_sum = 0;            // variable to store the buffer_sum of the received values
float fifo_buffer[BUFFER_SIZE] = {};  // buffer to store the values
uint8_t buffer_len = 0;
uint8_t buffer_index = 0;  // buffer index

WIFIUnified wifi;
MqttClient mqtt;
RTC_PCF8563 rtc;
Adafruit_ADS1015 analog_inputs;

PID air_in_feed_PID(&PIDinput, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // DIRECT or REVERSE

OneWire oneWire(ONE_WIRE_BUS);         // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
DallasTemperature sensors1(&oneWire);  // PASS our oneWire reference to Dallas Temperature.

//---- Function declaration ----/////////////////////////////////////////////////////////////////////////////
void setUpRTC();
float getIRTemp();
void stopRoutine();
void updateTemperature();
void setStage(int Stage);
void setUpDefaulParameters();
float getProbeTemp(uint8_t channel);
float readTempFrom(uint8_t channel);
String addressToString(uint8_t *address);
int responseToInt(byte *value, size_t len);
float responseToFloat(byte *value, size_t len);
void callback(char *topic, byte *payload, unsigned int len);  //callback function for mqtt, see definition after loop

void setup() {
  Serial.begin(115200);
  
  setUpDefaulParameters();

  Wire.begin();
  temp_probes.rtd.begin(THREE_WIRE);
  temp_probes.enableRTD();

  digital_outputs.setLatch();
  digital_outputs.setAll(0);

  setStage(0);

  analog_out.period_ms(AIR_PIN, 4);

  wifi.setUpWiFi();
  wifi.setUpOTA();

  delay(1000);
  
  wifi.checkVersion();
  // wifi.setUpWebServer();
  setUpRTC();

  mqtt.connect();
  mqtt.setCallback(callback);

  //Turn the PID on
  air_in_feed_PID.SetMode(AUTOMATIC);
  air_in_feed_PID.SetSampleTime(3000);
  //Adjust PID values


  air_in_feed_PID.SetTunings(Kp, Ki, Kd);
  Serial.println("paso todo");

  delay(750);
}

void loop() {
  DateTime now = rtc.now();

  if (!wifi.isConnected()) {
    wifi.reconnect();
    delay(2000);
    Serial.println("mamo");
    return;
  }

  if (mqtt.isServiceAvailable()) mqtt.loop();

  updateTemperature();

  // if (N_chooseTs == 1) TC2 = analogRead(A0);                                // Condition to choose if Ts is a IR sensor or OneWire sensor

  if ((TA) > (LOW_TEMP_LIMIT) && (TA) < (HIGH_TEMP_LIMIT)) TA_F = TA;  // if the temperature over the limit it will not be considered

  if ((TS) > (LOW_TEMP_LIMIT) && (TS) < (HIGH_TEMP_LIMIT)) TS_F = TS;

  if ((TC) > (LOW_TEMP_LIMIT) && (TC) < (HIGH_TEMP_LIMIT)) TC_F = TC;

  if ((TI) > (LOW_TEMP_LIMIT) && (TI) < (HIGH_TEMP_LIMIT)) TI_F = TI;

  if ((millis() - address_sending_timer >= 10000)) {

    String ta_string_address = addressToString(ADDRESS_TA);
    mqtt.publishData("mduino/sendadd1", ta_string_address);

    String ts_string_address = addressToString(ADDRESS_TS);
    mqtt.publishData("mduino/sendadd2", ts_string_address);

    String tc_string_address = addressToString(ADDRESS_TC);
    mqtt.publishData("mduino/sendadd3", tc_string_address);

    String ti_string_address = addressToString(ADDRESS_TI);
    mqtt.publishData("mduino/sendadd4", ti_string_address);

    address_sending_timer = millis();
  }

  //---- Get surface temperature average with a FIFO buffer ---- //////////////////////////////// Something fuckin' wrong with the average
  if (millis() - ts_avg_timer >= AVG_RESOLUTION) {
    
    if (buffer_len < BUFFER_SIZE) { //if buffer not full, we add the value
        buffer_sum += TS_F;
        fifo_buffer[buffer_len] = TS_F;
        buffer_len++;
      }
      else { //buffer full, we remove the oldest value and add the new one
        buffer_sum -= fifo_buffer[buffer_index];
        fifo_buffer[buffer_index] = TS_F;
        buffer_sum += TS_F;
        buffer_index = (buffer_index + 1) % BUFFER_SIZE; // update the buffer index
      }
      
      avg_ts = buffer_sum/buffer_len;

    mqtt.publishData(AVG_TS_TOPIC, temp_data.AvgTs_N);
    // Serial.println("Temp data published");
    ts_avg_timer = millis();
  }

  //---- Temperature MQTT publish ----///////////////////////////////////////////////////////////
  if (millis() - get_temp_timer >= TIME_ACQ_DELAY) {
    temp_data.Ta_N = TA_F;
    temp_data.Ts_N = TS_F;
    temp_data.Tc_N = TC_F;
    temp_data.Ti_N = TI_F;
    temp_data.AvgTs_N = avg_ts;

    mqtt.publishData(TA_TOPIC, temp_data.Ta_N);
    mqtt.publishData(TS_TOPIC, temp_data.Ts_N);
    mqtt.publishData(TC_TOPIC, temp_data.Tc_N);
    mqtt.publishData(TI_TOPIC, temp_data.Ti_N);

    // for debug purpose
    Serial.println("Average: " + String(temp_data.AvgTs_N));
    Serial.println("Ts: " + String(TS));
    Serial.println("TC: " + String(TC));
    Serial.println("Ta: " + String(TA));
    Serial.println("Nstart: " + String(N_start));
    Serial.println("Nstop: " + String(N_stop));
    Serial.println("A variable: " + String(N_SP.N_A));
    Serial.println("B variable: " + String(N_SP.N_B));
    Serial.println("P variable: " + String(Kp));
    Serial.println("I variable: " + String(Ki));
    Serial.println("D variable: " + String(Kd));
    Serial.println("setpoint raw: " + String(Setpoint));
    Serial.println("setpoint: " + String(setpoint_data.PID_setpoint));

    Serial.println("time: ");
    Serial.print(String(now.hour()) + "h ");
    Serial.println(String(now.minute()) + "min ");
    Serial.print(String(now.day()) + "day ");
    Serial.println(String(now.month()) + "month");
    Serial.println("stage 2 time: ");
    Serial.print(String(Stage2_hour) + "h ");
    Serial.print(String(Stage2_minute) + "min ");
    Serial.print(String(Stage2_day) + "day ");
    Serial.print(String(Stage2_month) + "month");

    get_temp_timer = millis();
  }

  //---- Time Stage ON/OFF and A & B MQTT Publish ----///////////////////////////////////////////////////////
  if (millis() - A_B_timer >= 10000) {
    // STAGE 1
    mqtt.publishData(ACK_F1_ST1_ONTIME, N_st1.N_f1_st1_ontime);
    mqtt.publishData(ACK_F1_ST1_OFFTIME, N_st1.N_f1_st1_offtime);

    // STAGE 2
    mqtt.publishData(ACK_F1_ST2_ONTIME, N_st2.N_f1_st2_ontime);
    mqtt.publishData(ACK_F1_ST2_OFFTIME, N_st2.N_f1_st2_offtime);
    mqtt.publishData(ACK_S1_ST2_ONTIME, N_st2.N_s1_st2_ontime);
    mqtt.publishData(ACK_S1_ST2_OFFTIME, N_st2.N_s1_st2_offtime);

    // STAGE 3
    mqtt.publishData(ACK_F1_ST3_ONTIME, N_st3.N_f1_st3_ontime);
    mqtt.publishData(ACK_F1_ST3_OFFTIME, N_st3.N_f1_st3_offtime);
    mqtt.publishData(ACK_S1_ST3_ONTIME, N_st3.N_s1_st3_ontime);
    mqtt.publishData(ACK_S1_ST3_OFFTIME, N_st3.N_s1_st3_offtime);

    // A & B
    mqtt.publishData(ACK_A, N_SP.N_A);
    mqtt.publishData(ACK_B, N_SP.N_B);

    A_B_timer = millis();
  }

  //---- PID Publishing ----//////////////////////////////////////////////////////////////////////
  // PID works only on STAGE 2
  if (stage == 2 && STOP == 0) {
    if (millis() - stg_2_pid_timer >= (TIME_ACQ_DELAY + 1)) {
      Serial.println("Soft PID Actual Output is" + String(Output));
      Output_float = float(coefOutput);
      PID_data.PID_output = ((Output_float - 0) / (255 - 0)) * (100 - 0) + 0;
      Serial.println("PID Output /100 is" + String(PID_data.PID_output));

      mqtt.publishData(PID_OUTPUT, PID_data.PID_output);
      stg_2_pid_timer = millis();
    }
  }

  //---- START, DELAYED, STOP Button pressed ----////////////////////////////////////////////////
  // delayed start push button or digital button pressed
  if (digital_inputs.read(DLY_S_IO) == 1 || N_d_start == 1) {
    START1 = 1;
    Serial.println("Delayed Start Pressed");
    N_d_start = 0;
    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("Stage 1 init M_S1 stop published");
  }

  // start push button or digital button pressed
  if (digital_inputs.read(START_IO) == 1 || N_start == 1) {
    START2 = 1;
    Serial.println("Start Pressed");
    N_start = 0;
    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("Stage 1 init M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("Stage 1 init M_S1 stop published");
  }

  // stop push button or digital button pressed
  if (digital_inputs.read(STOP_IO) == 1 || N_stop == 1) {
    STOP = 1;
    Serial.println("Stop Pressed");
    N_stop = 0;
  }

  //---- STOP ROUTINE ----///////////////////////////////////////////////////////////////////////
  if (STOP == 1) stopRoutine();
  //---- RTC Timer ----//////////////////////////////////////////////////////////////////////////

  if (((((now.hour() >= Stage2_hour && now.minute() >= Stage2_minute
          && now.day() >= Stage2_day && now.month() >= Stage2_month)
         && START1 == 1)
        || START2 == 1)
       && Stage2_started == 0 && Stage2_RTC_set == 0)) {

    START1 = MTR_State = C1_state = 0;
    digital_outputs.set(STAGE_1_IO, LOW);
    digital_outputs.set(STAGE_2_IO, LOW);
    digital_outputs.set(STAGE_3_IO, LOW);
    digital_outputs.set(VALVE_IO, LOW);
    digital_outputs.set(FAN_IO, LOW);

    F1_data.M_F1 = 2;
    S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("All M_F1 stop published ");

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("All M_S1 stop published");

    Serial.println("Stage 2 Initiated wait for 5 secs");
    Stage2_RTC_set = Stage2_started = 1;
    delay(5000);
  }

  //---- STAGE 1 ----////////////////////////////////////////////////////////////////////////////
  if (START1 == 1 && Stage2_RTC_set == 0 && STOP == 0) {
    if (C1_state == 0) {
      digital_outputs.set(STAGE_1_IO, HIGH);  // Turn On the LED of Stage 1
      C1_state = 1;                           // State of Stage 1 turned ON
      Serial.println("Stage 1 Started");
      setStage(1);
      Serial.println("Stage 1 Status Send packet ");
      F1_timer = millis() - (N_st1.N_f1_st1_ontime * MINS);
    }

    // Turn ON F1

    if (MTR_State == 0 && (HIGH != digital_inputs.read(FAN_IO)) && (millis() - F1_timer >= (N_st1.N_f1_st1_offtime * MINS))) {  // MTR_State is the motor of F1
      digital_outputs.set(FAN_IO, HIGH);                                                                                        // Turn ON F1
      Serial.println("Stage 1 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("Stage 1 init M_F1 ON published ");
      F1_timer = millis();
    }

    // Turn OFF F1 when the time set in the configuration is over
    if (MTR_State == 1 && (LOW != digital_inputs.read(FAN_IO)) && (millis() - F1_timer >= (N_st1.N_f1_st1_ontime * MINS))) {
      digital_outputs.set(FAN_IO, LOW);
      // analog_out.write(AIR_PIN, 0);
      Serial.println("Stage 1 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("Stage 1 init M_F1 OFF published ");
      F1_timer = millis();
    }
  }

  //---- STAGE 2 ----////////////////////////////////////////////////////////////////////////////
  if (Stage2_RTC_set == 1 && Stage3_started == 0 && STOP == 0) {
    if (C2_state == 0) {
      digital_outputs.set(STAGE_2_IO, HIGH);  // Turn On the LED of Stage 2

      C2_state = 1;
      Serial.println("Stage 2 Started");
      stage = 2;
      setStage(2);
      Serial.println("Stage 0 Status Send packet ");
      F1_stg_2_timmer = millis() - (N_st2.N_f1_st2_offtime * MINS);
    }

    // Turn ON F1 when time is over
    if (MTR_State == 0 && (millis() - F1_stg_2_timmer >= (N_st2.N_f1_st2_offtime * MINS))) {
      digital_outputs.set(FAN_IO, HIGH);  // Output of F1
      Serial.println("Stage 2 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;  // When M_F1 = 1 ==> ON

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stg2 F1 Start published ");
      F1_stg_2_timmer = millis();
    }

    // Turn OFF F1 when time is over
    if (MTR_State == 1 && (millis() - F1_stg_2_timmer >= (N_st2.N_f1_st2_ontime * MINS))) {
      digital_outputs.set(FAN_IO, LOW);
      Serial.println("Stage 2 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stg2 F1 stop published ");
      F1_stg_2_timmer = millis();
    }

    // Turn ON S1 when time is over
    if ((MTR_State == 1) && (S1_state == 0) && (millis() - S1_stg_2_timer >= (N_st2.N_s1_st2_offtime * MINS))) {
      digital_outputs.set(VALVE_IO, HIGH);  // Output of S1
      S1_state = 1;
      Serial.println("Stage 2 S1 ON");
      S1_data.M_S1 = 1;  // When M_S1 = 1 ==> ON

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg2 S1 start published");
      S1_stg_2_timer = millis();
    }

    // Turn OFF S1 when time is over
    if ((S1_state == 1 && (millis() - S1_stg_2_timer >= (N_st2.N_s1_st2_ontime * MINS))) || (MTR_State == 0)) {
      digital_outputs.set(VALVE_IO, LOW);  // Output of S1
      S1_state = 0;
      Serial.println("Stage 2 S1 OFF");
      S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg2 S1 stop published");

      S1_stg_2_timer = millis();
    }

    // Calculate the Setpoint every 3 seconds in Function of Ta with the formula : Setpoint = A*(B-Ta)
    if ((millis() - pid_computing_timer >= 3000)) {
      Setpoint = (-(N_SP.N_A * (temp_data.AvgTs_N)) + N_SP.N_B);  //use the average of the temperature over the x last minuites
      setpoint_data.PID_setpoint = float(Setpoint);

      mqtt.publishData(SETPOINT, setpoint_data.PID_setpoint);

      Serial.println("Setpoint published");
      pid_computing_timer = millis();
    }

    // Activate the PID when F1 ON
    if (MTR_State == 1 && (millis() - turn_on_pid_timer >= 3000)) {
      PIDinput = TA_F;
      coefOutput = (coefPID * Output) / 100;  // Transform the Output of the PID to the desired max value
      Serial.println(coefOutput);
      air_in_feed_PID.Compute();
      // analogWrite(A0_5, Output);
      analog_out.write(AIR_PIN, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      Serial.println("Converted_Output is " + String(Converted_Output));
      turn_on_pid_timer = millis();
    }

    // Put the PID at 0 when F1 OFF
    if (MTR_State == 0 && (millis() - turn_on_pid_timer >= 3000)) {
      //Setpoint = 0;
      PIDinput = 0;
      Output = 0;
      coefOutput = 0;
      // analogWrite(A0_5, Output);
      analog_out.write(AIR_PIN, Output);
      Converted_Output = ((Output - 0) / (255 - 0)) * (10000 - 0) + 0;
      Serial.println("Converted_Output is " + String(Converted_Output));
      turn_off_pid_timer = millis();
    }
  }

  //---- STAGE 3 ----////////////////////////////////////////////////////////////////////////////
  // Initialisation Stage3 (reset all the other stages to 0)
  if (TS_F >= N_tset.N_ts_set && TC_F >= N_tset.N_tc_set && Stage3_started == 0 && Stage2_started == 1) {
    START1 = START2 = Stage2_RTC_set = MTR_State = 0;

    // Turn All Output OFF
    analog_out.write(AIR_PIN, 0);
    // analogWrite(A0_5, 0);
    digital_outputs.set(STAGE_1_IO, LOW);
    digital_outputs.set(STAGE_2_IO, LOW);
    digital_outputs.set(STAGE_3_IO, LOW);
    digital_outputs.set(VALVE_IO, LOW);
    digital_outputs.set(FAN_IO, LOW);
    Output = 0;
    coefOutput = 0;

    F1_data.M_F1 = 2;  // When M_F1 = 2 ==> OFF

    mqtt.publishData(m_F1, F1_data.M_F1);
    Serial.println("stage 3 F1 init published ");

    S1_data.M_S1 = 2;  // When M_S1 = 2 ==> OFF

    mqtt.publishData(m_S1, S1_data.M_S1);
    Serial.println("stage 2 S1 init published");

    C2_state = S1_state = 0;  // Put the all the states to 0
    Serial.println("Stage 3 Initiated");
    Stage3_started = 1;
  }

  // Stage 3
  if (Stage3_started == 1 && Stage2_started == 1 && STOP == 0) {
    // State of Stage 3 turned to 1
    if (C3_state == 0) {
      digital_outputs.set(STAGE_3_IO, HIGH);  // Turn ON the LED of Stage 3

      C3_state = 1;
      Serial.println("Stage 3 Started");
      setStage(3);
      Serial.println("Stage 3 Status Send packet ");
      F1_stg_3_timer = millis() - (N_st3.N_f1_st3_offtime * MINS);
    }

    // Turn ON F1 when time is over
    if (MTR_State == 0 && (millis() - F1_stg_3_timer >= (N_st3.N_f1_st3_offtime * MINS))) {
      digital_outputs.set(FAN_IO, HIGH);
      // analog_out.write(AIR_PIN, duty_cycle);
      Serial.println("Stage 3 F1 On");
      MTR_State = 1;
      F1_data.M_F1 = 1;

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stage 3 F1 start published ");
      F1_stg_3_timer = millis();
    }

    // Turn OFF F1 when time is over
    if (MTR_State == 1 && (millis() - F1_stg_3_timer >= (N_st3.N_f1_st3_ontime * MINS))) {
      digital_outputs.set(FAN_IO, LOW);
      // analog_out.write(AIR_PIN, 0);
      Serial.println("Stage 3 F1 Off");
      MTR_State = 0;
      F1_data.M_F1 = 2;

      mqtt.publishData(m_F1, F1_data.M_F1);
      Serial.println("stage 3 F1 stop published ");
      F1_stg_3_timer = millis();
    }

    if (S1_state == 0 && (millis() - S1_stg_3_timer >= (N_st3.N_s1_st3_offtime * MINS))) {
      digital_outputs.set(VALVE_IO, HIGH);
      S1_state = 1;
      Serial.println("Stage 3 S1 ON");
      S1_data.M_S1 = 1;

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg3 S1 start published");
      S1_stg_3_timer = millis();
    }

    if (S1_state == 1 && (millis() - S1_stg_3_timer >= (N_st3.N_s1_st3_ontime * MINS))) {
      digital_outputs.set(VALVE_IO, LOW);
      S1_state = 0;
      Serial.println("Stage 3 S1 OFF with value of S1 ");
      S1_data.M_S1 = 2;

      mqtt.publishData(m_S1, S1_data.M_S1);
      Serial.println("stg3 S1 stop published");
      S1_stg_3_timer = millis();
    }
  }
}

//// fct Callback ==> RECEIVE MQTT MESSAGES ////////////////////////////////////////////////////////////////////
void callback(char *topic, byte *payload, unsigned int len) {
  Serial.println("Message arrived [" + String(topic) + "]");

  // Delayed start timing
  if (strcmp(topic, sub_hours) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_hour = responseToFloat(payload, len);
    Serial.println("Stage 2 Hours set to: " + String(Stage2_minute));
  }

  if (strcmp(topic, sub_minutes) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_minute = responseToFloat(payload, len);
    Serial.println("Stage 2 Minutes set to: " + String(Stage2_minute));
  }

  if (strcmp(topic, sub_day) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_day = responseToFloat(payload, len);
    Serial.println("Stage 2 Day set to: " + String(Stage2_day));
  }

  if (strcmp(topic, sub_month) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Stage2_month = responseToFloat(payload, len);
    Serial.println("Stage 2 Month set to: " + String(N_rtc.N_month));
  }

  //F1 stg1 on/off time
  if (strcmp(topic, sub_f1_st1_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st1.N_f1_st1_ontime = responseToFloat(payload, len);
    Serial.println("F1 Stage 1 on time set to: " + String(N_st1.N_f1_st1_ontime) + " MINS");
  }

  if (strcmp(topic, sub_f1_st1_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st1.N_f1_st1_offtime = responseToFloat(payload, len);
    Serial.println("F1 Stage 1 off time set to: " + String(N_st1.N_f1_st1_offtime) + " MINS");
  }

  // F1 and S1 STAGE 2 on/off time
  if (strcmp(topic, sub_f1_st2_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_f1_st2_ontime = responseToFloat(payload, len);
    Serial.println("F1 Stage 2 on time set to: " + String(N_st2.N_f1_st2_ontime) + " MINS");
  }

  if (strcmp(topic, sub_f1_st2_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_f1_st2_offtime = responseToFloat(payload, len);
    Serial.println("F1 Stage 2 off time set to: " + String(N_st2.N_f1_st2_offtime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st2_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_s1_st2_ontime = responseToFloat(payload, len);
    Serial.println("S1 Stage 2 on time set to: " + String(N_st2.N_s1_st2_ontime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st2_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st2.N_s1_st2_offtime = responseToFloat(payload, len);
    Serial.println("S1 Stage 2 off time set to: " + String(N_st2.N_s1_st2_offtime) + " MINS");
  }

  // F1 and S1 STAGE 3 on/off time
  if (strcmp(topic, sub_f1_st3_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_f1_st3_ontime = responseToFloat(payload, len);
    Serial.println("F1 Stage 3 on time set to: " + String(N_st3.N_f1_st3_ontime) + " MINS");
  }

  if (strcmp(topic, sub_f1_st3_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_f1_st3_offtime = responseToFloat(payload, len);
    Serial.println("F1 Stage 3 off time set to: " + String(N_st3.N_f1_st3_offtime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st3_ontime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_s1_st3_ontime = responseToFloat(payload, len);
    Serial.println("S1 Stage 3 on time set to: " + String(N_st3.N_s1_st3_ontime) + " MINS");
  }

  if (strcmp(topic, sub_s1_st3_offtime) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_st3.N_s1_st3_offtime = responseToFloat(payload, len);
    Serial.println("S1 Stage 3 off time set to: " + String(N_st3.N_s1_st3_offtime) + " MINS");
  }

  // Sub A and Sub B value update
  if (strcmp(topic, sub_A) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_SP.N_A = responseToFloat(payload, len);
    Serial.println("A set to: " + String(N_SP.N_A));
    R_A = 1;
  }

  if (strcmp(topic, sub_B) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_SP.N_B = responseToFloat(payload, len);
    Serial.println("B set to: " + String(N_SP.N_B));
    R_B = 1;
  }

  // PID update
  if (strcmp(topic, sub_P) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Kp = responseToFloat(payload, len);
    Serial.println("P set to: " + String(Kp));
    R_P = 1;
  }

  if (strcmp(topic, sub_I) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Ki = responseToFloat(payload, len);
    Serial.println("I set to: " + String(Ki));
    R_I = 1;
  }

  if (strcmp(topic, sub_D) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    Kd = responseToFloat(payload, len);
    Serial.println("D set to: " + String(Kd));
    R_D = 1;
  }

  if (R_P == 1 && R_I == 1 && R_D == 1 && START1 == 0 && START2 == 0 && STOP == 0) {
    air_in_feed_PID.SetTunings(Kp, Ki, Kd);
    Serial.println("New PID parameter updated");
    R_P = R_I = R_D = 0;
  }

  if (strcmp(topic, sub_coefPID) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    coefPID = responseToInt(payload, len);
    Serial.print("coef PID : " + String(coefPID));
  }

  // Target temperature Ts & Tc update
  if (strcmp(topic, sub_ts_set) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_tset.N_ts_set = responseToFloat(payload, len);
    Serial.println("Ts Condition set to: " + String(N_tset.N_ts_set));
  }

  if (strcmp(topic, sub_tc_set) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_tset.N_tc_set = responseToFloat(payload, len);
    // Tc_cond = N_tset->N_tc_set;
    Serial.println("Tc Condition set to: " + String(N_tset.N_tc_set));
  }

  // START
  if (strcmp(topic, sub_start) == 0 && START1 == 0 && START2 == 0 && STOP == 0) {
    N_start = responseToInt(payload, len);
    Serial.println("START BUTTON PRESSED ON NODE RED" + String(N_start));
  }

  // D_START
  if ((strcmp(topic, sub_d_start) == 0) && START2 == 0 && STOP == 0) {
    N_d_start = responseToInt(payload, len);
    Serial.println("d_start BUTTON PRESSED ON NODE RED" + String(N_d_start));
  }

  // STOP
  if (strcmp(topic, sub_stop) == 0) {
    N_stop = responseToInt(payload, len);
    Serial.println("stop BUTTON PRESSED ON NODE RED" + String(N_stop));
  }

  // Choose TS
  if (strcmp(topic, sub_chooseTs) == 0) {
    N_chooseTs = responseToInt(payload, len);
    Serial.println("Ts is now IR" + String(N_chooseTs));
  }
}

//// Stop button pressed ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void stopRoutine() {
  if (stop_temp1 == 0) {
    Serial.println("PROCESS STOP INITIATED");
    digital_outputs.set(STAGE_1_IO, LOW);
    digital_outputs.set(STAGE_2_IO, LOW);
    digital_outputs.set(STAGE_3_IO, LOW);
    digital_outputs.set(VALVE_IO, LOW);
    digital_outputs.set(FAN_IO, LOW);
    analog_out.write(AIR_PIN, 0);
    // analogWrite(A0_5, 0);

    stage = 0;
    Output = 0;
    coefOutput = 0;
    stop_temp1 = 1;

    F1_data.M_F1 = S1_data.M_S1 = 2;

    mqtt.publishData(m_F1, F1_data.M_F1);
    mqtt.publishData(m_S1, S1_data.M_S1);
    setStage(0);
    Serial.println("Stage 0 Status Send packet ");
  }

  if (stop_temp2 == 0) {
    MTR_State = C1_state = C2_state = C3_state = S1_state = START1 = START2 = Stage2_started = Stage3_started = Stage2_RTC_set = 0;
    stop_temp2 = 1;
  }

  if (stop_temp2 == 1) {
    Serial.println("PROCESS STOPPED");
    stop_temp1 = stop_temp2 = STOP = 0;
  }
}

void setUpRTC() {
  Wire1.begin();  // join i2c bus
  Wire1.beginTransmission(0x51);

  if (!rtc.begin(&Wire1)) {
    Serial.println("Couldn't find RTC");
    // while (1);
  }
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
}

void updateTemperature() {
  DateTime now = rtc.now();

  TA = getProbeTemp(TA_AI);
  TS = getProbeTemp(TS_AI);
  TC = getProbeTemp(TC_AI);

  // TA = readTempFrom(TA_AI);
  // TS = readTempFrom(TS_AI);
  // TC = readTempFrom(TC_AI);
}

String addressToString(uint8_t *address) {
  String formated_address;
  for (int i = 0; i < 8; i++) {
    formated_address += address[i];
    if (i < 7) formated_address += ",";
  }
  return formated_address;
}

void setStage(int Stage) {
  // if (stage_data.stage == Stage) return;
  stage_data.stage = Stage;
  mqtt.publishData(STAGE, Stage);
}

float responseToFloat(byte *value, size_t len) {
  String puta_mierda_mal_parida;
  for (int i = 0; i < len; i++) puta_mierda_mal_parida += (char)value[i];
  return puta_mierda_mal_parida.toFloat();
}

int responseToInt(byte *value, size_t len) {
  String puta_mierda_mal_parida;
  for (int i = 0; i < len; i++) puta_mierda_mal_parida += (char)value[i];
  return puta_mierda_mal_parida.toInt();
}

float getProbeTemp(uint8_t channel) {
  temp_probes.selectChannel(channel);
  float temperature = temp_probes.rtd.readTemperature(RNOMINAL, RREF);

  return temperature;
}

float getAnalogTemp(uint8_t channel) {
  uint16_t analog_value = analog_inputs.readADC_SingleEnded(TA_AI);  // Ta
  const float temperature = (analog_value - 0) * (50 + 50) / (1024 - 0) - 50;

  return temperature;
}

float getIRTemp() {
  uint16_t result;
  float temperature;
  Wire.beginTransmission(IR_SENSOR_ADDRESS);
  Wire.write(READ_TEMPERATURE);
  Wire.endTransmission(false);
  Wire.requestFrom(IR_SENSOR_ADDRESS, 2);
  result = Wire.read();
  result |= Wire.read() << 8;

  temperature = result * 0.02 - 273.15;
  return temperature;
}

float readTempFrom(uint8_t channel) {
  const uint16_t raw_voltage_ch = analog_in.read(channel);
  const float voltage_ch = (raw_voltage_ch * voltage_per_step);
  // Serial.println(voltage_ch);
  const float temp = (voltage_ch * temperature_per_step) + TEMPERATURE_MIN;
  
  return temp;
}

void setUpDefaulParameters(){
  // Default parameters
  N_st1.N_f1_st1_ontime = 1;
  N_st1.N_f1_st1_offtime = 40;

  N_st2.N_f1_st2_ontime = 30;
  N_st2.N_f1_st2_offtime = 10;
  N_st2.N_s1_st2_ontime = 1;
  N_st2.N_s1_st2_offtime = 5;

  N_st3.N_f1_st3_ontime = 10;
  N_st3.N_f1_st3_offtime = 30;
  N_st3.N_s1_st3_ontime = 1;
  N_st3.N_s1_st3_offtime = 15;

  N_tset.N_ts_set = 40;
  N_tset.N_tc_set = 40;

  N_SP.N_A = 0.5;
  N_SP.N_B = 20; 
}
