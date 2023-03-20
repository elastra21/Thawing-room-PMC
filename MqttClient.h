#ifndef MY_MQTT_H
#define MY_MQTT_H
#include <PubSubClient.h>

//             subscribe topics    -------------------------------------------------------------------->
#define sub_hours           "mduino/hours"
#define sub_minutes         "mduino/minutes"
#define sub_day             "mduino/day"
#define sub_month           "mduino/month"
#define sub_f1_st1_ontime   "mduino/f1_st1_ontime"
#define sub_f1_st1_offtime  "mduino/f1_st1_offtime"
#define sub_f1_st2_ontime   "mduino/f1_st2_ontime"
#define sub_f1_st2_offtime  "mduino/f1_st2_offtime"
#define sub_s1_st2_ontime   "mduino/s1_st2_ontime"
#define sub_s1_st2_offtime  "mduino/s1_st2_offtime"
#define sub_f1_st3_ontime   "mduino/f1_st3_ontime"
#define sub_f1_st3_offtime  "mduino/f1_st3_offtime"
#define sub_s1_st3_ontime   "mduino/s1_st3_ontime"
#define sub_s1_st3_offtime  "mduino/s1_st3_offtime"
#define sub_A               "mduino/A"
#define sub_B               "mduino/B"
#define sub_ts_set          "mduino/ts_set"
#define sub_tc_set          "mduino/tc_set"
#define sub_start           "mduino/start"
#define sub_d_start         "mduino/d_start"
#define sub_stop            "mduino/stop"
#define sub_TempAcqDelay    "mduino/TempAcqDelay"
#define sub_P               "mduino/P"
#define sub_I               "mduino/I"
#define sub_D               "mduino/D"
#define sub_avgTiming       "mduino/TsAvgTime"      // in ms the sampling rate for Ts calculation
#define sub_tsAvgSpan       "mduino/TsAvgFifoSpan"  // in minutes the span of the fifo for Ts calculation
#define sub_chooseTs        "mduino/chooseTs"
#define sub_coefPID         "mduino/coefPID"

//------------ publish index    -------------------------------------------------------------------->
#define m_F1                "mduino/M_F1"
#define m_S1                "mduino/M_S1"
#define STAGE               "mduino/stage"
#define AVG_TS_TOPIC        "mduino/AvgTs"
#define TA_TOPIC            "mduino/Ta"
#define TS_TOPIC            "mduino/Ts"
#define TC_TOPIC            "mduino/Tc"
#define TI_TOPIC            "mduino/Ti"
#define PID_OUTPUT          "mduino/PID_output"
#define SETPOINT            "mduino/setpoint"
#define ACK_F1_ST1_ONTIME   "mduino/ack_f1_st1_ontime"
#define ACK_F1_ST1_OFFTIME  "mduino/ack_f1_st1_offtime"
#define ACK_F1_ST2_ONTIME   "mduino/ack_f1_st2_ontime"
#define ACK_F1_ST2_OFFTIME  "mduino/ack_f1_st2_offtime"
#define ACK_S1_ST2_ONTIME   "mduino/ack_s1_st2_ontime"
#define ACK_S1_ST2_OFFTIME  "mduino/ack_s1_st2_offtime"
#define ACK_F1_ST3_ONTIME   "mduino/ack_f1_st3_ontime"
#define ACK_F1_ST3_OFFTIME  "mduino/ack_f1_st3_offtime"
#define ACK_S1_ST3_ONTIME   "mduino/ack_s1_st3_ontime"
#define ACK_S1_ST3_OFFTIME  "mduino/ack_s1_st3_offtime"
#define ACK_A               "mduino/ack_A"
#define ACK_B               "mduino/ack_B"



class MqttClient {
  public:
    void loop();
    void connect();
    void reconnect();
    bool isConnected();
    void subscribeRoutine();
    bool refreshMQTTStatus();
    bool isServiceAvailable();
    bool getConnectionStatus();
    void publishData(String topic, double value);
    void publishData(String topic, String value);
    void setCallback(void (*callback)(char*, uint8_t*, unsigned int));
  private:
    bool no_service_available = true;
    bool last_connection_state = false;

};
#endif
