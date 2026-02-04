#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPTelnet.h>

#define d1 5
#define d2 6
#define d3 7
#define d4 10
#define led_builtin 8

#define MOTOR_UP 1
#define MOTOR_DOWN -1
#define MOTOR_STOP 0

#define FULLY_CLOSED 2
#define UNKNOWN 3
#define FULLY_EXPOSED 4

#define FULLY_UP 0
#define FULLY_DOWN 92700

const char *ssid = "TP-Link_5235";
const char *passwd = "MaDaPi16";
const char *mqtt_server = "192.168.0.100";  // don't forget, that (in my case) it's running on MQTT
                                            // v. 3.1
const int mqtt_port = 1883;
const char *mqtt_user = "xiao0";
const char *mqtt_passwd = "broker#1234";
const char *mqtt_topic_pub = "window_roller/state";
const char *mqtt_topic_steps = "window_roller/step";
const char *mqtt_topic_sub = "window_roller/motor";

const int telnet_port = 23;

bool ota_update = false;
bool telnet_connected = false;
bool force_refresh = false;
bool do_half_step = true;
bool control_steps = false;
bool confirm_position = false;

int blid_position = FULLY_CLOSED;
int delay_time = 2; // 3-4 ms is fine (on full_steps)
int motor_state = MOTOR_STOP;
int steps_total = FULLY_DOWN;
int goal = FULLY_UP;

long long int last_time = 0;
long long int last_recon_time = 0;

std::array <int, 5> full_steps = {{d1, d2, d3, d4, d1}};
std::array <std::array<int, 2>, 9> half_steps = {{
                              {d1, -1},
                              {d1, d2},
                              {d2, -1},
                              {d2, d3},
                              {d3, -1},
                              {d3, d4},
                              {d4, -1},
                              {d1, d4},
                              {d1, -1}
                            }};
                            

WiFiServer telnetServer(telnet_port);
WiFiClient telnet;

WiFiClient client;
PubSubClient mqttClient(client);

void callback(char *topic, byte *payload, unsigned int length);
void step(bool drection=true, bool half_step=false);
void reconnect();
const char* getMQTTState(int state);
void reset_motor();

void setup() {
  Serial.begin(115200);
  pinMode(led_builtin, OUTPUT);

  WiFi.begin(ssid, passwd);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    
    digitalWrite(led_builtin, !digitalRead(led_builtin));
    vTaskDelay(pdMS_TO_TICKS(200));

  }

  Serial.printf("\nConnected with WiFi! IP: %s\n", WiFi.localIP().toString().c_str());

  digitalWrite(led_builtin, HIGH); // reverse logic
  telnetServer.begin();

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);


  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(d3, OUTPUT);
  pinMode(d4, OUTPUT);

  ArduinoOTA.begin();

  ArduinoOTA.onStart([]() {
    ota_update = true;
  });

  ArduinoOTA.onEnd([]() {
    ota_update = false;
  });

}

void loop() {                                     // TODO 'up' & 'down' works just fine
  ArduinoOTA.handle();                            // but 'full_close' & 'full_expose' not working

  if (!ota_update) {


    if (!mqttClient.connected() && millis() - last_recon_time > 5000) {
      reconnect();
      last_recon_time = millis();
    }

    mqttClient.loop();

    if ((!telnet || !telnet.connected()) && !telnet_connected) {
      
      telnet = telnetServer.accept();      
      if (telnet) {
      
        telnet.print("Telnet client have just connected!\n\r");
        Serial.print("Telnet client have just connected!\n");
        telnet_connected = true;
      
      }
    }

    if (motor_state == MOTOR_UP) {

      if (control_steps) {
        if (steps_total > goal) {

          step(true, do_half_step);

        } else { 

          blid_position = FULLY_EXPOSED;

          mqttClient.publish(mqtt_topic_pub, "FULLY_EXPOSED");
          Serial.print("INFO: \tWindow blinder: FULLY_EXPOSED\n");
          telnet.print("INFO: \tWindow blinder: FULLY_EXPOSED\n\r");
    
          motor_state = MOTOR_STOP;

          
        }
      } else {
          step(true, do_half_step);
      }
    
    } else if (motor_state == MOTOR_DOWN) {
        
      if (control_steps) {
        if (steps_total < goal) {

          step(false, do_half_step);

        } else { 

          blid_position = FULLY_CLOSED;

          mqttClient.publish(mqtt_topic_pub, "FULLY_CLOSED");
          Serial.print("INFO: \tWindow blinder: FULLY_CLOSED\n");
          telnet.print("INFO: \tWindow blinder: FULLY_CLOSED\n\r");
          motor_state = MOTOR_STOP;
          
        }
      } else {
          step(false, do_half_step);
      }
    
    } else if (motor_state == MOTOR_STOP) {
      reset_motor();
    }

    if (steps_total % 1000 < 20 && confirm_position) {
      mqttClient.publish(mqtt_topic_steps, std::to_string(steps_total).c_str());
      confirm_position = false;
    }

  }

}


void step(bool direction, bool go_half_step) {   // TODO: maybe split it to 2 functions
  confirm_position = true;

  int length = full_steps.size();
  if (go_half_step) length = half_steps.size();

  if (direction) {

    for (int i=1; i<length; i++) {
      if (go_half_step) {

        digitalWrite(half_steps[i-1][0], LOW);
        if (half_steps[i-1][1] != -1) digitalWrite(half_steps[i-1][1], LOW);

        digitalWrite(half_steps[i][0], HIGH);
        if (half_steps[i][1] != -1) digitalWrite(half_steps[i][1], HIGH);


      } else {

        digitalWrite(full_steps[i-1], LOW);
        digitalWrite(full_steps[i], HIGH);
      
      }
      
      steps_total ++;
      vTaskDelay(pdMS_TO_TICKS(delay_time));

    }

  } else {

    for (int i = (length - 2); i >= 0; i--) {

      if (go_half_step) {

        digitalWrite(half_steps[i+1][0], LOW);
        if (half_steps[i+1][1] != -1) digitalWrite(half_steps[i+1][1], LOW);

        digitalWrite(half_steps[i][0], HIGH);
        if (half_steps[i][1] != -1) digitalWrite(half_steps[i][1], HIGH);

      } else {

        digitalWrite(full_steps[i+1], LOW);
        digitalWrite(full_steps[i], HIGH);

      }

      steps_total --;
      vTaskDelay(pdMS_TO_TICKS(delay_time));

    }

  }
}

void reset_motor() {
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d4, LOW);
}

void callback(char *topic, byte *payload, unsigned int length) {

  String msg;

  for (int i=0; i<length; i++) {
    msg += (char)payload[i];
  }

  if (String(topic) == mqtt_topic_sub) {

    if (msg == "up") {

      motor_state = -1;
      control_steps = false;
      blid_position = UNKNOWN;

      Serial.print("INFO: \tWindow blinder: UP\n");
      telnet.print("INFO: \tWindow blinder: UP\n\r");

    } else if (msg == "down") {

      motor_state = 1;
      control_steps = false;
      blid_position = UNKNOWN;

      Serial.print("INFO: \tWindow blinder: DOWN\n");
      telnet.print("INFO: \tWindow blinder: DOWN\n\r");

    } else if (msg == "stop") {         // TODO if u manually controll stepper motor, 
                                        // position would be unprecise and full_close / full_expose
                                        // might not work properly
      motor_state = 0;                  
      control_steps = false;
      blid_position = UNKNOWN;

      Serial.print("INFO: \tWindow blinder: STOP\t\t");
      telnet.print("INFO: \tWindow blinder: STOP\t\t");

      Serial.printf("Steps made: %i\n", steps_total);
      telnet.printf("Steps made: %i\n\r", steps_total);


    } else if (msg == "full_step") {

      do_half_step = false;
      delay_time = 4;
      control_steps = false;
      
      Serial.print("INFO: \tWindow blinder: FULL_STEP\n");
      telnet.print("INFO: \tWindow blinder: FULL_STEP\n\r");

    } else if (msg == "half_step") {

      do_half_step = true;
      delay_time = 2;
      control_steps = false;

      Serial.print("INFO: \tWindow blinder: HALF_STEP\n");
      telnet.print("INFO: \tWindow blinder: HALF_STEP\n\r");
    } else if (msg == "full_close") {

      motor_state = 1;
      control_steps = true;
      goal = FULLY_DOWN;

      Serial.print("INFO: \tWindow blinder: GOING FULLY DOWN\n");
      telnet.print("INFO: \tWindow blinder: GOING FULLY DOWN\n\r");

    } else if (msg == "full_expose") {

      motor_state = -1;
      control_steps = true;
      goal = FULLY_UP;

      Serial.print("INFO: \tWindow blinder: GOING FULLY UP\n");
      telnet.print("INFO: \tWindow blinder: GOING FULLY UP\n\r");

    } else {

      telnet.printf("WARNING: \tUnexpected msg received: %s \n\r", msg.c_str());
      Serial.printf("WARNING: \tUnexpected msg received: %s \n", msg.c_str());
    }
  }

}

void reconnect() {

  bool connected = false;

  Serial.print("INFO: \tReconnecting with MQTT broker... ");
  telnet.print("INFO: \tReconnecting with MQTT broker... ");


  if (mqttClient.connect("window_roller", mqtt_user, mqtt_passwd)) {
    Serial.print("Connected!\n");
    telnet.print("Connected!\n\r");
    connected = true;

    mqttClient.subscribe(mqtt_topic_sub);
    
  } else {
    
    Serial.printf("Error: \t%s\n", getMQTTState(mqttClient.state()) );
    telnet.printf("Error: \t%s\n\r", getMQTTState(mqttClient.state()) );

  }
  
  if (!connected) {
    Serial.print("Trying again later...\n");
    telnet.print("Trying again later..\n\r");
  }  
  
  Serial.print("\n\n");
  telnet.print("\n\n\r");

}

const char* getMQTTState(int state) {
    switch (state) {
        case -4: return "MQTT_CONNECTION_TIMEOUT";
        case -3: return "MQTT_CONNECTION_LOST";
        case -2: return "MQTT_CONNECT_FAILED";
        case -1: return "MQTT_DISCONNECTED";
        case  0: return "MQTT_CONNECTED";
        case  1: return "MQTT_CONNECT_BAD_PROTOCOL";
        case  2: return "MQTT_CONNECT_BAD_CLIENT_ID";
        case  3: return "MQTT_CONNECT_UNAVAILABLE";
        case  4: return "MQTT_CONNECT_BAD_CREDENTIALS";
        case  5: return "MQTT_CONNECT_UNAUTHORIZED";
        default: return "Unknown type";
    }
}