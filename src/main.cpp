#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPTelnet.h>
#include <esp_system.h>

#define d1 10    // TODO sprawdzic piny
#define d2 5
#define d3 6
#define d4 7
#define led_builtin 8

#define MOTOR_UP 1
#define MOTOR_DOWN -1
#define MOTOR_STOP 0

#define FULLY_CLOSED 2
#define UNKNOWN_POS 3
#define FULLY_EXPOSED 4

#define FULLY_UP 0
#define HALF_OPENED 40000
#define FULLY_DOWN 86000   // połowa okna to 40 000


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

// Font colors
const char* CLR_RST = "\033[0m";
const char* CLR_RED = "\033[31m";
const char* CLR_GRN = "\033[32m";
const char* CLR_YLW = "\033[33m";

const int telnet_port = 23;

bool ota_update = false;
bool control_steps = false;
bool confirm_position = false;

int blid_position = FULLY_CLOSED;
const int delay_time = 2;                                  // 3-4 ms is fine on full_steps
int motor_state = MOTOR_STOP;                        // or 2 ms on half_steps
int steps_total = FULLY_DOWN;
int goal = FULLY_UP;

int64_t last_recon_time = 0;

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
void step(bool drection=true);
void reconnect();
const char* getMQTTState(int state);
void stop_motor();
const char* getResetReason();


void setup() {

  Serial.begin(115200);
  pinMode(led_builtin, OUTPUT);

  WiFi.begin(ssid, passwd);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    
    digitalWrite(led_builtin, !digitalRead(led_builtin));
    vTaskDelay(pdMS_TO_TICKS(200));

  }

  Serial.print("\nConnected with WiFi! IP: ");
  Serial.println(WiFi.localIP()); 

  digitalWrite(led_builtin, HIGH);                    // reverse logic
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

void loop() {                                   
  ArduinoOTA.handle();
  
  if (!ota_update) {

    int64_t now = esp_timer_get_time();

    if (!mqttClient.connected() && now - last_recon_time > 5000000) { //   5sec
      reconnect(); 
      last_recon_time = now;
    }

    mqttClient.loop();

    if (!telnet || !telnet.connected()) {
      
      telnet = telnetServer.accept();      
      if (telnet) {
      
        telnet.printf("INFO: \tTelnet client have just %sconnected!%s\n\r", CLR_GRN, CLR_RST);
        Serial.print("INFO: \tTelnet client have just connected!\n");

        telnet.printf("%sWARNING%s: \tLast reset reason: %s%s%s\n\r",
          CLR_YLW, CLR_RST, CLR_YLW, getResetReason(), CLR_RST);
      
      }
    }

    while (telnet && telnet.available()) telnet.read();

    if (motor_state == MOTOR_UP) {

      step(false);
      
      if (control_steps && steps_total <= goal) {

        mqttClient.publish(mqtt_topic_pub, "FULLY_EXPOSED");

        telnet.printf("INFO: \tWindow blinder: %sFULLY_EXPOSED%s\n\r", CLR_GRN, CLR_RST);
        
        blid_position = FULLY_EXPOSED;
        motor_state = MOTOR_STOP;
        stop_motor();

      }
    
    } else if (motor_state == MOTOR_DOWN) {
        
      step(true);

      if (control_steps && steps_total >= goal) {
          
        mqttClient.publish(mqtt_topic_pub, "FULLY_CLOSED");

        telnet.printf("INFO: \tWindow blinder: %sFULLY_CLOSED%s\n\r", CLR_GRN, CLR_RST);
        
        blid_position = FULLY_CLOSED;
        motor_state = MOTOR_STOP;
        stop_motor();

      }
    
    } else if (motor_state == MOTOR_STOP) {
      stop_motor();
    }

    if (steps_total % 1000 < 20 && confirm_position) {
      
      mqttClient.publish(mqtt_topic_steps, std::to_string(steps_total).c_str(), true);
      
      confirm_position = false;
    }

  }

}


void step(bool direction) {
  confirm_position = true;

  int length = half_steps.size();

  if (direction) {

    for (int i=1; i<length; i++) {
 
      digitalWrite(half_steps[i-1][0], LOW);
      if (half_steps[i-1][1] != -1) digitalWrite(half_steps[i-1][1], LOW);

      digitalWrite(half_steps[i][0], HIGH);
      if (half_steps[i][1] != -1) digitalWrite(half_steps[i][1], HIGH);
     
      steps_total ++;
      vTaskDelay(pdMS_TO_TICKS(delay_time));

    }

  } else {

    for (int i = (length - 2); i >= 0; i--) {

      digitalWrite(half_steps[i+1][0], LOW);
      if (half_steps[i+1][1] != -1) digitalWrite(half_steps[i+1][1], LOW);

      digitalWrite(half_steps[i][0], HIGH);
      if (half_steps[i][1] != -1) digitalWrite(half_steps[i][1], HIGH);

      steps_total --;
      vTaskDelay(pdMS_TO_TICKS(delay_time));

    }

  }
}

void stop_motor() {
  digitalWrite(d1, LOW);
  digitalWrite(d2, LOW);
  digitalWrite(d3, LOW);
  digitalWrite(d4, LOW);
}

void callback(char *topic, byte *payload, unsigned int length) {

  String msg;

  for (uint i=0; i<length; i++) {
    msg += (char)payload[i];
  }

  if (String(topic) == mqtt_topic_sub) {

    //                    GOING UP (manually)
    if (msg == "up") {

      motor_state = MOTOR_UP; 
      control_steps = false;
      blid_position = UNKNOWN_POS;

      telnet.printf("INFO: \tWindow blinder: %sUP%s\n\r", CLR_GRN, CLR_RST);

      telnet.printf("%sWARNING%s: \tYou migt recalibrate motor. When window is fully closed or \
        fully opened, use %s\"SET_CLOSED\"%s or %s\"SET_OPENED\"%s.\n\r", 
        CLR_YLW, CLR_RST, CLR_YLW, CLR_RST, CLR_YLW, CLR_RST);


    //                    GOING DOWN (manually)
    } else if (msg == "down") {

      motor_state = MOTOR_DOWN;
      control_steps = false;
      blid_position = UNKNOWN_POS;

      telnet.printf("INFO: \tWindow blinder: %sDOWN%s\n\r", CLR_GRN, CLR_RST);

      telnet.printf("%sWARNING%s: \tYou migt recalibrate motor. When window is fully closed or \
        fully opened, use %s\"SET_CLOSED\"%s or %s\"SET_OPENED\"%s.\n\r", 
        CLR_YLW, CLR_RST, CLR_YLW, CLR_RST, CLR_YLW, CLR_RST);

    //                    STOP (manually)
    } else if (msg == "stop") {

      motor_state = MOTOR_STOP;                  
      control_steps = false;
      blid_position = UNKNOWN_POS;

      telnet.printf("INFO: \tWindow blinder: %sSTOP%s\n\r", CLR_RED, CLR_RST);

      telnet.printf("%sWARNING%s: \tYou migt recalibrate motor. When window is fully closed or \
        fully opened, use %s\"SET_CLOSED\"%s or %s\"SET_OPENED\"%s.\n\r", 
        CLR_YLW, CLR_RST, CLR_YLW, CLR_RST, CLR_YLW, CLR_RST);

    } else if (msg == "full_close") {

      motor_state = MOTOR_DOWN;
      control_steps = true;
      goal = FULLY_DOWN;

      telnet.print("INFO: \tWindow blinder: GOING FULLY DOWN\n\r");

    } else if (msg == "full_expose") {

      motor_state = MOTOR_UP;
      control_steps = true;
      goal = FULLY_UP;

      telnet.print("INFO: \tWindow blinder: GOING FULLY UP\n\r");

    } else if (msg == "help") {

      telnet.printf("Commands:\n\r");

      telnet.printf("\t%sup%s: \twindow roller will go up until you send %sstop%s command.\n\r", 
        CLR_GRN, CLR_RST, CLR_RED, CLR_RST);

      telnet.printf("\t%sdown%s: \twindow roller will go down until you send %sstop%s command.\n\r", 
        CLR_GRN, CLR_RST, CLR_RED, CLR_RST);

      telnet.printf("\t%sstop%s: \twindow roller should %sstop%s no matter what.\n\r", 
        CLR_RED, CLR_RST, CLR_RED, CLR_RST);

      telnet.printf("\t%sfull_close%s: \twindow roller will go down until it cover entire window.\n\r", 
        CLR_GRN, CLR_RST, CLR_RED, CLR_RST);

      telnet.printf("\t%sfull_expose%s: \twindow roller will go up until it expose entire window.\n\r", 
        CLR_GRN, CLR_RST, CLR_RED, CLR_RST);

      telnet.print("\n\n\r");
    } else {

      telnet.printf("%sWARNING%s: \tUnexpected msg received: %s \n\r", CLR_YLW, CLR_RST, msg.c_str());
    }
  }

}

void reconnect() {

  bool connected = false;

  Serial.print("WARNING: \tReconnecting with MQTT broker...\n");
  telnet.printf("%sWARNING%s: \tReconnecting with MQTT broker...\n\r", CLR_YLW, CLR_RST);


  if (mqttClient.connect("window_blinder", mqtt_user, mqtt_passwd)) {
    Serial.print("Connected!\n");
    telnet.printf("%sConnected!%s\n\r", CLR_GRN, CLR_RST);
    
    connected = true;

    mqttClient.subscribe(mqtt_topic_sub);
    
  } else {
    
    Serial.printf("Error: \t%s\n", getMQTTState(mqttClient.state()) );
    telnet.printf("%sError%s: \t%s%s%s\n\r", 
      CLR_RED, CLR_RST, CLR_RED, getMQTTState(mqttClient.state()), CLR_RST);

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

const char* getResetReason() {
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON: return "Power on reset";
        case ESP_RST_SW:      return "Software reset";
        case ESP_RST_PANIC:   return "Exception / Panic reset";
        case ESP_RST_INT_WDT: return "Interrupt Watchdog";
        case ESP_RST_TASK_WDT:return "Task Watchdog";
        case ESP_RST_BROWNOUT:return "Brownout (voltage dip)";
        default:              return "Other";
    }
}
