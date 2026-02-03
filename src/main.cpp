#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>

#define d1 5
#define d2 6
#define d3 7
#define d4 10

const int array[5] = {d1, d2, d3, d4, d1};
bool ota_update = false;
int delay_time = 4; // 3,5 - 4,0 ms is fine 
bool up = false;
long long int last_time = 0;

void step(bool drection=true);


void setup() {
  Serial.begin(115200);

  WiFi.begin("******", "*********");

  while (WiFi.isConnected() != WL_CONNECTED) {
    Serial.print(".");
    vTaskDelay(pdMS_TO_TICKS(200));
  }

  pinMode(d1, OUTPUT);
  pinMode(d2, OUTPUT);
  pinMode(d3, OUTPUT);
  pinMode(d4, OUTPUT);

  ArduinoOTA.onStart([]() {
    ota_update = true;
  });

  ArduinoOTA.onEnd([]() {
    ota_update = false;
  });

}

void loop() {
  ArduinoOTA.handle();

  if (millis() - last_time > 5000) {
    up = !up;
    last_time = millis();
  }

  step(up);

  
}


void step(bool direction) {
  if (direction) {

    for (int i=1; i<5; i++) {
      digitalWrite(array[i-1], LOW);
      digitalWrite(array[i], HIGH);

      vTaskDelay(pdMS_TO_TICKS(delay_time));

    }

  } else {

    for (int i=3; i>=0; i--) {
      digitalWrite(array[i+1], LOW);
      digitalWrite(array[i], HIGH);

      vTaskDelay(pdMS_TO_TICKS(delay_time));

    }

  }
}
