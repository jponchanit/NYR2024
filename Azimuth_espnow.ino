#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

#define dirPin 40
#define stepPin 41
#define STEPS_PER_DEGREE 4.12045
QMC5883LCompass compass;
//Structure example to receive data
//Must match the sender structure
struct Vector2 {
    float x;
    float y;
}; struct Vector2 Vector;

// #define stepsPerRevolution  StepsPerRev //23hs2425 1600/360
float stepsPerRevolution(int compass_value, Vector2 Vector) {
  int Vector_x = (int)Vector.x;
  int CCW = (Vector_x - compass_value + 360) % 360;
  int CW = (compass_value - Vector_x + 360)%360;

    int rotation = (CCW <= CW) ? CW : -CCW;
    // Ensure the rotation is within the range of -180 to 180
    if (rotation > 180)
        rotation -= 360;
    else if (rotation <= -180)
        rotation += 360;

    return rotation;
}

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Vector, incomingData, sizeof(struct Vector2));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("azimuth: ");
  Serial.println(Vector.x);
  Serial.println();
}

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB toget recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  int compass_value;
      // Read compass values
      compass.read();
      compass_value = 360 - map(compass.getAzimuth(), -180, 180, 0, 360);
 int dir = stepsPerRevolution(compass_value, Vector);
  if (dir > 0) {
    digitalWrite(dirPin, LOW); // Clockwise
  } else {
    digitalWrite(dirPin, HIGH); // Counter-clockwise
    dir = -dir; // Take absolute value of steps
  }
}

void loop() {
  int compass_value;
      // Read compass values
      compass.read();
      compass_value = 360 - map(compass.getAzimuth(), -180, 180, 0, 360);
      Serial.print("Compass value : ");
      Serial.println(compass_value);
 int dir = stepsPerRevolution(compass_value, Vector);
 float steps = dir * STEPS_PER_DEGREE ;
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(800);
}

 Serial.println(stepsPerRevolution(compass_value, Vector));
 Serial.println(steps);
 Serial.print("azimuth: ");
 Serial.println(Vector.x);
 delay(1000);
}