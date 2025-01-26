#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#define dirPin 40
#define stepPin 41
#define STEPS_PER_DEGREE 4.12045

const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;
 
//Structure example to receive data
//Must match the sender structure
struct Vector2 {
    float x;
    float y;
}; struct Vector2 Vector;

//callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&Vector, incomingData, sizeof(struct Vector2));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("altitude: ");
  Serial.println(Vector.y);
  Serial.println();
}

int currentPosition = 0; // 0 to 200 for 0 to 180 degrees
const int maxSteps = 370.8405; // steps for 90 degrees (for a 200-step motor)

void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  Wire.begin();
Wire.beginTransmission(MPU_addr);
Wire.write(0x6B);
Wire.write(0);
Wire.endTransmission(true);
   pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  WiFi.mode(WIFI_STA);

  //Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  AcX=Wire.read()<<8|Wire.read();
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  int xAng = map(AcX,minVal,maxVal,-90,90);
  int yAng = map(AcY,minVal,maxVal,-90,90);
  int zAng = map(AcZ,minVal,maxVal,-90,90);
  
  x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
  y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
  z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
float Alt = z - Vector.y ;
float Alt_m = Alt * STEPS_PER_DEGREE;
float steps = Alt_m;
  // Spin the stepper motor 1 revolution slowly:
   digitalWrite(dirPin, LOW);
  for (int i = 0; i < steps; i++) {
    // These four lines result in 1 step:
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1600);
   
  }
  
  Serial.print("AngleX= ");
  Serial.println(x);
  
  Serial.print("AngleY= ");
  Serial.println(y);
  
  Serial.print("AngleZ= ");
  Serial.println(z);
  Serial.println("-----------------------------------------");
  delay(400);
}