#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <RtcDS1302.h>

ThreeWire myWire(13,14,12); // dat, CLK, reset
RtcDS1302<ThreeWire> Rtc(myWire);

//recieve boardaddress
uint8_t broadcastAddress1[] = {0xe4, 0x65, 0xb8, 0x78, 0xd4, 0xf0}; //esp32 38 pin yellow
uint8_t broadcastAddress2[] = {0xa8, 0x42, 0xe3, 0xab, 0x85, 0x10}; //a8:42:e3:ab:85:10 simple esp

#define PI 3.1415926535897932384626433832795
#define DEG_TO_RAD 0.01745329252
#define RAD_TO_DEG 57.2957795131
#define HOURS_TO_DEG 15.0
#define DEG_TO_HOURS 1.0 / 15.0


float Ra2Degree(float ra_hr, float ra_min, float ra_sec, float *Ra_deg){
  *Ra_deg = 15 * ra_hr + 15 * (ra_min / 60.0) + 15 * (ra_sec / 3600.0);
  return *Ra_deg ;
}

float Dec2Degree(float dec_deg, float dec_min, float dec_sec, float *Dec_deg){
  *Dec_deg = dec_deg + dec_min / 60.0 + dec_sec / 3600.0;
   return *Dec_deg ;
}
float UT2Degree(float ut_hr, float ut_min, float ut_sec, float *UT_hr){
  *UT_hr = ut_hr + (ut_min / 60.0) + (ut_sec / 3600.0);
  return  *UT_hr ;
}

float calculateGST(int year, int month, int day, float ut) {
        // Convert month and year to accommodate for the Julian calendar
        if  (month < 1 || month > 12) {
            year -= 1;
            month += 12;
        }

        int A = year / 100;
        int B = 2 - A + (A / 4);

        float JD = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day + ut / 24.0 - 1524.5;
        float JD0 = int(365.25 * (year + 4716)) + int(30.6001 * (month + 1)) + day + B - 1524.5;
        
        float D = JD - 2451545.0;
        float D0 = JD0 - 2451545.0;
        float T = D / 36525.0;

        float GMST = 280.46061837 + 360.98564736629 * D0 + T * T * (0.000387933 - T / 38710000.0);
        GMST = fmod(GMST, 360.0);

        if (GMST < 0) {
            GMST += 360.0;
        }

        return GMST * DEG_TO_HOURS; //GST
    }


float calculateHA(float ra, float longitude, float gst) {
        // Convert longitude to hours
        float longitude_hours = longitude * DEG_TO_HOURS;

        float lst = gst + longitude_hours;
Serial.println(lst);
        float ha = lst - ra;
    // if (ha < 0);
    //     ha = abs(ha);
        return ha * HOURS_TO_DEG;
    }

//az/alt formola
struct Vector2 {
    float x;
    float y;
}; struct Vector2 Vector;

class AltitudeAzimuth {
public:
    static Vector2 calAltitudeAzimuth(float ra, float dec, float latitude, float longitude, float ha) {
        Vector2 coordinate;

        float tan_y = sin(ha * DEG_TO_RAD);
        float tan_x = cos(ha * DEG_TO_RAD) * sin(latitude * DEG_TO_RAD) - tan(dec * DEG_TO_RAD) * cos(longitude * DEG_TO_RAD);

        float azimuth = atan2(tan_y, tan_x) * RAD_TO_DEG;
        if (azimuth > 0){ 
            azimuth = 360 - azimuth;
        }
        if (azimuth < 0){
            // azimuth = 360 - map(azimuth,-180, 180,0,360);
            azimuth = 180 + azimuth;
        }

        float altitude = asin((sin(latitude * DEG_TO_RAD) * sin(dec * DEG_TO_RAD)) +
                                (cos(latitude * DEG_TO_RAD) * cos(dec * DEG_TO_RAD) * cos(ha * DEG_TO_RAD)));
        altitude = altitude * RAD_TO_DEG;
        coordinate.x = azimuth;
        coordinate.y = altitude;

        return coordinate;
    }
};
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  Serial.print("Packet to: ");
  // Copies the sender mac address to a string
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
  Serial.print(" send status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}
#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
    char datestring[26];

    snprintf_P(datestring, 
            countof(datestring),
            PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
            dt.Month(),
            dt.Day(),
            dt.Year(),
            dt.Hour(),
            dt.Minute(),
            dt.Second() );
    Serial.print(datestring);
}

void setup() {
  Serial.begin(115200);

Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
    printDateTime(compiled);
    Serial.println();


 WiFi.mode(WIFI_STA);
 
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
   
  // register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // register first peer  
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // register second peer  
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

RtcDateTime dt = Rtc.GetDateTime();
  float RaDec_degree[2];
    // Initialize variables
    float ra = Ra2Degree(11, 22, 39.6, &RaDec_degree[0]);
    float dec = Dec2Degree(54,27 ,28.8, &RaDec_degree[1]);
  float ut_hour[1]; 
    float UT = UT2Degree(dt.Hour(), dt.Minute(), dt.Second(), &ut_hour[0]);
    // float ra = 165.931;
    // float dec = 61.75;
    float latitude = 13.2791845;
    float longitude = 100.9242725;
    float gst = calculateGST( dt.Year(),  dt.Month(),  dt.Day(), UT);
    Serial.println(gst);
    float ha = calculateHA(ra, longitude, gst);
    Serial.print(ha);
    // Call calAltitudeAzimuth function63
    Vector2 result = AltitudeAzimuth::calAltitudeAzimuth(ra, dec, latitude, longitude, ha);
    Serial.print("Azimuth: ");
    Serial.println(result.x);
    Serial.print("Altitude: ");
    Serial.println(result.y);
    
}

void loop() {
 struct Vector2 Vector;
  struct Vector2 Vector2;
  Vector.x ;
  Vector.y ;
  Vector2.x ;
  Vector2.y ;

RtcDateTime dt = Rtc.GetDateTime();
 float RaDec_degree[2];
    // Initialize variables
    float ra = Ra2Degree(5, 35, 24, &RaDec_degree[0]);
    float dec = Dec2Degree((-5),27 ,00, &RaDec_degree[1]);
  float ut_hour[1]; 
    float UT = UT2Degree(dt.Hour(), dt.Minute(), dt.Second(), &ut_hour[0]);
    // float ra = 165.931;
    // float dec = 61.75;
    float latitude = 13.2791845;
    float longitude = 100.9242725;
    float gst = calculateGST( dt.Year(),  dt.Month(),  dt.Day(), UT);
       Serial.println(gst);
    float ha = calculateHA(ra, longitude, gst);
    Serial.println(ha);
    // Call calAltitudeAzimuth function63
    Vector2 = AltitudeAzimuth::calAltitudeAzimuth(ra, dec, latitude, longitude, ha);
  Serial.print("Azimuth: ");
  Serial.println(Vector2.x);
  Serial.print("Altitude: ");
  Serial.println(Vector2.y);

  esp_err_t result1 = esp_now_send(0, (uint8_t *) &Vector2, sizeof(struct Vector2));
   
  if (result1 == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(2000);
}