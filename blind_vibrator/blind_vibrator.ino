#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <NewPing.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

#define GPS_RX D5
#define GPS_TX D6
#define trig_pin D7
#define echo_pin D8
#define VIBRATOR_PIN D3


TinyGPSPlus gps;


float longitude;
float latitude;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
NewPing sonar(trig_pin, echo_pin, 200); // Create a NewPing object

const int MPU_addr = 0x68;
float ax = 0, ay = 0, az = 0;
boolean fallDetected = false;
int Amp = 0;

char auth[] = "";
char ssid[] = "SSID";
char pass[] = "PASSWORD";

int16_t AcX, AcY, AcZ;

long writingTimer = 1;
long startTime = 0;
long waitTime = 0;


void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  Blynk.begin(auth, ssid, pass);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  pinMode(VIBRATOR_PIN, OUTPUT);

}

void loop() {
  delay(100); 
  int distance = sonar.ping_cm();
  Serial.println(distance);
  Blynk.run();
  if (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }
  vibrator(distance);
  mpu_read();
  calculateAmplitude();
  checkFall();
  /*waitTime = millis () - startTime;
if (waitTime > (writingTimer * 500) && i == 0)
  {  
startTime = millis (); 
}*/
  delay(100);
}
void mpu_read() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 6, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
}

void calculateAmplitude() {
  ax = (AcX - 2050) / 16384.00;
  ay = (AcY - 77) / 16384.00;
  az = (AcZ - 1947) / 16384.00;
  float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
  Amp = Raw_Amp * 10;
}

void checkFall() {
  if (Amp >= 15) { 
    if (!fallDetected) {
      
        Serial.println("Fall detected");
        fallDetected = true;        
         Blynk.logEvent("fall","Help at the location:"+String(gps.location.lat(), 6)+" , "+String(gps.location.lng(), 6));

      }
    }
   else {
    Blynk.virtualWrite(V1, String(gps.location.lat(), 6));
    Blynk.virtualWrite(V2, String(gps.location.lng(), 6));
    Serial.println("GPS updated");
    fallDetected=false;
 }
}
void vibrator(int distance)
{
  
  if (distance != 0) { // Make sure the distance measurement is valid
    int vibrationDuration = map(distance, 150, 10, 30,300);
    int beepintensity=map(distance, 150, 10, 1500,120);
    analogWrite(9,beepintensity);
    digitalWrite(VIBRATOR_PIN, HIGH);
    delay(vibrationDuration);
    digitalWrite(VIBRATOR_PIN, LOW);
     
  }
}
