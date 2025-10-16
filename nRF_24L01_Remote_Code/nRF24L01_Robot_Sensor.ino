//NRF24L01 Robot send data 


#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <Wire.h>


RF24 radio(7, 8); // CE, CSN
const byte address[6] = "00001";

struct sensor
{
  int moist;
  int N;
  int P;
  int K;
  float soilTemp;
  float envrTemp;
  float pressure;
  float altitute;
  byte  sensorNum;
};
sensor sensorData;

#define BMP280_ADDRESS 0x76
int soilMoistureValue = 0;
int soilmoisturepercent = 0;
#define ONE_WIRE_BUS 2
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_BMP280 bmp; // I2C

#define RE 6
#define DE 5

//const byte code[]= {0x01, 0x03, 0x00, 0x1e, 0x00, 0x03, 0x65, 0xCD};
const byte nitro[] = {0x01,0x03, 0x00, 0x1e, 0x00, 0x01, 0xe4, 0x0c};
const byte phos[] = {0x01,0x03, 0x00, 0x1f, 0x00, 0x01, 0xb5, 0xcc};
const byte pota[] = {0x01,0x03, 0x00, 0x20, 0x00, 0x01, 0x85, 0xc0};

byte values[11];
SoftwareSerial mod(4, 3);

unsigned long start_time, start_time_NPK; 
unsigned long timed_event, timed_event_NPK;
unsigned long current_time, current_time_NPK; // millis() function returns unsigned long
int npkTime = 0;
byte val1,val2,val3;



void setup() {
  Serial.begin(9600);
  sensors.begin();
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN);
  
  while ( !Serial ) //delay(100);   // wait for native usb
  Serial.println(F("BMP280 test"));
  unsigned status;
  status = bmp.begin(BMP280_ADDRESS);
  mod.begin(9600);
  pinMode(RE, OUTPUT);
  pinMode(DE, OUTPUT);
  
  timed_event = 2000; // after 1000 ms trigger the event
  timed_event_NPK = 250;
  current_time = millis();
  start_time = current_time; 
}


void loop() {
  //delay(5);
//  current_time = millis(); // update the timer every cycle
//  if (current_time - start_time >= timed_event) {
//    start_time = current_time;  // reset the timer
//    sensorCall();
//  }

  current_time_NPK = millis(); // update the timer every cycle
  if (current_time_NPK - start_time_NPK >= timed_event_NPK) {
    start_time_NPK = current_time_NPK;  // reset the timer
    npkTime += 1;
    
    if(npkTime == 1){
      val1 = nitrogen();
//      Serial.print("Nitrogen: ");
//      Serial.print(val1);
//      Serial.println(" mg/kg");
    }
    else if(npkTime == 2){
      val2 = phosphorous();
//      Serial.print("Phosphorous: ");
//      Serial.print(val2);
//      Serial.println(" mg/kg");
    }
     else if(npkTime == 3){
      val3 = potassium();
//      Serial.print("potassium: ");
//      Serial.print(val3);
//      Serial.println(" mg/kg");
      npkTime=0;
    }
  }
//  val1 = nitrogen();
//  val2 = phosphorous();
//  val3 = potassium();
    sensors.requestTemperatures();
    soilMoistureValue = analogRead(A0);  //put Sensor insert into soil  
    
    sensorData.moist = soilMoistureValue;
    sensorData.N = val1;
    sensorData.P = val2;
    sensorData.K = val3;
    sensorData.soilTemp = sensors.getTempCByIndex(0);
    sensorData.envrTemp = bmp.readTemperature();
    sensorData.pressure = bmp.readPressure();
    sensorData.altitute = bmp.readAltitude(1013.25);
    radio.stopListening();
    radio.write(&sensorData, sizeof(sensorData));
//    Serial.print("soilMoistureValue: ");
//  Serial.println(soilMoistureValue);
//  Serial.print("Soil Temperature: ");
//  Serial.print(sensors.getTempCByIndex(0));
//  Serial.print(" C  |  ");
//  Serial.print((sensors.getTempCByIndex(0) * 9.0) / 5.0 + 32.0);
//  Serial.println(" F ");
//  Serial.print(F("Air Temperature = "));
//  Serial.print(bmp.readTemperature());
//  Serial.println(" *C");
//
//  Serial.print(F("Pressure = "));
//  Serial.print(bmp.readPressure());
//  Serial.println(" Pa");
//
//  Serial.print(F("Approx altitude = "));
//  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
//  Serial.println(" m");
//
//  Serial.println();
}

//void sensorCall(){
//  byte val1,val2,val3;
//  val1 = nitrogen();
//  //delay(250);
//  val2 = phosphorous();
//  //delay(250);
//  val3 = potassium();
//  //delay(250);
//  Serial.print("Nitrogen: ");
//  Serial.print(val1);
//  Serial.println(" mg/kg");
//  Serial.print("Phosphorous: ");
//  Serial.print(val2);
//  Serial.println(" mg/kg");
//  Serial.print("potassium: ");
//  Serial.print(val3);
////  Serial.println(" mg/kg");
//
//  sensors.requestTemperatures();
//  soilMoistureValue = analogRead(A0);  //put Sensor insert into soil
//  Serial.print("soilMoistureValue: ");
//  Serial.println(soilMoistureValue);
//  Serial.print("Soil Temperature: ");
//  Serial.print(sensors.getTempCByIndex(0));
//  Serial.print(" C  |  ");
//  Serial.print((sensors.getTempCByIndex(0) * 9.0) / 5.0 + 32.0);
//  Serial.println(" F ");
//  Serial.print(F("Air Temperature = "));
//  Serial.print(bmp.readTemperature());
//  Serial.println(" *C");
//
//  Serial.print(F("Pressure = "));
//  Serial.print(bmp.readPressure());
//  Serial.println(" Pa");
//
//  Serial.print(F("Approx altitude = "));
//  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
//  Serial.println(" m");

//  Serial.println();
  //delay(2000);
//}




byte nitrogen(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(nitro,sizeof(nitro))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
//    Serial.print(values[i],HEX);
    }
//    Serial.println();
  }
  return values[4];
}
 
byte phosphorous(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(phos,sizeof(phos))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
//    Serial.print(values[i],HEX);
    }
//    Serial.println();
  }
  return values[4];
}
 
byte potassium(){
  digitalWrite(DE,HIGH);
  digitalWrite(RE,HIGH);
  delay(10);
  if(mod.write(pota,sizeof(pota))==8){
    digitalWrite(DE,LOW);
    digitalWrite(RE,LOW);
    for(byte i=0;i<7;i++){
    //Serial.print(mod.read(),HEX);
    values[i] = mod.read();
//    Serial.print(values[i],HEX);
    }
//    Serial.println();
  }
  return values[4];
}
