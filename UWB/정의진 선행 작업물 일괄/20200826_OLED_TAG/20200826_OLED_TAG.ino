/**
 * 
 * @todo
 *  - move strings to flash (less RAM consumption)
 *  - fix deprecated convertation form string to char* startAsTag
 *  - give example description
 */
#include <SPI.h>
#include "DW1000Ranging.h"

////////////////////////////////////////
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

////////////////////////////////////////


typedef struct* anchorAddress {

  uint16_t address = 0;
  float range[10] = 0.0f;
  int count = 0;
  float sum = 0.0f;
  int oneCycle = 0;

}anchorA;

 uint16_t checkAddress(anchorA *x){
  malloc
  }

anchorA* node[3];

// connection pins
const uint8_t PIN_SCK = 18; 
const uint8_t PIN_MOSI = 23;
const uint8_t PIN_MISO = 19;
const uint8_t PIN_SS = 2;
const uint8_t PIN_RST = 15;
const uint8_t PIN_IRQ = 17;

void setup() {
  Serial.begin(115200);
  delay(1000);


//////////////////////////////////////////////////////////////////////////////

  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);

//////////////////////////////////////////////////////////////////////////////

  //init the configuration
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  
  //we start the module as a tag
  DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_LONGDATA_RANGE_ACCURACY);
}

void loop() {
  DW1000Ranging.loop();
}

void newRange() {
  Serial.print("from: "); Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print("\t Range: "); Serial.print(DW1000Ranging.getDistantDevice()->getRange()); Serial.print(" m");
  Serial.print("\t RX power: "); Serial.print(DW1000Ranging.getDistantDevice()->getRXPower()); Serial.println(" dBm");

    if( getSA[0] == 0){
  getSA[0] = DW1000Ranging.getDistantDevice()->getShortAddress(); }
  else if(getSA[0] != DW1000Ranging.getDistantDevice()->getShortAddress() && getSA[1] == 0 ){
   getSA[1] = DW1000Ranging.getDistantDevice()->getShortAddress();}
  else if(getSA[0] != DW1000Ranging.getDistantDevice()->getShortAddress() && getSA[1] != DW1000Ranging.getDistantDevice()->getShortAddress() && getSA[2] == 0 ){
   getSA[2] = DW1000Ranging.getDistantDevice()->getShortAddress();}


  if(getSA[0] == DW1000Ranging.getDistantDevice()->getShortAddress()){

    if(count_0 == 10){
      count_0 = 0;
      full_0 = 1;
      }
      
    getR[0][count_0] = DW1000Ranging.getDistantDevice()->getRange();
    count_0++;
    }
    else if(getSA[1] == DW1000Ranging.getDistantDevice()->getShortAddress()){

    if(count_1 == 10){
      count_1 = 0;
      full_1 = 1;
      }
      
    getR[1][count_1] = DW1000Ranging.getDistantDevice()->getRange();
    count_1++;
    }
    else if(getSA[2] == DW1000Ranging.getDistantDevice()->getShortAddress()){

    if(count_2 == 10){
      count_2 = 0;
      full_2 = 1;
      }
    
    getR[2][count_2] = DW1000Ranging.getDistantDevice()->getRange();
    count_2++;
    }


  if(full_0){
    sum_0 = 0;
  for(int i=0;i<10;i++)
    sum_0 += getR[0][i];
    }
  
  if(full_1){
    sum_1 = 0;
  for(int i=0;i<10;i++)
    sum_1 += getR[1][i];
    }

  if(full_2){
    sum_2 = 0;
  for(int i=0;i<10;i++)
    sum_2 += getR[2][i];
    }
   
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print("From: ");
  display.print(getSA[0], HEX);
  display.setCursor(0, 10);
  display.print("Range: ");
  display.print(sum_0/10); display.print("m");
  display.setCursor(0, 20);
  display.print("From: ");
  display.print(getSA[1], HEX);
  display.setCursor(0, 30);
  display.print("Range: ");
  display.print(sum_1/10); display.print("m");
  display.setCursor(0, 40);
  display.print("From: ");
  display.print(getSA[2], HEX);
  display.setCursor(0, 50);
  display.print("Range: ");
  display.print(sum_2/10); display.print("m");
  display.display();
  
}

void newDevice(DW1000Device* device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device* device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
}
