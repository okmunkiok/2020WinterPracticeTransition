#define SERIAL_PRINT
#define DISPLAY_LED


// setupt display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define DefaultTextSize (1)
#define FisrtLineCursorY (17)
#define HeightEachLineHas (10)


// part A: UWB sensor measuring distance(range)
//
//
#include <SPI.h>
#include <DW1000.h>

// connection pins
const uint8_t PIN_RST = 15; // reset pin
const uint8_t PIN_IRQ = 17; // irq pin
const uint8_t PIN_SS = 2; // spi select pin

// messages used in the ranging protocol
// TODO replace by enum
#define BYTE_FOR_ANCHORS (1)
#define NUMBER_OF_ANCHORS (3)
float tagHeight = 0;
float distance[NUMBER_OF_ANCHORS];
float x[3] = {0, 7.68, 0};
float xSquare[3];
float y[3] = {0, 0, 13.5};
float ySquare[3];
float z[3] = {0, 0, 0};
float deltaZ_Square[3];
float dPrime_Square[3];
float matrixA[6];
float matrixAT[6];
float matrixAT_multiply_A[4];
float inverseMatrixAT_multiply_A[4];
float inverseMatrixAT_multiply_A_multiply_AT[6];
float matrixB[3];
//float array_delta_x_of_anchor[NUMBER_OF_ANCHORS];
//float array_delta_y_of_anchor[NUMBER_OF_ANCHORS];
float position_x = 0;
float position_y = 0;
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
// message flow state
volatile byte expectedMsgId = POLL_ACK;
// message sent/received state
volatile boolean sentAck = false;
volatile boolean receivedAck = false;
//boolean protocolFailed = false;
// timestamps to remember
DW1000Time timePollSent;
DW1000Time timePollReceived;
DW1000Time timePollAckSent;
DW1000Time timePollAckReceived;
DW1000Time timeRangeSent;
DW1000Time timeRangeReceived;
DW1000Time timeRangeReportSent;
DW1000Time timeRangeReportReceived;
// last computed range/time
DW1000Time timeComputedRange;
// data buffer
#define ORIGINAL_LEN_DATA (16)
#define LEN_DATA (ORIGINAL_LEN_DATA + BYTE_FOR_ANCHORS)
#define DATA_INDEX_BYTE_FOR_ANCHORS (LEN_DATA - 1)
byte which_anchor = 0;
byte data[LEN_DATA];
// watchdog and reset period
uint32_t lastActivity;
uint32_t resetPeriod = 250;
// reply times (same on both sides for symm. ranging)
uint16_t replyDelayTimeUS = 300000;
// ranging counter (per second)
uint16_t successRangingCount = 0;
uint32_t rangingCountPeriod = 0;
float samplingRate = 0;
#define MaxCountReset (10)
uint16_t countReset = 0;


// part B: compute current position of tag using 3 distances
//
//
//float x[3] = {0, 0, 5};
//float x_square[3];
//float y[3] = {0, 8, 0};
//float y_square[3];
//float z[3] = {0, 0, 0};
//float delta_z_square[3];
//float distance[3] = {3.605, 5.385, 4.242};
float distance_prime_square[3];
//float position_x = 0;
//float position_y = 0;
float height_z = 0;

typedef struct _matrix {
  float* ptr;
  int column;
  int row;
} MATRIX;

void noteActivity() {
    // update activity timestamp, so that we do not reach "resetPeriod"
    lastActivity = millis();
}

void resetInactive() {
    // tag sends POLL and listens for POLL_ACK
//    countReset++;
//    if
    Serial.print("resetInactive because for "); Serial.print(resetPeriod); Serial.println(" MilliSeconds, there was no response");
    which_anchor = 0;
    expectedMsgId = POLL_ACK;
    transmitPoll();
    noteActivity();
}

void handleSent() {
    // status change on sent success
    sentAck = true;
}

void handleReceived() {
    // status change on received success
    receivedAck = true;
}

void transmitPoll() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = POLL;
    data[DATA_INDEX_BYTE_FOR_ANCHORS] = which_anchor;
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
}

void transmitRange() {
    DW1000.newTransmit();
    DW1000.setDefaults();
    data[0] = RANGE;
    data[DATA_INDEX_BYTE_FOR_ANCHORS] = which_anchor;

    // delay sending the message and remember expected future sent timestamp
    DW1000Time deltaTime = DW1000Time(replyDelayTimeUS, DW1000Time::MICROSECONDS);
    timeRangeSent = DW1000.setDelay(deltaTime);
    timePollSent.getTimestamp(data + 1);
    timePollAckReceived.getTimestamp(data + 6);
    timeRangeSent.getTimestamp(data + 11);
    DW1000.setData(data, LEN_DATA);
    DW1000.startTransmit();
    //Serial.print("Expect RANGE to be sent @ "); Serial.println(timeRangeSent.getAsFloat());
}

void receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void computeRangeAsymmetric() {
    // asymmetric two-way ranging (more computation intense, less error prone)
    DW1000Time round1 = (timeRangeReceived - timePollAckSent).wrap();
    DW1000Time reply1 = (timeRangeSent - timePollAckReceived).wrap();
    DW1000Time round2 = (timeRangeReportReceived - timeRangeSent).wrap();
    DW1000Time reply2 = (timeRangeReportSent - timeRangeReceived).wrap();
    DW1000Time tof = (round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2);
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

void computeRangeSymmetric() {
    // symmetric two-way ranging (less computation intense, more error prone on clock drift)
    DW1000Time tof = ((timePollAckReceived - timePollSent) - (timePollAckSent - timePollReceived) +
                      (timeRangeReceived - timePollAckSent) - (timeRangeSent - timePollAckReceived)) * 0.25f;
    // set tof timestamp
    timeComputedRange.setTimestamp(tof);
}

void get_position(){
  for(int i = 0; i < 3; i++){
    dPrime_Square[i] = distance[i] * distance[i] - (xSquare[i] + ySquare[i] + deltaZ_Square[i]);
  }
  matrixB[0] = -2 * dPrime_Square[0] + dPrime_Square[1] + dPrime_Square[2];
  matrixB[1] = dPrime_Square[0] - 2 * dPrime_Square[1] + dPrime_Square[2];
  matrixB[2] = dPrime_Square[0] + dPrime_Square[1] - 2 * dPrime_Square[2];

  float (*ptrInverseMatrixAT_multiply_A_multiply_AT)[3] = (float (*)[3]) inverseMatrixAT_multiply_A_multiply_AT;
  position_x = 0;
  for(int i = 0; i < 3; i++){
    position_x += ptrInverseMatrixAT_multiply_A_multiply_AT[0][i] * matrixB[i];
  }

  position_y = 0;
  for(int i = 0; i < 3; i++){
    position_y += ptrInverseMatrixAT_multiply_A_multiply_AT[1][i] * matrixB[i];
  }

  Serial.print("position_x: "); Serial.print(position_x); Serial.print(" m\t");
  Serial.print("position_y: "); Serial.print(position_y); Serial.println(" m\t");
  
  display.setTextSize(DefaultTextSize);
  display.setTextSize(DefaultTextSize);
  display.setCursor(0, FisrtLineCursorY + HeightEachLineHas  * 3);
  display.setTextColor(WHITE, BLACK);
  display.print("position_x: "); display.print(position_x); display.println(" m");
  display.setTextSize(DefaultTextSize);
  display.setTextSize(DefaultTextSize);
  display.setCursor(0, FisrtLineCursorY + HeightEachLineHas  * 4);
  display.setTextColor(WHITE, BLACK);
  display.print("position_y: "); display.print(position_y); display.println(" m");
}

void setup() {
//  setup part B: compute current position of tag using 3 distances
  for(int i = 0; i < 3; i++){
    xSquare[i] = x[i] * x[i];
    ySquare[i] = y[i] * y[i];
    deltaZ_Square[i] = (tagHeight - z[0]) * (tagHeight - z[0]);
  }
  
  float (*ptrMatrixA)[2] = (float (*)[2]) matrixA;
  ptrMatrixA[0][0] = 4*x[0] - 2*x[1] - 2*x[2];
  ptrMatrixA[0][1] = 4*y[0] - 2*y[1] - 2*y[2];
  ptrMatrixA[1][0] = -2*x[0] + 4*x[1] - 2*x[2];
  ptrMatrixA[1][1] = -2*y[0] + 4*y[1] - 2*y[2];
  ptrMatrixA[2][0] = -2*x[0] - 2*x[1] + 4*x[2];
  ptrMatrixA[2][1] = -2*y[0] - 2*y[1] + 4*y[2];

  float (*ptrMatrixAT)[3] = (float (*)[3]) matrixAT;
  for(int i = 0; i < 2; i++){
    for(int j = 0; j < 3; j++){
      ptrMatrixAT[i][j] = ptrMatrixA[j][i];
    }
  }

  float (*ptrMatrixAT_multiply_A)[2] = (float (*)[2]) matrixAT_multiply_A;
  for(int i = 0; i < 2; i++){
    for(int j = 0; j < 2; j++){
      float sum = 0;
      for(int k = 0; k < 3; k++){
        sum += ptrMatrixAT[i][k] * ptrMatrixA[k][j];
      }
      ptrMatrixAT_multiply_A[i][j] = sum;
    }
  }

  float (*ptrInverseMatrixAT_multiply_A)[2] = (float (*)[2]) inverseMatrixAT_multiply_A;
  float determinant = ptrMatrixAT_multiply_A[0][0] * ptrMatrixAT_multiply_A[1][1] - ptrMatrixAT_multiply_A[0][1] * ptrMatrixAT_multiply_A[1][0];
  ptrInverseMatrixAT_multiply_A[0][0] = ptrMatrixAT_multiply_A[1][1] / determinant;
  ptrInverseMatrixAT_multiply_A[0][1] = -1 * ptrMatrixAT_multiply_A[0][1] / determinant;
  ptrInverseMatrixAT_multiply_A[1][0] = -1 * ptrMatrixAT_multiply_A[1][0] / determinant;
  ptrInverseMatrixAT_multiply_A[1][1] = ptrMatrixAT_multiply_A[0][0] / determinant;

  float (*ptrInverseMatrixAT_multiply_A_multiply_AT)[3] = (float (*)[3]) inverseMatrixAT_multiply_A_multiply_AT;
  for(int i = 0; i < 2; i++){
    for(int j = 0; j < 3; j++){
      float sum = 0;
      for(int k = 0; k < 2; k++){
        sum += ptrInverseMatrixAT_multiply_A[i][k] * ptrMatrixAT[k][j];
      }
      ptrInverseMatrixAT_multiply_A_multiply_AT[i][j] = sum;
    }
  }
  
  
//  setup part A: UWB sensor measuring distance(range)
//
//
    // DEBUG monitoring
    Serial.begin(115200);
    Serial.println(F("### DW1000-arduino-ranging-tag ###"));
    // initialize the driver
    DW1000.begin(PIN_IRQ, PIN_RST);
    DW1000.select(PIN_SS);
    Serial.println("DW1000 initialized ...");
    // general configuration
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(2);
    DW1000.setNetworkId(10);
    DW1000.enableMode(DW1000.MODE_LONGDATA_RANGE_LOWPOWER);
    DW1000.commitConfiguration();
    Serial.println(F("Committed configuration ..."));
    // DEBUG chip info and registers pretty printed
    char msg[128];
    DW1000.getPrintableDeviceIdentifier(msg);
    Serial.print("Device ID: "); Serial.println(msg);
    DW1000.getPrintableExtendedUniqueIdentifier(msg);
    Serial.print("Unique ID: "); Serial.println(msg);
    DW1000.getPrintableNetworkIdAndShortAddress(msg);
    Serial.print("Network ID & Device Address: "); Serial.println(msg);
    DW1000.getPrintableDeviceMode(msg);
    Serial.print("Device mode: "); Serial.println(msg);
    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    // anchor starts by transmitting a POLL message
    receiver();
    transmitPoll();
    noteActivity();

//  setup display
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
  Serial.println(F("SSD1306 allocation failed"));
  for(;;);
  }
  delay(2000);
  display.clearDisplay();
}

void loop() {
    int32_t curMillis = millis();
    if (!sentAck && !receivedAck) {
        // check if inactive
        if (millis() - lastActivity > resetPeriod) {
            resetInactive();
        }
        return;
    }
    // continue on any success confirmation
    if (sentAck) {
        sentAck = false;
        byte msgId = data[0];
        if (msgId == POLL) {
            DW1000.getTransmitTimestamp(timePollSent);
            //Serial.print("Sent POLL @ "); Serial.println(timePollSent.getAsFloat());
        } else if (msgId == RANGE) {
            DW1000.getTransmitTimestamp(timeRangeSent);
            noteActivity();
        }
    }
    if (receivedAck) {
        receivedAck = false;
        // get message and parse
        DW1000.getData(data, LEN_DATA);
        byte msgId = data[0];
        if (msgId != expectedMsgId) {
            // unexpected message, start over again
            //Serial.print("Received wrong message # "); Serial.println(msgId);
            expectedMsgId = POLL_ACK;
            transmitPoll();
            return;
        }
        if (msgId == POLL_ACK) {
            DW1000.getReceiveTimestamp(timePollAckReceived);
            expectedMsgId = RANGE_REPORT;
            transmitRange();
            noteActivity();
        } else if (msgId == RANGE_REPORT) {
            DW1000.getReceiveTimestamp(timeRangeReportReceived);
            expectedMsgId = POLL_ACK;
//            float curRange;
//            memcpy(&curRange, data + 1, 4);
            timePollAckSent.setTimestamp(data + 1);
            timeRangeReceived.setTimestamp(data + 6);
            timeRangeReportSent.setTimestamp(data + 11);
//            (re-)compute range as two-way ranging is done
            computeRangeAsymmetric(); // CHOSEN RANGING ALGORITHM
//            transmitRangeReport(timeComputedRange.getAsMicroSeconds());
//            transmitRangeReport();
//            float distance = timeComputedRange.getAsMeters();

            distance[which_anchor] = timeComputedRange.getAsMeters();
//            computePosition();
//            Serial.print("(x, y) == "); Serial.print(position_x); Serial.println(position_y);
            get_position();
            
//            Serial.print("position_x: "); Serial.print(position_x); Serial.print(" m\t//\t");
//            Serial.print("position_y: "); Serial.print(position_y); Serial.println(" m");
            
            Serial.print("distance_"); Serial.print(which_anchor); Serial.print(": "); Serial.print(distance[which_anchor]); Serial.print(" m");
            Serial.print("\t RX power: "); Serial.print(DW1000.getReceivePower()); Serial.print(" dBm");
            Serial.print("\t Sampling: "); Serial.print(samplingRate); Serial.println(" Hz");
            
            display.setTextSize(DefaultTextSize);
            display.setCursor(0, FisrtLineCursorY + HeightEachLineHas  * which_anchor);
            display.setTextColor(WHITE, BLACK);
            display.print("distance_"); display.print(which_anchor); display.print(": "); display.print(distance[which_anchor]); display.println(" m");
            display.display();
            
            if(which_anchor < NUMBER_OF_ANCHORS - 1){
              which_anchor++;
            }
            else{
              which_anchor = 0;
            }
            
            
            //Serial.print("FP power is [dBm]: "); Serial.print(DW1000.getFirstPathPower());
            //Serial.print("RX power is [dBm]: "); Serial.println(DW1000.getReceivePower());
            //Serial.print("Receive quality: "); Serial.println(DW1000.getReceiveQuality());
            // update sampling rate (each second)
            successRangingCount++;
            if (curMillis - rangingCountPeriod > 1000) {
                samplingRate = (1000.0f * successRangingCount) / (curMillis - rangingCountPeriod);
                rangingCountPeriod = curMillis;
                successRangingCount = 0;
            }
            transmitPoll();
            noteActivity();
        } else if (msgId == RANGE_FAILED) {
            expectedMsgId = POLL_ACK;
            transmitPoll();
            noteActivity();
        }
    }
}
