//************************************************************
// this is a simple example that uses the painlessMesh library
//
// 1. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 2. prints anything it receives to Serial.print
//
//
//************************************************************
#include "painlessMesh.h"

#define   MESH_PREFIX     "PureBrightness"
#define   MESH_PASSWORD   "2020WinterPractice"
#define   MESH_PORT       5555

#define HowManyJoints (3)
#define SIZEbufferJson (1024)
#define SIZEbufferJson_toString (1024)
#define PeriodCheckingCommandReceivedMilliSecond (5000)
#define MotorDefaultVelocity (600)
typedef struct state_joint{
  uint16_t id = 0;
  uint16_t velocity = MotorDefaultVelocity;
  int degreeCommanded = 0;
  int degreeHistory = 0;
  int timeCommandWritten = 0;
  unsigned char whetherCommandReceived = 0;
  unsigned char whetherCommandExecuted = 0;
};
StaticJsonDocument<SIZEbufferJson_toString> bufferJson_toString;
state_joint stateJoint[HowManyJoints];

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

void updateStateJoint(uint16_t indexJoint, int degreeCommanded){
  
}
bool sendMessage(uint16_t indexJoint) {

//  String msg = "cHello from node ";
//  msg += mesh.getNodeId();
//  mesh.sendBroadcast( msg );
  
  #if ARDUINOJSON_VERSION_MAJOR==6
    DynamicJsonDocument jsonBuffer(SIZEbufferJson);
    JsonObject msg = jsonBuffer.to<JsonObject>();
  #else
    DynamicJsonBuffer jsonBuffer;
    JsonObject& msg = jsonBuffer.createObject();
  #endif
  msg["id"] = stateJoint[indexJoint].id;
  msg["velocity"] = stateJoint[indexJoint].velocity;
  msg["degreeCommanded"] = stateJoint[indexJoint].degreeCommanded;
  msg["degreeHistory"] = stateJoint[indexJoint].degreeHistory;
  msg["timeCommandWritten"] = stateJoint[indexJoint].timeCommandWritten;
  msg["whetherCommandReceived"] = stateJoint[indexJoint].whetherCommandReceived;
  msg["whetherCommandExecuted"] = stateJoint[indexJoint].whetherCommandExecuted;
  
  String str;
  #if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, str);
  #else
    msg.printTo(str);
  #endif
    mesh.sendBroadcast(str);

  Serial.print("sent message to "); Serial.print(indexJoint); Serial.print(" == ");
  #if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, Serial);
  #else
    msg.printTo(Serial);
  #endif
    Serial.printf("\n");
    
  if(stateJoint[indexJoint].whetherCommandReceived)
    return 1;
  else
    return 0;
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void setup() {
  Serial.begin(115200);

//mesh.setDebugMsgTypes( ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE ); // all types on
  mesh.setDebugMsgTypes( ERROR | STARTUP );  // set before init() so that you can see startup messages

  mesh.init( MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT );
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

}

void loop() {
  // it will run the user scheduler as well
  mesh.update();

  if(Serial.available()){
    unsigned char separate = Serial.read();
    if(separate == 'j'){
      uint16_t indexJoint = Serial.read();
      indexJoint -= '0';
      if(indexJoint < HowManyJoints){
        unsigned char underBar = Serial.read();
        if(underBar == '_'){
          int degreeCommanded = Serial.parseInt();
          stateJoint[indexJoint].id = indexJoint;
          stateJoint[indexJoint].degreeCommanded = degreeCommanded;
          stateJoint[indexJoint].timeCommandWritten = millis();
          stateJoint[indexJoint].whetherCommandReceived = 0;
          stateJoint[indexJoint].whetherCommandExecuted = 0;
//          stateJoint[indexJoint].degreeCommanded = degreeCommanded;
//          while(sendMessage(indexJoint) != 1){
//            delay(PeriodCheckingCommandReceivedMilliSecond);
//          }
          sendMessage(indexJoint);          
        }
      }
    }
  }
}
