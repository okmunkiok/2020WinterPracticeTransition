//************************************************************
// this is a simple example that uses the painlessMesh library
//
// 1. sends a silly message to every node on the mesh at a random time between 1 and 5 seconds
// 2. prints anything it receives to Serial.print
//
//
//************************************************************
#define DeviceID (2)
#include <Stepper.h>
#define EnvPIN (4)
#define DirPIN (33)
#define ClkPIN (12)
const uint16_t stepsPerRevolution = 200;
//const uint16_t motor_set_velocity = 600;
#define MotorDefaultVelocity (600)
Stepper myStepper(stepsPerRevolution, ClkPIN, DirPIN);

#include "painlessMesh.h"
#define   MESH_PREFIX     "PureBrightness"
#define   MESH_PASSWORD   "2020WinterPractice"
#define   MESH_PORT       5555
#define SIZEbufferJson (1024)
#define SIZEbufferJson_toString (1024)
#define PeriodCheckingCommandReceivedMilliSecond (5000)
typedef struct state_joint{
  uint16_t id = DeviceID;
  uint16_t velocity = MotorDefaultVelocity;
  int degreeCommanded = 0;
  int degreeAccumulated = 0;
  int timeCommandWritten = 0;
  unsigned char whetherCommandReceived = 0;
  unsigned char whetherCommandExecuted = 0;
};
StaticJsonDocument<SIZEbufferJson_toString> bufferJson_toString;
state_joint stateJoint;

Scheduler userScheduler; // to control your personal task
painlessMesh  mesh;

void executeCommand(){
//  print_current_state();
  myStepper.setSpeed(stateJoint.velocity);
  myStepper.step(stateJoint.degreeCommanded * 100/45);
  stateJoint.whetherCommandExecuted = 1;
}
bool syncStateJointWithMsg(const char *msg){
  DeserializationError error = deserializeJson(bufferJson_toString, msg);
  if (error) {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return 0;
  }

  if(bufferJson_toString["id"] != DeviceID)
    return 0;
  if(bufferJson_toString["timeCommandWritten"] == stateJoint.timeCommandWritten)
    return 0;

  stateJoint.velocity = bufferJson_toString["velocity"];
  stateJoint.degreeCommanded = bufferJson_toString["degreeCommanded"];
  stateJoint.timeCommandWritten = bufferJson_toString["timeCommandWritten"];
  stateJoint.whetherCommandReceived = 1;
  stateJoint.whetherCommandExecuted = 0;

//  print_current_state();
//  myStepper.setSpeed(stateJoint.velocity);
//  myStepper.step(stateJoint.degree*100/45);
//  stateJoint.whetherCommandExecuted = 1;

  return 1;
}

// User stub

bool sendMessage() {
  #if ARDUINOJSON_VERSION_MAJOR==6
    DynamicJsonDocument jsonBuffer(SIZEbufferJson);
    JsonObject msg = jsonBuffer.to<JsonObject>();
  #else
    DynamicJsonBuffer jsonBuffer;
    JsonObject& msg = jsonBuffer.createObject();
  #endif
  
  msg["id"] = stateJoint.id;
  msg["velocity"] = stateJoint.velocity;
  msg["degreeCommanded"] = stateJoint.degreeCommanded;
  msg["degreeAccumulated"] = stateJoint.degreeAccumulated;
  msg["timeCommandWritten"] = stateJoint.timeCommandWritten;
  msg["whetherCommandReceived"] = stateJoint.whetherCommandReceived;
  msg["whetherCommandExecuted"] = stateJoint.whetherCommandExecuted;
  
  String str;
  #if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, str);
  #else
    msg.printTo(str);
  #endif
    mesh.sendBroadcast(str);

  Serial.print("sent message to "); Serial.print("Server"); Serial.print(" == ");
  #if ARDUINOJSON_VERSION_MAJOR==6
    serializeJson(msg, Serial);
  #else
    msg.printTo(Serial);
  #endif
    Serial.printf("\n");
    
  if(stateJoint.whetherCommandReceived)
    return 1;
  else
    return 0;
}

// Needed for painless library
void receivedCallback( uint32_t from, String &msg ) {
  Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
  if(syncStateJointWithMsg(msg.c_str()) == 0)
    return;
  sendMessage();
  executeCommand();
  sendMessage();
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

//  userScheduler.addTask( taskSendMessage );
//  taskSendMessage.enable();
}

void loop() {
  // it will run the user scheduler as well
  mesh.update();
}
