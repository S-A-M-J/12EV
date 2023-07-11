///////////////////////////////////////////////////
// Based on pybot robotic arm project from JJRobots
// Ported and changed by S.A.M
// Last updated on 11.07.23
///////////////////////////////////////////////////

//BLE
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define SERVICE_UUID "a5f125c0-7cec-4334-9214-58cffb8706c0"
#define CHARACTERISTIC_UUID_RX "a5f125c2-7cec-4334-9214-58cffb8706c0"
#define CHARACTERISTIC_UUID_TX "a5f125c1-7cec-4334-9214-58cffb8706c0"

BLECharacteristic twelveEVbotTxCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_WRITE);
BLECharacteristic twelveEVbotRxCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_NOTIFY);

bool BLEConnected = false;

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLEConnected = true;
    //Serial.println("ble connected");
  }
  void onDisconnect(BLEServer* pServer) {
    BLEConnected = false;
    pServer->getAdvertising()->start();
    Serial.println("BLE disconnected");
  }
};

//------------------------BLUETOOTH_CALLBACK---------------------------------------------------------------------------
class incomingCallbackHandler : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* twelveEVbotTxCharacteristic) {
    char* incomingMessage = (char*)twelveEVbotTxCharacteristic->getValue().c_str();
    Serial.print("message received: ");
    Serial.println(incomingMessage);
    char test[128];
    //memset(test,'\0',sizeof(test));
    strcpy(test, incomingMessage);
    strcat(test, "\0");
    Serial.println(test);
    char* messagePart;
    char delimiter[] = ",";
    messagePart = strtok(test, ",");
    Serial.println(messagePart);
    if (strcmp(messagePart, "#JJAI") == 0) {
      messagePart = strtok(NULL, delimiter);
      iCH1 = atoi(messagePart);
      messagePart = strtok(NULL, delimiter);
      iCH2 = atoi(messagePart);
      messagePart = strtok(NULL, delimiter);
      iCH3 = atoi(messagePart);
      messagePart = strtok(NULL, delimiter);
      iCH4 = atoi(messagePart); //electromagnet
      messagePart = strtok(NULL, delimiter);
      iCH5 = atoi(messagePart);
      iCH5 = 0;
      iCH7 = 0;
      iCH8 = 0;
      newMessage = 1;
      mode = 2;
    } else if (strcmp(messagePart, "#EV12Start") == 0){
      Serial.println("Starting Disassembly");
      EV12running = true;
      EV12Disassembly();
    } else if (strcmp(messagePart, "#moveToBin") == 0){
      messagePart = strtok(NULL, delimiter);
      int16_t binNumber = atoi(messagePart);
      Serial.println(binNumber);
      moveToBin(binNumber);
    } else if (strcmp(messagePart, "#calibrate") == 0){
      mode = 5;
      newMessage++;
    }
  }
};

void setupBLE(){
  
  // Create the BLE Device
  BLEDevice::init("12EVbot");

  // Create the BLE Server
  BLEServer* pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService* comService = pServer->createService(SERVICE_UUID);

  comService->addCharacteristic(&twelveEVbotTxCharacteristic);
  comService->addCharacteristic(&twelveEVbotRxCharacteristic);
  twelveEVbotRxCharacteristic.addDescriptor(new BLE2902());
  twelveEVbotTxCharacteristic.setCallbacks(new incomingCallbackHandler());
  // Start the service
  comService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
}

void sendBLEData() {
  char value[64] = "";
  strcat(value, "");
  
  twelveEVbotRxCharacteristic.setValue(value);
  twelveEVbotRxCharacteristic.notify();
}

int32_t ExtractParamInt4b(uint8_t pos) {
  union {
    unsigned char Buff[4];
    int32_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 3];
  u.Buff[1] = (unsigned char)MsgBuffer[pos + 2];
  u.Buff[2] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[3] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}

int16_t ExtractParamInt2b(uint8_t pos) {
  union {
    unsigned char Buff[2];
    int16_t d;
  } u;
  u.Buff[0] = (unsigned char)MsgBuffer[pos + 1];
  u.Buff[1] = (unsigned char)MsgBuffer[pos];
  return (u.d);
}


// Read messages from Serial
// Message: 8 channels (16 bits)
void USBMsgRead()
{
  uint8_t i;
  // New bytes available to process?
  while (Serial.available() > 0) {
    // We rotate the Buffer (we could implement a ring buffer in future)
    for (i = 0; i < (MSGMAXLEN - 1); i++) {
      MsgBuffer[i] = MsgBuffer[i + 1];
    }
    MsgBuffer[MSGMAXLEN - 1] = (unsigned char)Serial.read();
    Serial.print((char)MsgBuffer[MSGMAXLEN-1]);
    ParseMsg(0);
  }
}

void ParseMsg(uint8_t interface)
{
  Serial.println((char)MsgBuffer[0]);
  // Message JJAH: Hello Message (This is a presentation message when the API connect to the robot) => Enable WIFI output messages (if the message comes from a wifi interface)
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'H')) {
    Serial.println("->MSG: JJAH: HELLO!");
    newMessage = 0; // No message to proccess on main code...
    working = false;
    trajectory_processing = false;
    if (interface == 1)
      enable_udp_output = true;
  }

  // Message JJAM: Manual control mode (Direct Kinematic)
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'M')) {
    Serial.print("->MSG: JJAM:");
    iCH1 = ExtractParamInt2b(4);  // axis1
    iCH2 = ExtractParamInt2b(6);  // axis2
    iCH3 = ExtractParamInt2b(8);  // z
    iCH4 = ExtractParamInt2b(10); // servo1 => orientation
    iCH5 = ExtractParamInt2b(12); // servo2 => gripper
    iCH6 = ExtractParamInt2b(14);
    iCH7 = ExtractParamInt2b(16);
    iCH8 = ExtractParamInt2b(18);
    Serial.print(iCH1);
    Serial.print(" ");
    Serial.print(iCH2);
    Serial.print(" ");
    Serial.println(iCH3);
    mode = 1;
    newMessage = 1;
    working = true; // Work to do...
    trajectory_processing = false;
    if (interface == 1)
      enable_udp_output = true;
  }
  // Message JJAI: Automatic control mode (Inverse Kinematic)
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'I')) {
    Serial.println("->MSG: JJAI:");
    for(int i=0;i<20;i++){
      Serial.println(MsgBuffer[i]);
    }
    
    iCH1 = ExtractParamInt2b(4);  // target x
    iCH2 = ExtractParamInt2b(6);  // target y
    iCH3 = ExtractParamInt2b(8);  // target z
    iCH4 = ExtractParamInt2b(10); // target wrist orientation(servo1)servo1 => orientation
    iCH5 = ExtractParamInt2b(12); // target gripper position(servo2)
    iCH6 = ExtractParamInt2b(14); // solution type : 0 positive, 1 negative (elbow position)
    iCH7 = ExtractParamInt2b(16);
    iCH8 = ExtractParamInt2b(18);
    Serial.println("parameters");
    Serial.println((int)iCH1);
    Serial.println(iCH2);
    Serial.println(iCH3);
    Serial.println(iCH4);
    Serial.println(iCH5);
    Serial.println(iCH6);
    Serial.println(iCH7);
    Serial.println(iCH8);
    mode = 2;
    newMessage = 1;
    working = true; // Work to do...
    trajectory_processing = false;
    if (interface == 1)
      enable_udp_output = true;
  }
  // Message JJAT: Trajectory mode (Inverse Kinematic)
  // In this mode the robot draws a trajectory defined by a set of points. The robot don´t decelerate on intermediate points.
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'T')) {
    //Serial.println("->MSG: JJAT:");
    trajectory_processing = true;   // trajectory proccesing mode...
    working = false;  
    iCH1 = ExtractParamInt2b(4);   //target Angle1
    iCH2 = ExtractParamInt2b(6);   //target Angle2
    iCH3 = ExtractParamInt2b(8);   //target z
    iCH4 = ExtractParamInt2b(10);  //target wrist orientation(servo1)servo1 => orientation
    iCH5 = ExtractParamInt2b(12);  //target gripper position(servo2)
    iCH6 = ExtractParamInt2b(14);  //solution type : 0 positive, 1 negative (elbow position)
    iCH7 = ExtractParamInt2b(16);  //Point number : from 0 to 50 (max points)
    iCH8 = ExtractParamInt2b(18);  //Last point: 0: intermediate point, 1:last point
    mode = 3; // Line Trajectory mode

    Serial.print("-->JJAT ");
    Serial.print(iCH7);
    Serial.print(" :");
    Serial.print(iCH1);
    Serial.print(" ");
    Serial.print(iCH2);
    Serial.print(" ");
    Serial.println(iCH3);
    
    if (iCH7 == 0) {
      // First point? => empty trajectory vector
      for (int i = 0; i < MAX_TRAJECTORY_POINTS; i++)
        for (int j = 0; j < 5; j++)
          trajectory_vector[i][j] = 0;
    }
    if ((iCH7 >= MAX_TRAJECTORY_POINTS)||(iCH7<0)) {
      Serial.println("-->TR POINT OVERFLOW!");
      iCH7 = MAX_TRAJECTORY_POINTS - 1;
    }
    else {
      trajectory_vector[iCH7][0] = iCH1/100.0;
      trajectory_vector[iCH7][1] = iCH2/100.0;
      if (iCH3 == NODATA)
        trajectory_vector[iCH7][2] = NODATA;
      else
        trajectory_vector[iCH7][2] = iCH3/100.0;
      trajectory_vector[iCH7][3] = iCH4;
      trajectory_vector[iCH7][4] = iCH5;
    }
    if (iCH8 == 1) { // iCH8==1 means Last point=>execute
      trajectory_mode = true;  // Execute trajectory
      trajectory_num_points = iCH7;
      trajectory_point = 0;
      working = true;  // Work to do...
    }
    else
      trajectory_mode = false;

    newMessage = 1;
    if (interface == 1)
      enable_udp_output = true;
  }

  // Setup message "JJAS" Set robot speed and acc
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'S')) {
    Serial.print("->MSG: JJAS:");
    iCH1 = ExtractParamInt2b(4);
    iCH2 = ExtractParamInt2b(6);
    iCH3 = ExtractParamInt2b(8);
    iCH4 = ExtractParamInt2b(10);
    iCH5 = ExtractParamInt2b(12);  // Trajectory speed (default=20) More speed->less accuracy
    Serial.print(" SPEED XY:");
    Serial.print(iCH1);
    //Serial.print(" ");
    //Serial.print((MAX_SPEED_M1 * float(iCH1)) / 100.0);
    Serial.print(" SPEED Z:");
    Serial.print(iCH2);
    //Serial.print(" ");
    //Serial.print((MAX_SPEED_M3 * float(iCH2)) / 100.0);
    Serial.print(" ACC XY:");
    Serial.print(iCH3);
    Serial.print("ACC Z:");
    Serial.print(iCH4);
    Serial.print(" TRAJ S:");
    Serial.print(iCH5);
    
    trajectory_tolerance_M1 = (iCH5/10.0f)*M1_AXIS_STEPS_PER_UNIT;
    trajectory_tolerance_M2 = (iCH5/10.0f)*M2_AXIS_STEPS_PER_UNIT;
    trajectory_tolerance_M3 = (iCH5/10.0f)*M3_AXIS_STEPS_PER_UNIT;

    Serial.print(" ");
    Serial.print(trajectory_tolerance_M1);
    Serial.print(" ");
    Serial.print(trajectory_tolerance_M2);
    Serial.print(" ");
    Serial.println(trajectory_tolerance_M3);

    configSpeed((MAX_SPEED_M1 * float(iCH1)) / 100.0, (MAX_SPEED_M2 * float(iCH1)) / 100.0, (MAX_SPEED_M3 * float(iCH2)) / 100.0);
    configAcceleration((MAX_ACCEL_M1 * float(iCH3)) / 100.0, (MAX_ACCEL_M2 * float(iCH3)) / 100.0, (MAX_ACCEL_M3 * float(iCH4)) / 100.0);
    if (interface == 1)
      enable_udp_output = true;
  }

  // robot motors calibration message "JJAC" 
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'C')) {
    Serial.println("->MSG: JJAC:");
    working = 1;
    newMessage = 1;
    mode = 5;        // Calibration mode
    if (interface == 1)
      enable_udp_output = true;
  }

  // Emergency stop message "JJAE" Stops the robot and disble motors
  if ((char(MsgBuffer[0]) == 'J') && (char(MsgBuffer[1]) == 'J') && (char(MsgBuffer[2]) == 'A') && (char(MsgBuffer[3]) == 'E')) {
    Serial.println("->MSG: JJAE:");
    working = 1;
    newMessage = 1;
    mode = 4;        // Emergency stop mode
    if (interface == 1)
      enable_udp_output = true;
  }
}


void debugMsg()
{
  Serial.print("->mode:");
  Serial.print(mode);
  Serial.print(" CH:");
  Serial.print(iCH1);
  Serial.print(" ");
  Serial.print(iCH2);
  Serial.print(" ");
  Serial.print(iCH3);
  Serial.print(" ");
  Serial.print(iCH4);
  Serial.print(" ");
  Serial.print(iCH5);
  Serial.print(" ");
  Serial.print(iCH6);
  Serial.print(" ");
  Serial.print(iCH7);
  Serial.print(" ");
  Serial.println(iCH8);
}
