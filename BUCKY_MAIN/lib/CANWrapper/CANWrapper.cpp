#include <CANWrapper.h>
#include <CAN.h>
#include <stdio.h>

CANWrapper::CANWrapper(int baudrate, int rx, int tx){
  CANWrapper::baudrate = baudrate;
  CANWrapper::rx = rx;
  CANWrapper::tx = tx;
};

int CANWrapper::setup(){
  CAN.setPins(rx, tx);
  return CAN.begin(baudrate);
}

int CANWrapper::sendDouble(double message, int id){
  union {
    double message;
    byte tempArray[8];
  } u;
  u.message = message;
  CAN.beginPacket(id);
  for (int i = 0; i < 8;i++){
    CAN.write(u.tempArray[i]);
  }
  return CAN.endPacket();
}

void receiveDouble(int test){
  int packetId = CAN.packetId();
  union {
    double message;
    byte tempArray[8];
  } u;
  for (int i = 0; i < 8; i++){
    u.tempArray[i] = CAN.read();
  }

  if (packetId == 1){
    irTheta = u.message;
  }
  else if (packetId == 2) {
    irRadius = u.message;
  }
}

void CANWrapper::registerCallback(){
  CAN.onReceive(receiveDouble);
}

void CANWrapper::end() {
    CAN.end();
}