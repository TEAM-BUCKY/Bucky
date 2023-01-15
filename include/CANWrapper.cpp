#include "CANWrapper.h"
#include "CAN.h"

void CANWrapper::begin(int baudrate) {
  // Initialize the CAN bus
  CAN.begin(baudrate);
}

void CANWrapper::parsePacket() {
  CAN.parsePacket();
}

void CANWrapper::sendData(int id, int* data, int size) {
    CAN.beginPacket(id);
    // check if data is a list if so loop through it and send each element
    for (int i = 0; i < size; i++) {
        CAN.write(data[i]);
    }
    CAN.endPacket();
}

int CANWrapper::available() {
    return CAN.available();
}

char* CANWrapper::readData() {
    char* data = new char[255];
    int i = 0;
    while (CAN.available()) {
        char byte = (char)CAN.read();
        data[i++] = byte;
    }
    data[i]='\0';
    return data;
}

void CANWrapper::sendDataString(int id, char* data, int size) {
    CAN.beginPacket(id);
    for (int i = 0; i < size; i++) {
        CAN.write(data[i]);
    }
    CAN.endPacket();
}

void CANWrapper::end() {
    CAN.end();
}
