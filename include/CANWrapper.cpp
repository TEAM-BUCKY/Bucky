#include "CANWrapper.h"
#include "CAN.h"

void CANWrapper::begin(int baudrate) {
  // Initialize the CAN bus
  CAN.begin(baudrate);
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

char CANWrapper::readData() {
    char data;
    while (CAN.available()) {
        data += CAN.read();
    }
    return data;
}

void CANWrapper::end() {
    CAN.end();
}
