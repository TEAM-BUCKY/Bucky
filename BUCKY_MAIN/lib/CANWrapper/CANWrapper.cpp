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

void CANWrapper::setPins(int rx, int tx) {
    CAN.setPins(rx, tx);
}

void CANWrapper::sendFloat(int id, float data) {
    CAN.beginPacket(id);
    String a = String(data);
    for (int i = 0; i < sizeof(a); i++) {
        CAN.write(a[i]);
    }
    CAN.endPacket();   
}

void CANWrapper::sendBetterFloat(int id, float data) {
    CAN.beginPacket(id);
    String dataString = String(data);
    int length = sizeof(dataString);
    int decimal = dataString.substring(length - 2).toInt();
    int number = dataString.substring(0, length - 3).toInt();
    CAN.write(number);
    CAN.write(decimal);
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

void CANWrapper::setFilter(int id) {
    CAN.filter(id);
    CAN.filterExtended(id);
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