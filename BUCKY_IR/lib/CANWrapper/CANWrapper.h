#ifndef CANWRAPPER_H
#define CANWRAPPER_H

class CANWrapper {
    public:
        void begin(int baudrate);
        void sendData(int id, int* data, int size);
        void sendDataString(int id, char* data, int size);
        void sendFloat(int id, float data);
        void sendBetterFloat(int id, float data);
        int available();
        char* readData();
        void parsePacket();
        void end();
};

#endif
