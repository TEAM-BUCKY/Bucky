#ifndef CANWRAPPER_H
#define CANWRAPPER_H

class CANWrapper {
    public:
        void begin(int baudrate);
        void sendData(int id, int* data, int size);
        int available();
        char readData();
        void end();
};

#endif