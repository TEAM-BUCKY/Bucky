#ifndef SERIAL_H_
#define SERIAL_H_

class Serial_C {
    private:
        String message;
        bool isMaster;
    public:
        Serial_C(bool isMaster);
        void setup();
        String receive();
        void send(String message);
};

#endif