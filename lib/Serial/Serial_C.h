#ifndef SERIAL_H_
#define SERIAL_H_

class Serial_C {
    private:
        String message, receive_string;
        bool isMaster;
    public:
        Serial_C(bool isMaster);
        void setup();
        void receive();
        void send(float f1, float f2);
        float theta;
        float x;
        
};

#endif