#ifndef SERIAL_H_
#define SERIAL_H_

class Serial_C {
    private:
        String message;
        bool isMaster;
    public:
        Serial_C(bool isMaster);
        void setup();
        void receive();
        void send(String message);
        void send(float speedM1, float speedM2, float speedM3);
        float motorSpeeds[3];
        String receive_string;
};

#endif