#ifndef CANWRAPPER_H
#define CANWRAPPER_H

double irTheta, irRadius;

class CANWrapper {
    private:
        int baudrate, rx, tx;
    public:
        CANWrapper(int baudrate = 500E3, int rx = 4, int tx = 5);
        int setup();
        void end();
        int sendDouble(double message, int id);
        void registerCallback();

};

#endif
