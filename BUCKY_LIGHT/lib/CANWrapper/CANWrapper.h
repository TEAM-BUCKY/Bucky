#ifndef CANWRAPPER_H
#define CANWRAPPER_H

double irTheta, irRadius;

class CANWrapper {
    private:
        int baudrate, rx, tx;
    public:
        CANWrapper(int baudrate, int rx, int tx);
        int setup();
        void end();
        int sendDouble(double message, int id);
        void registerCallback();

};

#endif
