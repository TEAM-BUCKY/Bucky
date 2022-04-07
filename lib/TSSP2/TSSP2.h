#ifndef TSSP2_H_
#define TSSP2_H_

class TSSP2{
    private:
        int pins[14], maxPulseWidth, maxSensorNumber, activeSensors;
        int ir_num = 14;
        int deltaPulseWidth = 2;

    public:
        TSSP2(int pins[14]); // change this to ir_num
        bool getSensorPin(int pin);
        void setAllSensorPinsInput();
        void getAllSensorPulseWidth(float pulseWidth[14], int timeLimit); // also change this to ir_num
};

#endif