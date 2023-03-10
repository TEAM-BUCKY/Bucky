#ifndef TSSP2_H_
#define TSSP2_H_

class TSSP2{
    private:
        int pins[16];
        int ir_num = 16;
        int deltaPulseWidth = 2;

        const float unitVectorX[16] = {0.000, 0.000, 0.707, 0.923, 1.000, 0.923, 0.707, 0.383, 0.000, -0.383, -0.707, -0.923, -1.000, -0.923, -0.707, 0.000};
        const float unitVectorY[16] = {1.000, 1.000, 0.707, 0.383, 0.000, -0.383, -0.707, -0.923, -1.000, -0.923, -0.707, -0.383, 0.000, 0.383, 0.707, 1.000};

    public:
        TSSP2(int pins[16]); // change this to ir_num
        bool getSensorPin(int pin);
        void setAllSensorPinsInput();
        void getAllSensorPulseWidth(int timeLimit); // also change this to ir_num
        void calcVector();
        void calcRTfromXY();
        int maxPulseWidth, maxSensorNumber, activeSensors;
	    int pulseWidth[16] = {};


        float IRInfo_x;
        float IRInfo_y;
        float IRInfo_radius;
        float IRInfo_theta;
};

#endif