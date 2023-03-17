#ifndef TSSP2_H_
#define TSSP2_H_

class TSSP2{
    private:
        int pins[16];
        const int ir_num = 16;
        const int deltaPulseWidth = 2;
        bool getSensorPin(int pin);
        const double unitVectorX[16] = {0.000, 0.000, 0.707, 0.923, 1.000, 0.923, 0.707, 0.383, 0.000, -0.383, -0.707, -0.923, -1.000, -0.923, -0.707, 0.000};
        const double unitVectorY[16] = {1.000, 1.000, 0.707, 0.383, 0.000, -0.383, -0.707, -0.923, -1.000, -0.923, -0.707, -0.383, 0.000, 0.383, 0.707, 1.000};

    public:
        TSSP2(int pins[16]);
        void setup();
        void update(int timeLimit);
        int maxPulseWidth, maxSensorNumber, activeSensors;
	    int pulseWidth[16] = {};
        float IRInfo_x;
        float IRInfo_y;
        float IRInfo_radius;
        float IRInfo_theta;
};

#endif