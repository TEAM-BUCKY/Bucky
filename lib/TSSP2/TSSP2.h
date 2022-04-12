#ifndef TSSP2_H_
#define TSSP2_H_

class TSSP2{
    private:
        int pins[14];
        int ir_num = 14;
        int deltaPulseWidth = 2;

        const float unitVectorX[14] = {1.000, 1.000, 0.623, 
        0.223, 0.223, -0.623, -0.901, -1.000, -0.901, -0.623, 
        -0.223, 0.223, 0.623, 0.000};
        const float unitVectorY[14] = {0.000, 0.000, 0.782, 
        0.975, 0.975, 0.782, 0.434, 0.000, -0.434, -0.782, 
        -0.975, -0.975, 0.782, 0.000};

    public:
        TSSP2(int pins[14]); // change this to ir_num
        bool getSensorPin(int pin);
        void setAllSensorPinsInput();
        void getAllSensorPulseWidth(int timeLimit); // also change this to ir_num
        void calcVector();
        void calcRTfromXY();
        int maxPulseWidth, maxSensorNumber, activeSensors;
	    int pulseWidth[14] = {};


        float IRInfo_x;
        float IRInfo_y;
        float IRInfo_radius;
        float IRInfo_theta;
};

#endif
