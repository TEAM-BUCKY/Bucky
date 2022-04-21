#ifndef HMC5883L_LIB
#define HMC5883L_LIB

class HMC5883L_Calibrate {
    private:
        int minX = 0;
        int maxX = 0;
        int minY = 0;
        int maxY = 0;
    public:
        HMC5883L_Calibrate();
        void setup();
        void calibrate();
        int offX = 0;
        int offY = 0;
};

class HMC5883L_Compass {
    private:
        float DECLINATION, calibration;
        int offX, offY, avgAmount;
    public:
        HMC5883L_Compass(float DECLINATION, int avgAmount);
        float calibrate();
        void setup(int offX, int offY);
        float calculate();
};

#endif