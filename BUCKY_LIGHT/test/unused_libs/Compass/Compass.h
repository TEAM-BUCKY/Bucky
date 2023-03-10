#ifndef COMPASS_H_
#define COMPASS_H_

class Compass {
    private:
        float DECLINATION;
        int calibration;
    public:
        Compass(float DECLINATION);
        void setup();
        float compassRead();
        void compassCalibrate(int n);
        float compassCalculate(double heading);
};



#endif