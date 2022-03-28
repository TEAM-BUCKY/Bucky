#ifndef LIGHTRING_H_
#define LIGHTRING_H_

class LightRing {
    private:
        int pins[5];
        void pinSelect(int pin);
    public:
        LightRing(int pins[9]); // first 5 are the pins (P0, P1, P2...), the other 3 are; CS, WR, EN and one PIN that is going to be our input
        void setup();
        int ringRead();

};


#endif