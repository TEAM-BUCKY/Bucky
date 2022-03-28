#ifndef LightGate_H_
#define LightGate_H_


class LightGate {
    private:
        int pin;
    public:
        LightGate(int pin);
        void setup();
        bool read();

};

#endif