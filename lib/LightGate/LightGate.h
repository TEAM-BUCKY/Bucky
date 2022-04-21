#ifndef LightGate_H_
#define LightGate_H_


class LightGate {
    private:
        int pin;
    public:
        LightGate(int pin);
        void setup(); // Set pin to INPUT
        bool read(); // Returns True if ball obstructs laser

};

#endif