//
// Created by koen on 2/28/26.
//

#ifndef BUCKY_COMPASS_H
#define BUCKY_COMPASS_H

class Compass
{
    private:
        int address;
    public:
        explicit Compass(const int address = 0x18) : address(address) {}

        float getHeading();


};

#endif //BUCKY_COMPASS_H