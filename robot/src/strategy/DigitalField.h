#ifndef ROBOT_DIGITALFIELD_H
#define ROBOT_DIGITALFIELD_H


constexpr float FIELD_WIDTH = 182; // CM
constexpr float FIELD_HEIGHT = 243; // CM

struct Point
{
    float x;
    float y;
};

struct DigitalField
{
    Point robot;
    Point opponent;

    Point ball;
};


#endif //ROBOT_DIGITALFIELD_H