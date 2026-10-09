#ifndef ROBOT_DIGITALFIELD_H
#define ROBOT_DIGITALFIELD_H


constexpr float FIELD_WIDTH = 122; // CM
constexpr float FIELD_HEIGHT = 183; // CM

constexpr float FIELD_SIDE = 30; // CM

constexpr float GOAL_AREA = 45; // CM
constexpr float GOAL_WIDTH = 45; // CM

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