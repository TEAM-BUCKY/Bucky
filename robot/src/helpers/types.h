#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

class Percentage
{
    float value;

    public:
    Percentage(const float value)
    {
        this->value = fmaxf(fminf(value, 1.0f), 0.0f);
    }
    operator float() const { return value; }

    const Percentage& operator=(const float newValue)
    {
        this->value = fmaxf(fminf(newValue, 1.0f), 0.0f);
        return *this;
    }
};

class PercentageSigned
{
    float value;

    public:
    PercentageSigned(const float value)
    {
        this->value = fmaxf(fminf(value, 1.0f), -1.0f);
    }
    operator float() const { return value; }

    PercentageSigned& operator=(const float newValue)
    {
        this->value = fmaxf(fminf(newValue, 1.0f), -1.0f);
        return *this;
    }
};

#endif //ROBOT_TYPES_H