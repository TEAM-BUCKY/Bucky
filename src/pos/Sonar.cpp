#include "Sonar.h"

void Sonar::init() const
{
    pinMode(trigPin, OUTPUT);
}

double Sonar::getDistance() const
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    const uint32_t duration = pulseIn(echoPin, HIGH, 30000);
    return 0.034 * duration / 2;
}

void Sonar::changePins(const SonarPins pins)
{
    this->trigPin = pins.trigPin;
    this->echoPin = pins.echoPin;
}

Sonar sonar1({});
Sonar sonar2({});
Sonar sonar3({});
Sonar sonar4({});

void setupSonar(const SonarPins pins1, const SonarPins pins2, const SonarPins pins3, const SonarPins pins4)
{
    sonar1.changePins(pins1);
    sonar2.changePins(pins2);
    sonar3.changePins(pins3);
    sonar4.changePins(pins4);

    sonar1.init();
    sonar2.init();
    sonar3.init();
    sonar4.init();
}

SonarReading readSonars()
{
    SonarReading reading;
    reading.sonar1 = sonar1.getDistance();
    reading.sonar2 = sonar2.getDistance();
    reading.sonar3 = sonar3.getDistance();
    reading.sonar4 = sonar4.getDistance();

    return reading;
}