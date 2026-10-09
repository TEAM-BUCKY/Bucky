// Power-on reset of the firmware's globals, by destroying and re-constructing each in place with
// the arguments main.cpp uses. Keep this list in sync with main.cpp and the header-inline globals;
// tests/test_firmware_programs.py guards it by comparing against the firmware objects' symbols
// (fw_globals.txt).
#include <memory>
#include <new>

#include "board/board.h"
#include "debug.h"
#include "firmware/firmware.h"
#include "hardware/io/gpio/pwm.h"
#include "hardware/io/serial/SerialPort.h"
#include "hardware/motor/Kicker.h"
#include "hardware/motor/Motors.h"
#include "hardware/sensors/Button.h"
#include "hardware/sensors/GPort.h"
#include "hardware/sensors/pos/Accelerometer.h"
#include "hardware/sensors/pos/Compass.h"
#include "hardware/sensors/pos/Sonar.h"

// main.cpp
extern Motors motorDriver;
extern I2CDMABus sensorI2C;
extern Compass compass;
extern Accelerometer accel;
extern Sonar sonar;
extern Kicker kicker;
extern GPort gPort1;
extern GPort gPort2;
extern UsbSerialPort host;
extern UartSerialPort<512, 256> bluetooth;

namespace {
template <typename T, typename... Args>
void renew(T& obj, Args&&... args) {
    std::destroy_at(&obj);
    ::new (static_cast<void*>(&obj)) T(std::forward<Args>(args)...);
}
}  // namespace

namespace sim {

void firmware_reboot() {
    renew(motorDriver, Board::MOTOR1, Board::MOTOR2, Board::MOTOR3);
    sensorI2C = I2CDMABus{};
    renew(compass);
    renew(accel);
    renew(sonar);
    renew(kicker, Board::KICKER);
    renew(gPort1);
    renew(gPort2);
    renew(host);
    renew(bluetooth);
    renew(dbg);
    renew(button1);
    renew(button2);
    pwm_sync = PwmSyncState{};
}

}  // namespace sim
