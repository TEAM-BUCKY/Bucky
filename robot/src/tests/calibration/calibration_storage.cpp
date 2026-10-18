#include <tests/tests.h>
#include <tests/calibration/calibration_storage.h>
#include <EEPROM.h>

bool loadCalibration(Motors& md, Compass& compass) {
    StoredCalibration cal = {};
    EEPROM.get(0, cal);

    if (cal.magic != CALIBRATION_MAGIC) return false;

    for (uint8_t i = 0; i < 3; i++) {
        if (cal.maxTicksPerSec[i] < 100.0f || cal.maxTicksPerSec[i] > 10000.0f)
            return false;
        md.setMaxTicksPerSec(i, cal.maxTicksPerSec[i]);
    }

    if (cal.magValid) {
        compass.setCalibration(cal.magOffset[0], cal.magOffset[1], cal.magOffset[2],
                               cal.magScale[0],  cal.magScale[1],  cal.magScale[2]);
    }
    return true;
}
