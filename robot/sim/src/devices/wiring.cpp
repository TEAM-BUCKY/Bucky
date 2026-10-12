// The H562RG board: every device model wired to the pins and buses board_h562rg.h assigns.
#include "board/board.h"
#include "devices/devices.h"

namespace sim {

Devices& devices() {
    static const PinName pwm_pads[PwmProbe::PADS] = {
        Board::MOTOR1.inA, Board::MOTOR1.inB, Board::MOTOR2.inA, Board::MOTOR2.inB,
        Board::MOTOR3.inA, Board::MOTOR3.inB, Board::KICKER,
    };
    // Leaked like the world: device models must outlive any parked firmware thread.
    static Devices* d = [] {
        auto* dev = new Devices{
            Lis2mdl{},
            Lsm303Acc{},
            GPortBoard(Board::G_PORT1, Board::G_PORT1_KIND),
            GPortBoard(Board::G_PORT2, Board::G_PORT2_KIND),
            SonarArray(Board::SONAR),
            EncoderPlant(Board::ENCODERS),
            PwmProbe(pwm_pads),
        };
        World& w = world();
        for (Device* x : std::initializer_list<Device*>{&dev->compass, &dev->accel, &dev->gport1,
                                                        &dev->gport2, &dev->sonar, &dev->encoders,
                                                        &dev->pwm}) {
            w.add_device(x);
            x->reset();
        }
        io::i2c_attach(Board::SENSOR_I2C.instance, &dev->compass);
        io::i2c_attach(Board::SENSOR_I2C.instance, &dev->accel);
        return dev;
    }();
    return *d;
}

}  // namespace sim
