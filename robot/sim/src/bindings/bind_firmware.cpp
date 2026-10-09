// Bindings for the firmware's own classes (robot/src), the board constants and main.cpp's globals.
// Method names follow the C++ API so firmware code and Python tests read alike.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "bindings/bindings.h"
#include "board/board.h"
#include "core/world.h"
#include "hardware/io/cordic/cordic.h"
#include "hardware/io/encoder/Encoder.h"
#include "hardware/motor/Kicker.h"
#include "hardware/motor/Motors.h"
#include "hardware/sensors/Button.h"
#include "hardware/sensors/GPort.h"
#include "hardware/sensors/LineSensor.h"
#include "hardware/sensors/pos/Accelerometer.h"
#include "hardware/sensors/pos/Compass.h"
#include "hardware/sensors/pos/Sonar.h"
#include "helpers/Math.h"
#include "strategy/DigitalField.h"
#include "strategy/EKF.h"

namespace py = pybind11;

// Free functions with external linkage in the firmware.
uint32_t translatePosition(uint32_t pos);
float getSmoothFunction(float begin, float target, float totalSpeed, uint32_t time);

// main.cpp
extern Motors motorDriver;
extern I2CDMABus sensorI2C;
extern Compass compass;
extern Accelerometer accel;
extern Sonar sonar;
extern Kicker kicker;
extern GPort gPort1;
extern GPort gPort2;

namespace bucky_bindings {

namespace {

PinName pin_arg(const uint32_t p) { return static_cast<PinName>(p); }

std::vector<uint32_t> pins_of(const MotorPin& m) { return {static_cast<uint32_t>(m.inA), static_cast<uint32_t>(m.inB)}; }

}  // namespace

void bind_board(py::module_& m) {
    auto b = m.def_submodule("board", "The simulated PCB (robot/src/board/board_h562rg.h)");

    py::class_<MotorPin>(b, "MotorPin")
        .def(py::init([](const uint32_t a, const uint32_t bb) { return MotorPin{pin_arg(a), pin_arg(bb)}; }))
        .def_property_readonly("inA", [](const MotorPin& p) { return static_cast<uint32_t>(p.inA); })
        .def_property_readonly("inB", [](const MotorPin& p) { return static_cast<uint32_t>(p.inB); });
    py::class_<EncoderPins>(b, "EncoderPins")
        .def_property_readonly("pinA", [](const EncoderPins& p) { return static_cast<uint32_t>(p.pinA); })
        .def_property_readonly("pinB", [](const EncoderPins& p) { return static_cast<uint32_t>(p.pinB); });
    py::class_<SonarPins>(b, "SonarPins")
        .def_property_readonly("trigPin", [](const SonarPins& p) { return static_cast<uint32_t>(p.trigPin); })
        .def_property_readonly("echoPins", [](const SonarPins& p) {
            std::vector<uint32_t> v;
            for (const PinName e : p.echoPins) v.push_back(static_cast<uint32_t>(e));
            return v;
        });
    py::class_<GPortHardware>(b, "GPortHardware")
        .def_property_readonly("resetPin", [](const GPortHardware& h) { return static_cast<uint32_t>(h.resetPin); })
        .def_property_readonly("clockPin", [](const GPortHardware& h) { return static_cast<uint32_t>(h.clockPin); })
        .def_property_readonly("adcPin", [](const GPortHardware& h) { return static_cast<uint32_t>(h.adcPin); });

    const auto ref = py::return_value_policy::reference;
    b.attr("NAME") = Board::NAME;
    b.attr("MOTOR1") = py::cast(&Board::MOTOR1, ref);
    b.attr("MOTOR2") = py::cast(&Board::MOTOR2, ref);
    b.attr("MOTOR3") = py::cast(&Board::MOTOR3, ref);
    b.attr("MOTOR_PINS") = std::vector<std::vector<uint32_t>>{pins_of(Board::MOTOR1), pins_of(Board::MOTOR2), pins_of(Board::MOTOR3)};
    b.attr("ENCODERS") = std::vector<const EncoderPins*>{&Board::ENCODERS[0], &Board::ENCODERS[1], &Board::ENCODERS[2]};
    b.attr("SONAR") = py::cast(&Board::SONAR, ref);
    b.attr("KICKER") = static_cast<uint32_t>(Board::KICKER);
    b.attr("BUTTON1") = static_cast<uint32_t>(Board::BUTTON1);
    b.attr("BUTTON2") = static_cast<uint32_t>(Board::BUTTON2);
    b.attr("G_PORT1") = py::cast(&Board::G_PORT1, ref);
    b.attr("G_PORT2") = py::cast(&Board::G_PORT2, ref);
    b.attr("G_PORT1_KIND") = static_cast<int>(Board::G_PORT1_KIND);
    b.attr("G_PORT2_KIND") = static_cast<int>(Board::G_PORT2_KIND);
    b.attr("ANALOG") = std::vector<uint32_t>{Board::ANALOG[0], Board::ANALOG[1], Board::ANALOG[2], Board::ANALOG[3]};
#ifdef BOARD_HAS_KICKER
    b.attr("HAS_KICKER") = true;
#else
    b.attr("HAS_KICKER") = false;
#endif
#ifdef BOARD_HAS_GPORTS
    b.attr("HAS_GPORTS") = true;
#else
    b.attr("HAS_GPORTS") = false;
#endif
#ifdef BOARD_HAS_BUTTONS
    b.attr("HAS_BUTTONS") = true;
#else
    b.attr("HAS_BUTTONS") = false;
#endif
}

void bind_firmware(py::module_& m) {
    const auto ref = py::return_value_policy::reference;

    // ---- enums / plain data -------------------------------------------------------------------
    py::enum_<GSensorKind>(m, "GSensorKind")
        .value("None_", GSensorKind::None).value("IR", GSensorKind::IR).value("Line", GSensorKind::Line);
    py::enum_<LineColor>(m, "LineColor")
        .value("Red", LineColor::Red).value("Green", LineColor::Green)
        .value("Blue", LineColor::Blue).value("Dark", LineColor::Dark);
    py::enum_<CompassState>(m, "CompassState")
        .value("UNINIT", CompassState::UNINIT).value("BOOT_WAIT", CompassState::BOOT_WAIT)
        .value("CHECK_ID", CompassState::CHECK_ID).value("RESET_WAIT", CompassState::RESET_WAIT)
        .value("CONFIGURE", CompassState::CONFIGURE).value("SETTLE_WAIT", CompassState::SETTLE_WAIT)
        .value("SAMPLING", CompassState::SAMPLING).value("READY", CompassState::READY)
        .value("FAILED", CompassState::FAILED);
    py::enum_<EncoderBackend>(m, "EncoderBackend")
        .value("None_", EncoderBackend::None).value("Timer", EncoderBackend::Timer)
        .value("Exti", EncoderBackend::Exti);

    py::class_<I2CDMABus>(m, "I2CDMABus").def(py::init<>());
    m.def("make_sensor_bus", [] {
        I2CDMABus bus{};
        i2c_dma_init<Board::SENSOR_I2C_FREQ>(&bus, Board::SENSOR_I2C.instance, Board::SENSOR_I2C.sda,
                                             Board::SENSOR_I2C.scl, Board::SENSOR_I2C.dmaRx,
                                             Board::SENSOR_I2C.dmaRxRequest);
        return bus;
    }, "A fresh I2CDMABus on the board's sensor I2C, initialised like setupEnvironment() does");
    m.def("i2c_dma_probe", [](const I2CDMABus& bus, const uint8_t addr) { return i2c_dma_probe(&bus, addr); });

    py::class_<LineFrame>(m, "LineFrame")
        .def(py::init<>())
        .def("get", &LineFrame::get)
        .def_property_readonly("value", [](const LineFrame& f) {
            py::array_t<uint16_t> a({4, 16});
            auto r = a.mutable_unchecked<2>();
            for (int c = 0; c < 4; c++)
                for (int s = 0; s < 16; s++) r(c, s) = f.value[c][s];
            return a;
        });
    py::class_<LineRGB>(m, "LineRGB")
        .def(py::init([](const uint8_t r, const uint8_t g, const uint8_t b) { return LineRGB{r, g, b}; }))
        .def_readwrite("r", &LineRGB::r).def_readwrite("g", &LineRGB::g).def_readwrite("b", &LineRGB::b)
        .def("__iter__", [](const LineRGB& c) {
            return py::iter(py::make_tuple(c.r, c.g, c.b));
        })
        .def("__repr__", [](const LineRGB& c) {
            return "LineRGB(" + std::to_string(c.r) + ", " + std::to_string(c.g) + ", " + std::to_string(c.b) + ")";
        });

    py::class_<SonarReading>(m, "SonarReading")
        .def_property_readonly("distance", [](const SonarReading& r) {
            return std::vector<float>(r.distance, r.distance + SONAR_COUNT);
        })
        .def_property_readonly("valid", [](const SonarReading& r) {
            return std::vector<bool>(r.valid, r.valid + SONAR_COUNT);
        });

    py::class_<VectorXY>(m, "VectorXY")
        .def(py::init([](const float x, const float y) { return VectorXY{x, y}; }))
        .def_readwrite("x", &VectorXY::x).def_readwrite("y", &VectorXY::y);
    py::class_<SpeedRange>(m, "SpeedRange")
        .def_readonly("min", &SpeedRange::min).def_readonly("max", &SpeedRange::max)
        .def_readonly("scale", &SpeedRange::scale);
    py::class_<MotorEncoderValue>(m, "MotorEncoderValue")
        .def_readonly("ticks", &MotorEncoderValue::ticks).def_readonly("speed", &MotorEncoderValue::speed);

    py::class_<Point>(m, "Point")
        .def(py::init([](const float x, const float y) { return Point{x, y}; }), py::arg("x") = 0, py::arg("y") = 0)
        .def_readwrite("x", &Point::x).def_readwrite("y", &Point::y);
    py::class_<DigitalField>(m, "DigitalField")
        .def(py::init<>())
        .def_readwrite("robot", &DigitalField::robot)
        .def_readwrite("opponent", &DigitalField::opponent)
        .def_readwrite("ball", &DigitalField::ball);
    m.attr("FIELD_WIDTH") = FIELD_WIDTH;
    m.attr("FIELD_HEIGHT") = FIELD_HEIGHT;

    // ---- sensors ------------------------------------------------------------------------------
    py::class_<Compass>(m, "Compass")
        .def(py::init<>())
        .def("begin", &Compass::begin, py::keep_alive<1, 2>())
        .def("tick", &Compass::tick)
        .def("isReady", &Compass::isReady)
        .def("isFailed", &Compass::isFailed)
        .def("update", &Compass::update, py::arg("timeoutMs") = 10)
        .def("startRead", &Compass::startRead)
        .def("isReadComplete", &Compass::isReadComplete)
        .def("processRead", &Compass::processRead)
        .def("getHeading", &Compass::getHeading)
        .def("getOffset", &Compass::getOffset)
        .def("getRawX", &Compass::getRawX)
        .def("getRawY", &Compass::getRawY)
        .def("getRawZ", &Compass::getRawZ)
        .def("setCalibration", &Compass::setCalibration)
        .def("computeRotation", &Compass::computeRotation)
        .def("setPD", &Compass::setPD)
        .def("reset", &Compass::reset);

    py::class_<Accelerometer>(m, "Accelerometer")
        .def(py::init<>())
        .def("begin", &Accelerometer::begin, py::keep_alive<1, 2>())
        .def("isOk", &Accelerometer::isOk)
        .def("read", [](const Accelerometer& a) -> py::object {
            float x = 0, y = 0, z = 0;
            if (!a.read(x, y, z)) return py::none();
            return py::make_tuple(x, y, z);
        }, "(ax, ay, az) in g, or None on failure");

    py::class_<Sonar>(m, "Sonar")
        .def(py::init<>())
        .def("begin", &Sonar::begin)
        .def("startRead", &Sonar::startRead)
        .def("isReadComplete", &Sonar::isReadComplete)
        .def("processRead", &Sonar::processRead)
        .def("read", &Sonar::read);

    py::class_<GPort>(m, "GPort")
        .def(py::init<>())
        .def("begin", py::overload_cast<const GPortHardware&, GSensorKind>(&GPort::begin), py::keep_alive<1, 2>())
        .def("setLineOrder", [](GPort& p, const std::vector<LineColor>& order) {
            if (order.size() != 4) throw py::value_error("4 colours");
            p.setLineOrder(order.data());
        })
        .def("kind", &GPort::kind)
        .def("frameLength", &GPort::frameLength)
        .def("frameSequence", &GPort::frameSequence)
        .def("hasNewFrame", &GPort::hasNewFrame)
        .def("desyncCount", &GPort::desyncCount)
        .def("resetCount", &GPort::resetCount)
        .def("lineColorAt", &GPort::lineColorAt)
        .def("holdClock", &GPort::holdClock)
        .def("stepClock", &GPort::stepClock)
        .def("releaseClock", &GPort::releaseClock)
        .def("readIR", [](const GPort& p) -> py::object {
            uint16_t out[GPort::SENSORS];
            if (!p.readIR(out)) return py::none();
            return py::cast(std::vector<uint16_t>(out, out + GPort::SENSORS));
        }, "16 values (index = physical sensor) or None")
        .def("readLine", [](const GPort& p) -> py::object {
            LineFrame f{};
            if (!p.readLine(f)) return py::none();
            return py::cast(f);
        });
    m.attr("GPORT_SENSORS") = GPort::SENSORS;

    py::class_<LineSensor>(m, "LineSensor")
        .def(py::init<GPort&>(), py::keep_alive<1, 2>())
        .def("update", &LineSensor::update)
        .def("raw", &LineSensor::raw, ref)
        .def("reflected", &LineSensor::reflected)
        .def("calibrateWhite", &LineSensor::calibrateWhite, py::arg("frames") = 32)
        .def("calibrated", &LineSensor::calibrated)
        .def("whiteLevel", &LineSensor::whiteLevel)
        .def("hasSensor", &LineSensor::hasSensor)
        .def("color", &LineSensor::color)
        .def("showColor", &LineSensor::showColor)
        .def("celebrate", &LineSensor::celebrate, py::arg("durationMs") = 3000);

    py::class_<Button>(m, "Button")
        .def(py::init<>())
        .def("begin", [](Button& b, const uint32_t pin) { return b.begin(pin_arg(pin)); })
        .def("pressed", &Button::pressed);

    // ---- actuators ----------------------------------------------------------------------------
    py::class_<Motors>(m, "Motors")
        .def(py::init<MotorPin, MotorPin, MotorPin>())
        .def("init", [](Motors& mo, const float minSpeed, const float maxSpeed, const bool encoders) {
            mo.init(minSpeed, maxSpeed, encoders ? Board::ENCODERS : nullptr);
        }, py::arg("minSpeed") = MIN_SPEED, py::arg("maxSpeed") = MAX_SPEED, py::arg("encoders") = false,
           "encoders=True wires Board::ENCODERS, like setupEnvironment()")
        .def("updateAllMotors", &Motors::updateAllMotors)
        .def("syncUpdateAllMotors", &Motors::syncUpdateAllMotors)
        .def("driveDegrees", &Motors::driveDegrees, py::arg("degrees"), py::arg("scale") = 100, py::arg("rotation") = 0)
        .def("driveRadians", &Motors::driveRadians, py::arg("radians"), py::arg("scale") = 100, py::arg("rotation") = 0)
        .def("driveVector", &Motors::driveVector, py::arg("vector"), py::arg("rotation") = 0)
        .def("driveMotorsDirect", &Motors::driveMotorsDirect)
        .def("changeSpeed", &Motors::changeSpeed, py::arg("minSpeed") = MIN_SPEED, py::arg("maxSpeed") = MAX_SPEED)
        .def("getSpeedRange", &Motors::getSpeedRange)
        .def("setPIGains", &Motors::setPIGains)
        .def("setMaxTicksPerSec", py::overload_cast<float>(&Motors::setMaxTicksPerSec))
        .def("getEncoderSpeeds", [](const Motors& mo) {
            float out[3];
            mo.getEncoderSpeeds(out);
            return std::vector<float>(out, out + 3);
        })
        .def("getEncoderTicks", [](const Motors& mo) {
            uint16_t out[3];
            mo.getEncoderTicks(out);
            return std::vector<uint16_t>(out, out + 3);
        })
        .def("getEncoderValues", [](const Motors& mo) {
            MotorEncoderValue out[3];
            mo.getEncoderValues(out);
            return std::vector<MotorEncoderValue>(out, out + 3);
        })
        .def_static("wheelSpeeds", [](const float s, const float c, const float scale, const float rot) {
            float out[3];
            Motors::wheelSpeeds(s, c, scale, rot, out);
            return py::make_tuple(out[0], out[1], out[2]);
        }, py::arg("sinHeading"), py::arg("cosHeading"), py::arg("scale"), py::arg("rotation"));
    m.attr("MIN_SPEED") = MIN_SPEED;
    m.attr("MAX_SPEED") = MAX_SPEED;
    m.def("getSmoothFunction", &getSmoothFunction, py::arg("begin"), py::arg("target"),
          py::arg("totalSpeed"), py::arg("time"));

    py::class_<Kicker>(m, "Kicker")
        .def(py::init([](const uint32_t pin) { return Kicker(pin_arg(pin)); }))
        .def("init", &Kicker::init)
        .def("kick", &Kicker::kick)
        .def("stop", &Kicker::stop);

    // ---- strategy -----------------------------------------------------------------------------
    py::class_<State>(m, "State")
        .def_readonly("ballX", &State::ballX).def_readonly("ballY", &State::ballY)
        .def_readonly("ballSpeedX", &State::ballSpeedX).def_readonly("ballSpeedY", &State::ballSpeedY)
        .def_readonly("robotX", &State::robotX).def_readonly("robotY", &State::robotY)
        .def_readonly("robotTheta", &State::robotTheta)
        .def_readonly("speedX", &State::speedX).def_readonly("speedY", &State::speedY);
    py::class_<EKFNoise>(m, "EKFNoise")
        .def(py::init<>())
        .def_readwrite("accel", &EKFNoise::accel).def_readwrite("thetaDrift", &EKFNoise::thetaDrift)
        .def_readwrite("ballAccel", &EKFNoise::ballAccel).def_readwrite("ballSpeedInit", &EKFNoise::ballSpeedInit)
        .def_readwrite("sonarPos", &EKFNoise::sonarPos).def_readwrite("compassTheta", &EKFNoise::compassTheta)
        .def_readwrite("encoderSpeed", &EKFNoise::encoderSpeed).def_readwrite("irAngle", &EKFNoise::irAngle)
        .def_readwrite("irDistance", &EKFNoise::irDistance);
    py::class_<EKF>(m, "EKF")
        .def(py::init<>())
        .def("init", &EKF::init, py::arg("robotX"), py::arg("robotY"), py::arg("robotTheta"),
             py::arg("positionStd") = 20.0f, py::arg("thetaStd") = 0.3f)
        .def("setNoise", &EKF::setNoise)
        .def("predict", &EKF::predict)
        .def("updatePosition", &EKF::updatePosition)
        .def("updateHeading", &EKF::updateHeading)
        .def("updateSpeed", &EKF::updateSpeed)
        .def("updateBall", &EKF::updateBall)
        .def("getState", &EKF::getState)
        .def("getCovariance", [](const EKF& e) {
            const auto& c = e.getCovariance();
            py::array_t<float> a({9, 9});
            auto r = a.mutable_unchecked<2>();
            for (int i = 0; i < 9; i++)
                for (int j = 0; j < 9; j++) r(i, j) = c[i][j];
            return a;
        })
        .def("writeTo", &EKF::writeTo);

    // ---- free functions -----------------------------------------------------------------------
    m.def("translatePosition", &translatePosition);
    m.def("cordic_sin_cos", [](const float a) {
        float s, c;
        cordic_sin_cos(a, &s, &c);
        return py::make_tuple(s, c);
    });
    m.def("cordic_atan2", &cordic_atan2);
    m.def("cordic_atan2_mod", [](const float y, const float x) {
        float a, mod;
        cordic_atan2_mod(y, x, &a, &mod);
        return py::make_tuple(a, mod);
    });
    m.def("cordic_sqrt", &cordic_sqrt);
    m.def("cordic_ln", &cordic_ln);
    m.def("radiansToDegrees", &Math::radiansToDegrees);
    m.def("degreesToRadians", &Math::degreesToRadians);
    m.def("wrapRadians", &Math::wrapRadians);
    m.def("wrapDegrees", &Math::wrapDegrees);
    m.def("wrapSignedDegrees", &Math::wrapSignedDegrees);

    m.def("encoder_init", [](const uint8_t i) { encoder_init(i, Board::ENCODERS[i]); },
          "encoder_init(i, Board::ENCODERS[i])");
    m.def("encoder_get_ticks", &encoder_get_ticks);
    m.def("encoder_get_speed", &encoder_get_speed);
    m.def("encoder_reset", &encoder_reset);
    m.def("encoder_is_active", &encoder_is_active);
    m.def("encoder_update_speed", &encoder_update_speed);
    m.def("encoder_backend", &encoder_backend);

    // ---- main.cpp's globals -------------------------------------------------------------------
    auto g = m.def_submodule("globals",
        "main.cpp's global objects. While a program runs, use only side-effect-free getters "
        "between steps (anything that reads the clock raises SimMisuse).");
    g.def("motorDriver", [] { return &motorDriver; }, ref);
    g.def("sensorI2C", [] { return &sensorI2C; }, ref);
    g.def("compass", [] { return &compass; }, ref);
    g.def("accel", [] { return &accel; }, ref);
    g.def("sonar", [] { return &sonar; }, ref);
    g.def("kicker", [] { return &kicker; }, ref);
    g.def("gPort1", [] { return &gPort1; }, ref);
    g.def("gPort2", [] { return &gPort2; }, ref);
    g.def("irPort", [] { return Board::G_PORT1_KIND == GSensorKind::IR ? &gPort1 : &gPort2; }, ref);
    g.def("linePort", [] { return Board::G_PORT1_KIND == GSensorKind::Line ? &gPort1 : &gPort2; }, ref);
}

}  // namespace bucky_bindings
