#ifndef DRIVERS_STEPPER_MOTOR_DRV8711_HPP
#define DRIVERS_STEPPER_MOTOR_DRV8711_HPP

#include "hal/synchronous_interfaces/SynchronousSpi.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "infra/timer/Timer.hpp"

namespace drivers::stepper_motor
{
    class Drv8711Sync
    {
    public:
        struct Control
        {
            enum class DeadTime
            {
                _400ns,
                _450ns,
                _650ns,
                _850ns,
            };

            enum class AmplifierGain
            {
                _5,
                _10,
                _20,
                _40
            };

            enum class Step
            {
                full,
                half,
                _1_4,
                _1_8,
                _1_16,
                _1_32,
                _1_64,
                _1_128,
                _1_256,
            };

            enum class Direction
            {
                direct,
                inverse,
            };

            enum class StallDetection
            {
                internal,
                external,
            };

            bool enableMotor = false;
            Direction direction = Direction::direct;
            bool advanceOneStep = false;
            Step step = Step::_1_4;
            StallDetection stallDetection = StallDetection::internal;
            AmplifierGain amplifierGain = AmplifierGain::_5;
            DeadTime deadTime = DeadTime::_850ns;
        };

        struct Torque
        {
            enum class BackEMFSampleThreshold
            {
                _50us,
                _100us,
                _200us,
                _300us,
                _400us,
                _600us,
                _800us,
                _1000us,
            };

            BackEMFSampleThreshold backEMFSampleThreshold = BackEMFSampleThreshold::_100us;
            uint8_t fullScaleCurrent = 0xff;
        };

        struct Off
        {
            enum class PwmMode
            {
                internalIndexer,
                externalControl,
            };

            constexpr uint32_t FixedOffTimeInNanoSeconds()
            {
                return (fixedOffTime * 500) + 500;
            }

            PwmMode pwmMode = PwmMode::internalIndexer;
            uint8_t fixedOffTime = 0x30;
        };

        struct Blank
        {
            constexpr uint32_t BlankingTimeInNanoSeconds()
            {
                auto value = blankingTime * 20;

                if (value < 1000) return 1000;

                return value;
            }

            bool adaptativeBlankingTime = false;
            uint8_t blankingTime = 0x80;
        };

        struct Decay
        {
            enum class Mode
            {
                forceSlow,
                slowForIncreasingMixedForDecreasing,
                forceFast,
                mixed,
                slowForIncreasingAutoForDecreasing,
                automatic,
            };

            constexpr uint32_t DecayTransitionTimeInNanoSeconds()
            {
                return decayTransitionTime * 500;
            }

            Mode mode = Mode::slowForIncreasingMixedForDecreasing;
            uint8_t decayTransitionTime = 0x10;
        };

        struct Stall
        {
            enum class AssertCounter
            {
                one,
                two,
                four,
                eight,
            };

            enum class BackEMFDivisor
            {
                _32,
                _16,
                _8,
                _4,
            };

            AssertCounter assertCounter = AssertCounter::one;
            BackEMFDivisor backEMFDivisor = BackEMFDivisor::_32;
            uint8_t detectionThreshold = 0x40;
        };

        struct Drive
        {
            enum class OCPThreshold
            {
                _250mV,
                _500mV,
                _750mV,
                _1000mV,
            };

            enum class OCPDeglitchTime
            {
                _1us,
                _2us,
                _4us,
                _8us,
            };

            enum class GateDriveTime
            {
                _250ns,
                _500ns,
                _1000ns,
                _2000ns,
            };

            enum class LowSideGateDrivePeakCurrent
            {
                _100mA,
                _200mA,
                _300mA,
                _400mA,
            };

            enum class HighSideGateDrivePeakCurrent
            {
                _50mA,
                _100mA,
                _150mA,
                _200mA,
            };

            OCPThreshold oCPThreshold = OCPThreshold::_500mV;
            OCPDeglitchTime oCPDeglitchTime = OCPDeglitchTime::_4us;
            GateDriveTime lowSideGateDriveTime = GateDriveTime::_500ns;
            GateDriveTime highSideGateDriveTime = GateDriveTime::_500ns;
            LowSideGateDrivePeakCurrent lowSideGateDrivePeakCurrent = LowSideGateDrivePeakCurrent::_300mA;
            HighSideGateDrivePeakCurrent highSideGateDrivePeakCurrent = HighSideGateDrivePeakCurrent::_150mA;
        };

        struct Status
        {
            bool overTemperatureShutdown = false;
            bool channelAOverCurrentShutdown = false;
            bool channelBOverCurrentShutdown = false;
            bool channelAPreDriveFault = false;
            bool channelBPreDriveFault = false;
            bool underVoltageLockout = false;
            bool stallDetected = false;
            bool latchedStallDetected = false;
        };

        struct Helpers
        {
            constexpr float MaximumCurrentInAmperes(Control::AmplifierGain gain, float torque, float senseResistorValue)
            {
                static const std::array<float, 4> isGain {{ 5.0f, 10.0f, 20.0f, 40.0f }};

                return (2.75f * torque) / (256.0f * senseResistorValue * isGain.at(static_cast<std::size_t>(gain)));
            }

            constexpr bool IsSaturatingMotor(float maximumCurrent, float motorSupplyVoltage, float motorWindingResistance, float mosfetDrainSourceResistance, float senseResistorValue)
            {
                return maximumCurrent < motorSupplyVoltage / (motorWindingResistance + (2 * mosfetDrainSourceResistance) + senseResistorValue);
            }
        };

        Drv8711Sync(hal::SynchronousSpi& spi, hal::GpioPin& chipSelect, hal::GpioPin& reset, hal::GpioPin& sleep);



    private:

    private:
        hal::SynchronousSpi& spi;
        hal::OutputPin chipSelect;
        hal::OutputPin reset;
        hal::OutputPin sleep;
        infra::TimerSingleShot timer;
        infra::Function<void()> onReset;
        infra::Function<void()> onDone;
    };
}

#endif
