#include "drivers/stepper_motor/Drv8711.hpp"

namespace drivers::stepper_motor
{
    namespace
    {
        constexpr uint8_t readOperation = 1;
        constexpr uint8_t writeOperation = 0;

        constexpr uint8_t enableOffset = 0;
        constexpr uint8_t rdirOffset = 1;
        constexpr uint8_t rstepOffset = 2;
        constexpr uint8_t modeOffset = 4;
        constexpr uint8_t exStallOffset = 7;
        constexpr uint8_t isGainOffset = 8;
        constexpr uint8_t dtimeOffset = 10;

        constexpr uint16_t isGainMask = 0x300;
        constexpr uint16_t torqueMask = 0x0ff;
        constexpr uint16_t offTimeMask = 0x0ff;
        constexpr uint16_t offPwmModeMask = 0x0ff;
        constexpr uint16_t pwmModeMask = 0x100;
        constexpr uint16_t backEmfSampleThresholdMask = 0x700;
        constexpr uint16_t exStallMask = 0x80;
        constexpr uint16_t deadTimeMask = 0xC00;
        constexpr uint16_t modeMask = 0x78;
        constexpr uint16_t enableMask = 0x1;
        constexpr uint16_t rstepMask = 0x4;

        constexpr uint8_t smplthOffset = 8;
        constexpr uint8_t pwmModeOffset = 8;
        constexpr uint8_t abtOffset = 8;
        constexpr uint8_t decModeOffset = 8;

        constexpr uint8_t sdcntOffset = 8;
        constexpr uint8_t vdivOffset = 10;

        constexpr uint8_t ocpthOffset = 0;
        constexpr uint8_t ocpdegOffset = 2;
        constexpr uint8_t tdrivenOffset = 4;
        constexpr uint8_t tdrivepOffset = 6;
        constexpr uint8_t idrivenOffset = 8;
        constexpr uint8_t idrivepOffset = 10;

        constexpr uint8_t otsOffset = 0;
        constexpr uint8_t aocpOffset = 1;
        constexpr uint8_t bocpOffset = 2;
        constexpr uint8_t apdfOffset = 3;
        constexpr uint8_t bpdfOffset = 4;
        constexpr uint8_t uvloOffset = 5;
        constexpr uint8_t stdOffset = 6;
        constexpr uint8_t stdlayOffset = 7;
    }

    Drv8711Sync::Drv8711Sync(hal::SynchronousSpi& spi, hal::GpioPin& chipSelect, hal::GpioPin& reset, hal::GpioPin& sleep, const infra::Function<void()>& onDone)
        : spi(spi)
        , chipSelect(chipSelect)
        , reset(reset)
        , sleep(sleep, true)
        , onDone(onDone)
    {
        reset.Set(true);
        timer.Start(std::chrono::milliseconds(1), [this]()
            {
                this->reset.Set(false);
                this->timer.Start(std::chrono::milliseconds(1), [this]()
                    {
                        this->onDone();
                    });
            });
    }

    void Drv8711Sync::Reset(const infra::Function<void()>& onDone)
    {
        this->onDone = onDone;
        reset.Set(true);
        timer.Start(std::chrono::milliseconds(1), [this]()
            {
                this->reset.Set(false);
                this->timer.Start(std::chrono::milliseconds(1), [this]()
                    {
                        this->onDone();
                    });
            });
    }

    void Drv8711Sync::EnableSleep(const infra::Function<void()>& onDone)
    {
        this->onDone = onDone;
        sleep.Set(false);
        timer.Start(std::chrono::milliseconds(1), [this]()
            {
                this->onDone();
            });
    }

    void Drv8711Sync::DisableSleep(const infra::Function<void()>& onDone)
    {
        this->onDone = onDone;
        sleep.Set(true);
        timer.Start(std::chrono::milliseconds(1), [this]()
            {
                this->onDone();
            });
    }

    void Drv8711Sync::EnableMotor(const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Control::address);

        Write(Torque::address, (data & ~modeMask) | enableMask);

        onDone();
    }

    void Drv8711Sync::DisableMotor(const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Control::address);

        Write(Torque::address, (data & ~modeMask));

        onDone();
    }

    void Drv8711Sync::GoOneStep(const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Control::address);

        data = (data & ~rstepMask) | rstepOffset;

        Write(Torque::address, data);

        onDone();
    }

    void Drv8711Sync::SetMicroSteps(Control::Step microSteps, const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Control::address);

        Write(Torque::address, (data & ~modeMask) | static_cast<uint16_t>(microSteps) << modeOffset);

        onDone();
    }

    void Drv8711Sync::SetDeadTime(Control::DeadTime deadTime, const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Control::address);

        Write(Torque::address, (data & ~deadTimeMask) | static_cast<uint16_t>(deadTime) << exStallOffset);

        onDone();
    }

    void Drv8711Sync::SetStallDetection(Control::StallDetection stallDetection, const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Control::address);

        Write(Torque::address, (data & ~exStallMask) | static_cast<uint16_t>(stallDetection) << exStallOffset);

        onDone();
    }

    void Drv8711Sync::SetBackEMFSampleThreshold(Torque::BackEMFSampleThreshold backEmfSampleThreshold, const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Torque::address);

        Write(Torque::address, (data & ~backEmfSampleThresholdMask) | static_cast<uint16_t>(backEmfSampleThreshold) << smplthOffset);

        onDone();
    }

    void Drv8711Sync::SetIndexer(Off::PwmMode indexer, const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Off::address);

        Write(Torque::address, (data & ~pwmModeMask) | static_cast<uint16_t>(indexer) << pwmModeOffset);

        onDone();
    }

    void Drv8711Sync::SetOffTime(uint8_t offtime, const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Off::address);

        Write(Torque::address, (data & ~offTimeMask) | offtime);

        onDone();
    }

    void Drv8711Sync::SetAmplifierGainAndTorque(Control::AmplifierGain gain, uint8_t torque, const infra::Function<void()>& onDone)
    {
        uint16_t data = Read(Control::address);

        Write(Control::address, (data & ~isGainMask) | static_cast<uint16_t>(gain) << isGainOffset);

        data = Read(Torque::address);

        Write(Torque::address, (data & ~torqueMask) | torque);

        onDone();
    }

    void Drv8711Sync::ConfigureOffTime(const Off& off, const infra::Function<void()>& onDone)
    {
        uint16_t data = 0;

        data |= static_cast<uint16_t>(off.fixedOffTime);
        data |= static_cast<uint16_t>(off.pwmMode) << pwmModeOffset;

        Write(Off::address, data);

        onDone();
    }

    void Drv8711Sync::ConfigureBlankingTime(const Blank& blank, const infra::Function<void()>& onDone)
    {
        uint16_t data = 0;

        data |= static_cast<uint16_t>(blank.blankingTime);
        data |= static_cast<uint16_t>(blank.adaptativeBlankingTime) << abtOffset;

        Write(Blank::address, data);

        onDone();
    }

    void Drv8711Sync::ConfigureDecay(const Decay& decay, const infra::Function<void()>& onDone)
    {
        uint16_t data = 0;

        data |= static_cast<uint16_t>(decay.decayTransitionTime);
        data |= static_cast<uint16_t>(decay.mode) << decModeOffset;

        Write(Decay::address, data);

        onDone();
    }

    void Drv8711Sync::ConfigureStall(const Stall& stall, const infra::Function<void()>& onDone)
    {
        uint16_t data = 0;

        data |= static_cast<uint16_t>(stall.detectionThreshold);
        data |= static_cast<uint16_t>(stall.assertCounter) << sdcntOffset;
        data |= static_cast<uint16_t>(stall.backEMFDivisor) << vdivOffset;

        Write(Stall::address, data);

        onDone();
    }

    void Drv8711Sync::ConfigureDrive(const Drive& drive, const infra::Function<void()>& onDone)
    {
        uint16_t data = 0;

        data |= static_cast<uint16_t>(drive.oCPThreshold) << ocpthOffset;
        data |= static_cast<uint16_t>(drive.oCPDeglitchTime) << ocpdegOffset;
        data |= static_cast<uint16_t>(drive.lowSideGateDriveTime) << tdrivenOffset;
        data |= static_cast<uint16_t>(drive.highSideGateDriveTime) << tdrivepOffset;
        data |= static_cast<uint16_t>(drive.lowSideGateDrivePeakCurrent) << idrivenOffset;
        data |= static_cast<uint16_t>(drive.highSideGateDrivePeakCurrent) << idrivepOffset;

        Write(Drive::address, data);

        onDone();
    }

    void Drv8711Sync::GetControlRegister(const infra::Function<void(const Drv8711Sync::Control&)>& onDone)
    {
        auto data = Read(Control::address);

        control.enableMotor = data >> enableOffset;
        control.direction = static_cast<Control::Direction>(data >> rdirOffset);
        control.advanceOneStep = data >> rstepOffset;
        control.step = static_cast<Control::Step>(data >> modeOffset);
        control.stallDetection = static_cast<Control::StallDetection>(data >> exStallOffset);
        control.amplifierGain = static_cast<Control::AmplifierGain>(data >> isGainOffset);
        control.deadTime = static_cast<Control::DeadTime>(data >> dtimeOffset);

        onDone(control);
    }

    void Drv8711Sync::GetTorqueRegister(const infra::Function<void(const Drv8711Sync::Torque&)>& onDone)
    {
        auto data = Read(Torque::address);

        torque.fullScaleCurrent = data & 0xff;
        torque.backEMFSampleThreshold = static_cast<Torque::BackEMFSampleThreshold>(data >> smplthOffset);

        onDone(torque);
    }

    void Drv8711Sync::GetOffTimeRegister(const infra::Function<void(const Drv8711Sync::Off&)>& onDone)
    {
        auto data = Read(Blank::address);

        off.fixedOffTime = data & 0xff;
        off.pwmMode = static_cast<Off::PwmMode>(data >> pwmModeOffset);

        onDone(off);
    }

    void Drv8711Sync::GetBlankRegister(const infra::Function<void(const Drv8711Sync::Blank&)>& onDone)
    {
        auto data = Read(Blank::address);

        blank.blankingTime = data & 0xff;
        blank.adaptativeBlankingTime = data >> abtOffset;

        onDone(blank);
    }

    void Drv8711Sync::GetDecayRegister(const infra::Function<void(const Drv8711Sync::Decay&)>& onDone)
    {
        auto data = Read(Decay::address);

        decay.decayTransitionTime = data & 0xff;
        decay.mode = static_cast<Decay::Mode>(data >> decModeOffset);

        onDone(decay);
    }

    void Drv8711Sync::GetStallRegister(const infra::Function<void(const Drv8711Sync::Stall&)>& onDone)
    {
        auto data = Read(Stall::address);

        stall.detectionThreshold = data & 0xff;
        stall.assertCounter = static_cast<Stall::AssertCounter>(data >> sdcntOffset);
        stall.backEMFDivisor = static_cast<Stall::BackEMFDivisor>(data >> vdivOffset);

        onDone(stall);
    }

    void Drv8711Sync::GetDriveRegister(const infra::Function<void(const Drv8711Sync::Drive&)>& onDone)
    {
        auto data = Read(Drive::address);

        drive.oCPThreshold = static_cast<Drive::OCPThreshold>(data >> ocpthOffset);
        drive.oCPDeglitchTime = static_cast<Drive::OCPDeglitchTime>(data >> ocpdegOffset);
        drive.lowSideGateDriveTime = static_cast<Drive::GateDriveTime>(data >> tdrivenOffset);
        drive.highSideGateDriveTime = static_cast<Drive::GateDriveTime>(data >> tdrivepOffset);
        drive.lowSideGateDrivePeakCurrent = static_cast<Drive::LowSideGateDrivePeakCurrent>(data >> idrivenOffset);
        drive.highSideGateDrivePeakCurrent = static_cast<Drive::HighSideGateDrivePeakCurrent>(data >> idrivepOffset);

        onDone(drive);
    }

    void Drv8711Sync::GetStatusRegister(const infra::Function<void(const Drv8711Sync::Status&)>& onDone)
    {
        auto data = Read(Status::address);

        std::copy_n(reinterpret_cast<uint8_t*>(&data), 1, reinterpret_cast<uint8_t*>(&status));

        onDone(status);
    }

    uint16_t Drv8711Sync::Read(uint8_t address)
    {
        uint16_t output;
        uint16_t input = address << 12 | readOperation << 15;

        chipSelect.Set(true);

        spi.SendAndReceive(infra::MakeConstByteRange(input), infra::MakeByteRange(output), hal::SynchronousSpi::Action::stop);

        chipSelect.Set(false);

        return output & 0x3ff;
    }

    void Drv8711Sync::Write(uint8_t address, uint16_t data)
    {
        data |= address << 12 | writeOperation << 15;

        chipSelect.Set(true);

        spi.SendData(infra::MakeConstByteRange(data), hal::SynchronousSpi::Action::stop);

        chipSelect.Set(false);

        really_assert(data == Read(Drive::address));
    }
}