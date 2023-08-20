#include "drivers/display/tft/Ssd2119.hpp"
#include "drivers/display/tft/DisplayTft.hpp"

namespace drivers::display::tft
{
    namespace
    {
        constexpr uint8_t deviceCodeRead = 0x00;
        constexpr uint8_t oscStart = 0x00;
        constexpr uint8_t outputControl = 0x01;
        constexpr uint8_t lcdDriveAcControl = 0x02;
        constexpr uint8_t powerControl1 = 0x03;
        constexpr uint8_t displayControl = 0x07;
        constexpr uint8_t frameCycleControl = 0x0b;
        constexpr uint8_t powerControl2 = 0x0c;
        constexpr uint8_t powerControl3 = 0x0d;
        constexpr uint8_t powerControl4 = 0x0e;
        constexpr uint8_t gateScanStart = 0x0f;
        constexpr uint8_t sleepMode1 = 0x10;
        constexpr uint8_t entryModeRegister = 0x11;
        constexpr uint8_t sleepMode2 = 0x12;
        constexpr uint8_t genIfControl = 0x15;
        constexpr uint8_t powerControl5 = 0x1e;
        constexpr uint8_t ramData = 0x22;
        constexpr uint8_t frameFrequency = 0x25;
        constexpr uint8_t analogSetting = 0x26;
        constexpr uint8_t vcomOpt1 = 0x28;
        constexpr uint8_t vcomOpt2 = 0x29;
        constexpr uint8_t gammaControl1 = 0x30;
        constexpr uint8_t gammaControl2 = 0x31;
        constexpr uint8_t gammaControl3 = 0x32;
        constexpr uint8_t gammaControl4 = 0x33;
        constexpr uint8_t gammaControl5 = 0x34;
        constexpr uint8_t gammaControl6 = 0x35;
        constexpr uint8_t gammaControl7 = 0x36;
        constexpr uint8_t gammaControl8 = 0x37;
        constexpr uint8_t gammaControl9 = 0x3a;
        constexpr uint8_t gammaControl10 = 0x3b;
        constexpr uint8_t verticalRamPosition = 0x44;
        constexpr uint8_t horizontalRamStart = 0x45;
        constexpr uint8_t horizontalRamEnd = 0x46;
        constexpr uint8_t xRamAddress = 0x4e;
        constexpr uint8_t yRamAddress = 0x4f;

        constexpr uint16_t displayControlDefault = 0x33;

        enum class WriteRamMethodsFor262kColorMode
        {
            a,
            b,
            c
        };

        enum class WriteRamMode
        {
            spi,
            dataBus,
        };

        enum class NoSync
        {
            onChipFrameStart,
            immediately,
        };

        enum class IncrementMode
        {
            horizontalDecVerticalDec,
            horizontalIncVerticalDec,
            horizontalDecVerticalInc,
            horizontalIncVerticalInc,
        };

        enum class AddressCounterDirection
        {
            horizontal,
            vertical,
        };

        enum class Vcix2
        {
            _5_1_volts,
            _5_3_volts,
            _5_5_volts,
            _5_7_volts,
            _5_9_volts,
            _6_1_volts,
        };

        enum class MultiplyFactorVlcd63
        {
            _2_810,
            _2_900,
            _3_000,
            _1_780,
            _1_850,
            _1_930,
            _2_020,
            _2_090,
            _2_165,
            _2_245,
            _2_335,
            _2_400,
            _2_500,
            _2_570,
            _2_645,
            _2_725,
        };

        constexpr uint16_t SleepMode1(bool enable)
        {
            return enable;
        }

        constexpr uint16_t OscillatorStart(bool enable)
        {
            return enable;
        }

        constexpr uint8_t PowerControl5(bool otp, uint8_t vcm)
        {
            return (vcm & 0x3f) | (otp << 7);
        }

        constexpr uint16_t OutputControl(bool reverse, bool outputGate, bool rgb, bool scanningOrder, bool shiftDirection, bool tb, uint8_t numberOfLines)
        {
            uint16_t content = numberOfLines;

            content |= shiftDirection << 14;
            content |= reverse << 13;
            content |= outputGate << 12;
            content |= rgb << 11;
            content |= scanningOrder << 10;
            content |= tb << 9;

            return content;
        }

        constexpr uint16_t LcdDrivingWaveformControl(bool fld, bool enableWSyncPin, bool bc, bool eor, bool wsmd, uint8_t numberOfLines)
        {
            uint16_t content = 0;

            content |= fld << 12;
            content |= enableWSyncPin << 11;
            content |= bc << 10;
            content |= eor << 9;
            content |= wsmd << 8;
            content |= numberOfLines;

            return content;
        }

        constexpr uint16_t EntryMode(Ssd2119Sync::Config::ColorMode colorMode, Ssd2119Sync::Config::RgbInterface rgbInterface, WriteRamMode wmode, Ssd2119Sync::Config::ClockSource dmode, WriteRamMethodsFor262kColorMode ty, IncrementMode id, AddressCounterDirection am)
        {
            uint16_t content = 0;

            content |= static_cast<uint16_t>(am) << 3;
            content |= static_cast<uint16_t>(id) << 4;
            content |= static_cast<uint16_t>(ty) << 6;
            content |= static_cast<uint16_t>(dmode) << 8;
            content |= static_cast<uint16_t>(wmode) << 9;
            content |= static_cast<uint16_t>(rgbInterface) << 11;
            content |= static_cast<uint16_t>(colorMode) << 13;

            return content;
        }

        constexpr uint16_t PowerControl2(Vcix2 vcix2)
        {
            return static_cast<uint16_t>(vcix2);
        }

        constexpr uint16_t PowerControl3(MultiplyFactorVlcd63 vlcd63)
        {
            return static_cast<uint16_t>(vlcd63);
        }

        constexpr uint16_t PowerControl4(bool enableVComG, uint8_t vdv)
        {
            uint16_t content = 0;

            content |= enableVComG << 13;
            content |= (vdv & 0x1F) << 8;

            return content;
        }

        constexpr uint16_t ToDriverColor(const Color& color)
        {
            auto c = color.Combination();

            return ((((c) & 0x00f80000) >> 8) | (((c) & 0x0000fc00) >> 5) | (((c) & 0x000000f8) >> 3));
        }
    }

    Ssd2119Sync::Ssd2119Sync(hal::SynchronousSpi& spi, hal::GpioPin& chipSelect, hal::GpioPin& reset, hal::GpioPin& dataOrCommand, const Config& config)
        : spi(spi)
        , chipSelect(chipSelect)
        , reset(reset)
        , dataOrCommand(dataOrCommand)
        , config(config)
        , width(config.width)
        , height(config.height)
        , entryMode(EntryMode(config.colorMode, config.rgbInterface, WriteRamMode::spi, config.clockSource, WriteRamMethodsFor262kColorMode::a, IncrementMode::horizontalIncVerticalInc, AddressCounterDirection::horizontal))
    {
        Reset([this]()
            {
                Initialize(this->config);
            });
    }

    void Ssd2119Sync::DrawPixel(std::size_t x, std::size_t y, Color color)
    {
        PrepareToDraw(x, y);
        WriteData(ToDriverColor(color));
    }

    void Ssd2119Sync::DrawHorizontalLine(std::size_t xStart, std::size_t xEnd, std::size_t y, Color color)
    {
        WriteCommand(entryModeRegister);
        WriteData(entryMode | direction.at(static_cast<uint8_t>(config.orientation)).horizontal);

        PrepareToDraw(xStart, y);

        auto c = ToDriverColor(color);

        while(xStart++ <= xEnd)
        {
            WriteData(c);
        }
    }

    void Ssd2119Sync::DrawVerticalLine(std::size_t x, std::size_t yStart, std::size_t yEnd, Color color)
    {
        WriteCommand(entryModeRegister);
        WriteData(entryMode | direction.at(static_cast<uint8_t>(config.orientation)).vertical);

        PrepareToDraw(x, yStart);

        auto c = ToDriverColor(color);

        while(yStart++ <= yEnd)
        {
            WriteData(c);
        }
    }

    void Ssd2119Sync::Reset(const infra::Function<void()>& onReset)
    {
        this->onReset = onReset;

        reset.Set(false);
        timer.Start(std::chrono::milliseconds(10), [this]()
            {
                reset.Set(true);
                timer.Start(std::chrono::milliseconds(20), [this]()
                {
                    this->onReset();
                });
            });
        
    }

    void Ssd2119Sync::Initialize(const Config& config)
    {
        WriteCommand(sleepMode1);
        //WriteData(SleepMode1(true));
        WriteData(0x0001);

        WriteCommand(powerControl5);
        //WriteData(PowerControl5(true, 0x3A));
        WriteData(0x00BA);
        WriteCommand(vcomOpt1);
        //WriteData(0x6);
        WriteData(0x0006);

        WriteCommand(oscStart);
        //WriteData(OscillatorStart(true));
        WriteData(0x0001);

        WriteCommand(outputControl);
        //WriteData(OutputControl(true, true, false, false, false, false, config.height - 1));
        WriteData(0x30EF);
        WriteCommand(lcdDriveAcControl);
        //WriteData(LcdDrivingWaveformControl(false, false, true, true, false, 0));
        WriteData(0x0600);

        WriteCommand(sleepMode1);
        //WriteData(SleepMode1(false));
        WriteData(0x0000);

        timer.Start(std::chrono::milliseconds(30), [this]()
            {
                WriteCommand(entryModeRegister);
                //WriteData(entryMode);
                WriteData(0x6830);

                WriteCommand(displayControl);
                //WriteData(displayControlDefault);
                WriteData(0x0033);

                WriteCommand(powerControl2);
                //WriteData(PowerControl2(Vcix2::_6_1_volts));
                WriteData(0x0005);

                SetGamma();

                WriteCommand(powerControl3);
                //WriteData(PowerControl3(MultiplyFactorVlcd63::_2_090));
                WriteData(0x0007);
                WriteCommand(powerControl4);
                //WriteData(PowerControl4(true, 0x10));
                WriteData(0x3100);

                SetDimension(this->config.width, this->config.height);
                PrepareToDraw(0, 0);

                auto total = this->config.width * this->config.height;

                while (total-- > 0)
                    WriteData(0xf0f);
            });
    }
    
    void Ssd2119Sync::SetGamma()
    {
        WriteCommand(gammaControl1);
        WriteData(0x0000);
        WriteCommand(gammaControl2);
        WriteData(0x0400);
        WriteCommand(gammaControl3);
        WriteData(0x0106);
        WriteCommand(gammaControl4);
        WriteData(0x0700);
        WriteCommand(gammaControl5);
        WriteData(0x0002);
        WriteCommand(gammaControl6);
        WriteData(0x0702);
        WriteCommand(gammaControl7);
        WriteData(0x0707);
        WriteCommand(gammaControl8);
        WriteData(0x0203);
        WriteCommand(gammaControl9);
        WriteData(0x1400);
        WriteCommand(gammaControl10);
        WriteData(0x0F03);
    }

    void Ssd2119Sync::SetDimension(std::size_t horizontal, std::size_t vertical)
    {
        WriteCommand(verticalRamPosition);
        WriteData((vertical - 1) << 8);

        WriteCommand(horizontalRamStart);
        WriteData(0);

        WriteCommand(horizontalRamEnd);
        WriteData(horizontal - 1);
    }

    void Ssd2119Sync::SetPosition(std::size_t x, std::size_t y)
    {
        WriteCommand(xRamAddress);
        WriteData(x);

        WriteCommand(yRamAddress);
        WriteData(y);
    }

    uint16_t Ssd2119Sync::GetX(std::size_t x, std::size_t y)
    {
        switch (config.orientation)
        {
            case Ssd2119Sync::Config::Orientation::portrait:
                return width - y - 1;

            case Ssd2119Sync::Config::Orientation::landscape:
                return width - x - 1;

            case Ssd2119Sync::Config::Orientation::portraitFlip:
                return y;

            default:
                return x;
        }
    }

    uint16_t Ssd2119Sync::GetY(std::size_t x, std::size_t y)
    {
        switch (config.orientation)
        {
            case Ssd2119Sync::Config::Orientation::portrait:
                return x;

            case Ssd2119Sync::Config::Orientation::landscape:
                return height - y - 1;

            case Ssd2119Sync::Config::Orientation::portraitFlip:
                return height - x - 1;

            default:
                return y;
        }
    }

    void Ssd2119Sync::PrepareToDraw(std::size_t x, std::size_t y)
    {
        // /SetPosition(GetX(x, y), GetY(x, y));
        SetPosition(0 ,0);
        WriteCommand(ramData);
    }

    void Ssd2119Sync::WriteData(uint16_t data)
    {
        std::array<uint8_t, 2> payload {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xff)};

        dataOrCommand.Set(true);
        chipSelect.Set(false);

        spi.SendData(payload, hal::SynchronousSpi::Action::stop);

        chipSelect.Set(true);
    }

    void Ssd2119Sync::WriteCommand(uint8_t reg)
    {
        std::array<uint8_t, 2> payload {0, reg};

        dataOrCommand.Set(false);
        chipSelect.Set(false);

        spi.SendData(payload, hal::SynchronousSpi::Action::stop);

        chipSelect.Set(true);
    }
}