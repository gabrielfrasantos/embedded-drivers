#ifndef DRIVERS_DISPLAY_TFT_SSD2119_H
#define DRIVERS_DISPLAY_TFT_SSD2119_H

#include "drivers/display/tft/DisplayTft.hpp"
#include "hal/synchronous_interfaces/SynchronousSpi.hpp"
#include "hal/interfaces/Gpio.hpp"
#include "infra/timer/Timer.hpp"

namespace drivers::display::tft
{
    class Ssd2119Sync
        : public DisplayTft
    {
    public:
        struct Config
        {
            constexpr Config()
            {}

            enum class Orientation : uint8_t
            {
                portrait,
                landscape,
                portraitFlip,
                landscapeFlip,
            };

            enum class ColorMode : uint8_t
            {
                _262k = 2,
                _65k = 3,
            };

            enum class ClockSource
            {
                internalOscillator,
                externalOscillatorViaDotClock,
            };

            enum class RgbInterface
            {
                ignoreControlPins,
                useControlPIns
            };

            std::size_t width = 320;
            std::size_t height = 240;
            Orientation orientation = Orientation::landscape;
            ColorMode colorMode = ColorMode::_65k;
            ClockSource clockSource = ClockSource::internalOscillator;
            RgbInterface rgbInterface = RgbInterface::ignoreControlPins;
        };

        Ssd2119Sync(hal::SynchronousSpi& spi, hal::GpioPin& chipSelect, hal::GpioPin& reset, hal::GpioPin& dataOrCommand, const infra::Function<void()>& onDone, const Config& config = Config());

        void DrawPixel(std::size_t x, std::size_t y, Color color, const infra::Function<void()>& onDone) override;
        void DrawHorizontalLine(std::size_t xStart, std::size_t xEnd, std::size_t y, Color color, const infra::Function<void()>& onDone) override;
        void DrawVerticalLine(std::size_t x, std::size_t yStart, std::size_t yEnd, Color color, const infra::Function<void()>& onDone) override;
        void DrawRectangle(std::size_t xStart, std::size_t xEnd, std::size_t yStart, std::size_t yEnd, Color color, const infra::Function<void()>& onDone) override;
        void DrawBackground(Color color, const infra::Function<void()>& onDone) override;

    private:
        void WriteData(uint16_t data);
        void WriteCommand(uint8_t reg);
        void Initialize(const Config& config);
        void Reset(const infra::Function<void()>& onReset);
        void SetGamma();
        void SetDimension(std::size_t width, std::size_t height);
        void SetPosition(std::size_t x, std::size_t y);

        void PrepareToDraw(std::size_t x, std::size_t y);

        uint16_t GetX(std::size_t x, std::size_t y);
        uint16_t GetY(std::size_t x, std::size_t y);

    private:
        struct Direction
        {
            uint8_t horizontal;
            uint8_t vertical;
        };

        hal::SynchronousSpi& spi;
        hal::OutputPin chipSelect;
        hal::OutputPin reset;
        hal::OutputPin dataOrCommand;
        const Config& config;
        std::size_t width;
        std::size_t height;
        uint16_t entryMode;
        infra::TimerSingleShot timer;
        infra::Function<void()> onReset;
        infra::Function<void()> onDone;

        constexpr static std::array<Direction, 4> direction {{ {0x28, 0x20}, {0x00, 0x08}, {0x18, 0x10}, {0x30, 0x38} }};
    };
}

#endif
