#ifndef DRIVERS_DISPLAY_TFT_INTERFACE_H
#define DRIVERS_DISPLAY_TFT_INTERFACE_H

#include <cstdint>
#include "infra/util/AutoResetFunction.hpp"

namespace drivers::display::tft
{
    class Color
    {
    public:
        Color(uint8_t red, uint8_t green, uint8_t blue)
            : color(red << 16 | green << 8 | blue)
        {}

        Color(uint32_t color)
            : color(color)
        {}

        uint8_t Red() const
        {
            return (color >> 16) & 0xff; 
        }

        uint8_t Green() const
        {
            return (color >> 8) & 0xff; 
        }

        uint8_t Blue() const
        {
            return color & 0xff; 
        }

        constexpr uint32_t Combination() const
        {
            return color;
        }

    private:
        uint32_t color;
    };

    class DisplayTft
    {
    public:
        virtual ~DisplayTft() = default;

        virtual void DrawPixel(std::size_t x, std::size_t y, Color color, const infra::Function<void()>& onDone) = 0;
        virtual void DrawHorizontalLine(std::size_t xStart, std::size_t xEnd, std::size_t y, Color color, const infra::Function<void()>& onDone) = 0;
        virtual void DrawVerticalLine(std::size_t x, std::size_t yStart, std::size_t yEnd, Color color, const infra::Function<void()>& onDone) = 0;
        virtual void DrawRectangle(std::size_t xStart, std::size_t xEnd, std::size_t yStart, std::size_t yEnd, Color color, const infra::Function<void()>& onDone) = 0;
        virtual void DrawBackground(Color color, const infra::Function<void()>& onDone) = 0;
    };
}

#endif
