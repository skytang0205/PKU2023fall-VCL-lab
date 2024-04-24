#include "Labs/Common/ImageRGB.h"

#include <spdlog/spdlog.h>

namespace VCX::Labs::Common {
    ImageRGB CreatePureImageRGB(std::size_t const width, std::size_t const height, glm::vec3 const & color) {
        ImageRGB image(width, height);
        image.Fill(color);
        return image;
    }

    ImageRGB CreateCheckboardImageRGB(std::size_t const width, std::size_t const height, std::size_t const delta) {
        ImageRGB image(width, height);
        for (std::size_t x = 0; x < width; ++x) {
            for (std::size_t y = 0; y < height; ++y) {
                std::size_t xx = x / delta;
                std::size_t yy = y / delta;
                if ((xx + yy) % 2 == 0)
                    image.At(x, y) = { .8, .8, .8 };
                else
                    image.At(x, y) = { 1., 1., 1. };
            }
        }
        return image;
    }

    ImageRGB AlphaBlend(ImageRGBA const & source, ImageRGB const & dest) {
        if (source.GetSize() != dest.GetSize()) {
            spdlog::error("VCX::Labs::Common::AlphaBlend(..): incompatible size.");
            std::exit(EXIT_FAILURE);
        }
        Common::ImageRGB result(source.GetSize());
        auto width  = source.GetSizeX();
        auto height = source.GetSizeY();
        for (std::size_t x = 0; x < width; ++x)
            for (std::size_t y = 0; y < height; ++y) {
                auto const c = source.At(x, y);
                result.At(x, y) = glm::vec3(c.r, c.g, c.b) * c.a + dest.At(x, y) * (1 - c.a); 
            }
        return result;
    }
}
