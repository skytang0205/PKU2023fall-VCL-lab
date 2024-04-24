#pragma once

#include "Engine/TextureND.hpp"

namespace VCX::Labs::Common {
    using ImageRGB  = Engine::Texture2D<Engine::Formats::RGB8>;
    using ImageRGBA = Engine::Texture2D<Engine::Formats::RGBA8>;

	ImageRGB CreatePureImageRGB(std::size_t const width, std::size_t const height, glm::vec3 const & color);
	ImageRGB CreateCheckboardImageRGB(std::size_t const width, std::size_t const height, std::size_t const delta = 32);

	ImageRGB AlphaBlend(ImageRGBA const & source, ImageRGB const & dest);
}
