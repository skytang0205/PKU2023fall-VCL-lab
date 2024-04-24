#pragma once
#include <cstdint>
#include <string_view>

#include <imgui.h>

#include "Engine/GL/resource.hpp"
#include "Engine/GL/Texture.hpp"

namespace VCX::Labs::Common::ImGuiHelper {
    void TextCentered(std::string_view const text);
    void TextRight   (std::string_view const text);

    void ZoomTooltip(
        Engine::GL::UniqueTexture2D const &     tex,
        std::pair<std::uint32_t, std::uint32_t> texSize,
        ImVec2 const &                          pos,
        bool const                              filpped    = false,
        float const                             regionSize = 40.f,
        float const                             zoomLevel  = 4.f);

    void SaveImage(
        Engine::GL::UniqueTexture2D const &     tex,
        std::pair<std::uint32_t, std::uint32_t> texSize,
        bool const                              flipped = false);
}
