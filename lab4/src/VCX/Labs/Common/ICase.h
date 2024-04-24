#pragma once

#include <functional>
#include <string_view>
#include <optional>

#include <imgui_internal.h>

#include "Engine/GL/resource.hpp"
#include "Engine/GL/Texture.hpp"

namespace VCX::Labs::Common {

    struct CaseRenderResult {
        bool Fixed;
        bool Flipped = false;
        Engine::GL::UniqueTexture2D const & Image;
        std::pair<std::uint32_t, std::uint32_t> ImageSize;
    };

    class ICase {
    public:
        virtual std::string_view const GetName() = 0;
        
        virtual void OnSetupPropsUI() {}
        virtual CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) = 0;
        virtual void OnProcessInput(ImVec2 const & pos) {}
    };
}
