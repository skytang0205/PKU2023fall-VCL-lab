#pragma once

#include "Engine/Async.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Drawing2D {

    class CaseDrawFilled : public Common::ICase {
    public:
        CaseDrawFilled();

        virtual std::string_view const GetName() override { return "Triangle Drawing"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueTexture2D _texture;

        Common::ImageRGB _empty;

        Engine::Async<Common::ImageRGB> _task;

        bool _enableZoom     = true;
        bool _enableLeftDrag = false;
        bool _recompute      = true;

        int                       _selectIdx = -1;
        std::array<glm::ivec2, 3> _vertices {
            glm::ivec2 { 30,  50},
            glm::ivec2 {160, 300},
            glm::ivec2 {310,  40}
        };
    };
} // namespace VCX::Labs::Drawing2D
