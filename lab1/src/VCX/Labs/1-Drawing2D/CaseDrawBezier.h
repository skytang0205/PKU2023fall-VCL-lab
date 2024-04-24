#pragma once

#include <vector>

#include "Engine/Async.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Drawing2D {

    class CaseDrawBezier : public Common::ICase {
    public:
        CaseDrawBezier();

        virtual std::string_view const GetName() override { return "Bezier Curve"; }

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

        int                     _selectIdx = -1;
        std::vector<glm::fvec2> _handles   = {
              { 20, 160},
              {130, 250},
              {210,  90},
              {290,  10}
        };
    };
}
