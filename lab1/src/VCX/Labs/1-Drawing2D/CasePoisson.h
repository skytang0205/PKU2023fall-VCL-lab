#pragma once

#include "Engine/Async.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Drawing2D {

    class CasePoisson : public Common::ICase {
    public:
        CasePoisson();

        virtual std::string_view const GetName() override { return "Image Inpainting"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueTexture2D _texture;

        Common::ImageRGB _inputBack;
        Common::ImageRGB _inputFront;
        Common::ImageRGB _empty;

        Engine::Async<Common::ImageRGB> _task;

        bool _enableZoom    = true;
        bool _recompute     = true;
        bool _enablePoisson = false;
    };
}
