#pragma once

#include "Engine/Async.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Drawing2D {

    class CaseDithering : public Common::ICase {
    public:
        enum struct AlgorithmType {
            Original = 0,
            Threshold,
            Random,
            BlueNoise,
            Ordered,
            ErrorDiffuse
        };

        CaseDithering();

        virtual std::string_view const GetName() override { return "Image Dithering"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueTexture2D _texture;
        Engine::GL::UniqueTexture2D _textureLarge;

        Common::ImageRGB _input;
        Common::ImageRGB _empty;
        Common::ImageRGB _emptyLarge;
        Common::ImageRGB _blueNoise;

        Engine::Async<Common::ImageRGB> _task;

        bool _enableZoom = true;
        bool _recompute  = true;
        bool _isLarge    = false;

        AlgorithmType _algType;
    };
} // namespace VCX::Labs::Drawing2D
