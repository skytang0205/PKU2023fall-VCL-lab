#pragma once

#include "Engine/Async.hpp"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Drawing2D {

    class CaseFiltering : public Common::ICase {
    public:
        enum struct FilterType {
            Original = 0,
            Blur,
            Edge
        };

        CaseFiltering();

        virtual std::string_view const GetName() override { return "Image Filtering"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::GL::UniqueTexture2D _texture;

        Common::ImageRGB _input;
        Common::ImageRGB _empty;

        Engine::Async<Common::ImageRGB> _task;

        bool _enableZoom = true;
        bool _recompute  = true;

        FilterType _filterType = FilterType::Original;
    };
}
