#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/1-Drawing2D/CaseDithering.h"
#include "Labs/1-Drawing2D/CaseDrawBezier.h"
#include "Labs/1-Drawing2D/CaseDrawFilled.h"
#include "Labs/1-Drawing2D/CaseDrawLine.h"
#include "Labs/1-Drawing2D/CaseFiltering.h"
#include "Labs/1-Drawing2D/CasePoisson.h"
#include "Labs/1-Drawing2D/CaseSSAA.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Drawing2D {
    class App : public VCX::Engine::IApp {
    private:
        Common::UI _ui;

        CaseDithering  _caseDithering;
        CaseFiltering  _caseFiltering;
        CasePoisson    _casePoisson;
        CaseDrawLine   _caseDrawLine;
        CaseDrawFilled _caseDrawFilled;
        CaseDrawBezier _caseDrawBezier;
        CaseSSAA       _caseSSAA;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseDithering,
            _caseFiltering,
            _casePoisson,
            _caseDrawLine,
            _caseDrawFilled,
            _caseSSAA,
            _caseDrawBezier,
        };

    public:
        App();

        void OnFrame() override;
    };
}