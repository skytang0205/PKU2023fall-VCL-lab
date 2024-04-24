#pragma once

#include <vector>

#include "Assets/bundled.h"
#include "Engine/app.h"
#include "Engine/SurfaceMesh.h"
#include "Labs/2-GeometryProcessing/CaseMarchingCubes.h"
#include "Labs/2-GeometryProcessing/CaseParameterization.h"
#include "Labs/2-GeometryProcessing/CaseSimplification.h"
#include "Labs/2-GeometryProcessing/CaseSmoothing.h"
#include "Labs/2-GeometryProcessing/CaseSubdivision.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::GeometryProcessing {
    class App : public Engine::IApp {
    private:
        Common::UI           _ui;
        Viewer               _viewer;
        CaseSubdivision      _caseSubdivision;
        CaseParameterization _caseParameterization; 
        CaseSimplification   _caseSimplification;
        CaseSmoothing        _caseSmoothing;
        CaseMarchingCubes    _caseMarchingCubes;

        std::size_t _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseSubdivision,
            _caseParameterization,
            _caseSimplification,
            _caseSmoothing,
            _caseMarchingCubes,
        };

    public:
        App();

        void OnFrame() override;
    };
}
