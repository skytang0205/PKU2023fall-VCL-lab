#pragma once

#include "Engine/Async.hpp"
#include "Labs/2-GeometryProcessing/Viewer.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::GeometryProcessing {

    class CaseParameterization : public Common::ICase {
    public:
        CaseParameterization(Viewer & viewer);

        virtual std::string_view const GetName() override { return "Mesh Parameterization"; }

        virtual void                     OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                     OnProcessInput(ImVec2 const & pos) override;

    private:
        Engine::Async<Engine::SurfaceMesh> _task;
        Viewer &                           _viewer;
        Engine::Camera                     _camera;
        Common::OrbitCameraManager         _cameraManager;
        bool                               _recompute     = true;
        bool                               _running       = false;
        ModelObject                        _modelObject;
        RenderOptions                      _options;
        int                                _numIterations = 50;
    };
} // namespace VCX::Labs::GeometryProcessing
