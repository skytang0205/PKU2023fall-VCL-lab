#pragma once

#include "Engine/Async.hpp"
#include "Labs/2-GeometryProcessing/Content.h"
#include "Labs/2-GeometryProcessing/Viewer.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::GeometryProcessing {

    class CaseSimplification : public Common::ICase {
    public:
        CaseSimplification(Viewer & viewer, std::initializer_list<Assets::ExampleModel> && models);

        virtual std::string_view const GetName() override { return "Mesh Simplification"; }
        
        virtual void OnSetupPropsUI() override;
        virtual Common::CaseRenderResult OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void OnProcessInput(ImVec2 const & pos) override;
    
    private:
        std::vector<Assets::ExampleModel> const _models;

        Engine::Async<Engine::SurfaceMesh>    _task;
        Viewer                             &  _viewer;
        Engine::Camera                        _camera              { .Eye = glm::vec3(-1, 1, 1) };
        Common::OrbitCameraManager            _cameraManager       { glm::vec3(-1, 1, 1) };
        std::size_t                           _modelIdx            { 0 };
        bool                                  _recompute           { true };
        bool                                  _running             { false };
        ModelObject                           _modelObject;
        RenderOptions                         _options;
        int                                   _simplificationLevel { 2 };

        char const *                GetModelName(std::size_t const i) const { return Content::ModelNames[std::size_t(_models[i])].c_str(); }
        Engine::SurfaceMesh const & GetModelMesh(std::size_t const i) const { return Content::ModelMeshes[std::size_t(_models[i])]; }
    };
}
