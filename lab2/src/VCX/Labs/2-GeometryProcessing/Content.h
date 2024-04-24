#pragma once

#include "Assets/bundled.h"
#include "Engine/SurfaceMesh.h"

namespace VCX::Labs::GeometryProcessing {
    class Content {
    public:
        static std::array<Engine::SurfaceMesh, Assets::ExampleModels.size()> const ModelMeshes;
        static std::array<std::string,         Assets::ExampleModels.size()> const ModelNames;
    };
}
