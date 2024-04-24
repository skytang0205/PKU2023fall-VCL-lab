#pragma once

#include <glm/glm.hpp>

#include "Assets/bundled.h"
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/UniformBlock.hpp"
#include "Labs/2-GeometryProcessing/ModelObject.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::GeometryProcessing {
    struct PassConstants {
        alignas(64) glm::mat4  NormalTransform;
        alignas(64) glm::mat4  Model;
        alignas(64) glm::mat4  View;
        alignas(64) glm::mat4  Projection;
        alignas(16) glm::vec3  LightDirection;
        alignas(16) glm::vec3  LightColor;
        alignas(16) glm::vec3  ObjectColor;
        alignas(4)  float      Ambient;
        alignas(4)  int        HasTexCoord;
        alignas(4)  int        Wireframe;
        alignas(4)  int        Flat;
    };

    struct RenderOptions {
        glm::vec3 LightDirection = glm::vec3(1.f, -1.f, 1.f);
        glm::vec3 LightColor     = glm::vec3(1.f, 1.f, 1.f);
		glm::vec3 ObjectColor    = glm::vec3(0.8f, 0.5f, 0.f);
        float     LightDirScalar = 210.f;
        float     Ambient        = 0.1f;
        bool      Wireframe      = false;
        bool      Flat           = true;
    };

    class Viewer {
    public:
        Viewer();

        auto GetSize() const { return _frame.GetSize(); }
        auto const & GetTexture() { return _frame.GetColorAttachment(); }

        Common::CaseRenderResult Render(RenderOptions const & options, ModelObject & modelObject, Engine::Camera &camera, Engine::ICameraManager & cameraManager, std::pair<std::uint32_t, std::uint32_t> desiredSize);

        static void SetupRenderOptionsUI(RenderOptions & options, Common::OrbitCameraManager & cameraManager);

    private:
        Engine::GL::UniqueProgram                      _program;
        Engine::GL::UniqueUniformBlock<PassConstants>  _uniformBlock;
        Engine::GL::UniqueRenderFrame                  _frame;
    };
}
