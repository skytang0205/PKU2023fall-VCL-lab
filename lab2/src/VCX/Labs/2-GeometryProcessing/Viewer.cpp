#include <filesystem>

#include <spdlog/spdlog.h>

#include "Engine/loader.h"
#include "Labs/2-GeometryProcessing/Viewer.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::GeometryProcessing {
    Viewer::Viewer():
        _program(
            Engine::GL::UniqueProgram({
                Engine::GL::SharedShader("assets/shaders/three.vert"),
                Engine::GL::SharedShader("assets/shaders/three.geom"),
                Engine::GL::SharedShader("assets/shaders/three.frag") })),
        _uniformBlock(0, Engine::GL::DrawFrequency::Stream) {
        _program.BindUniformBlock("PassConstants", 0);
    }

    Common::CaseRenderResult Viewer::Render(RenderOptions const & options, ModelObject & modelObject, Engine::Camera &camera, Engine::ICameraManager & cameraManager, std::pair<std::uint32_t, std::uint32_t> desiredSize) {
        _frame.Resize(desiredSize);
        gl_using(_frame);

        glEnable(GL_DEPTH_TEST);

        cameraManager.Update(camera);

        glm::mat4 model = modelObject.GetTransform();
        _uniformBlock.Update({
            .NormalTransform = glm::transpose(glm::inverse(model)),
            .Model           = model,
            .View            = camera.GetViewMatrix(),
            .Projection      = camera.GetProjectionMatrix(float(desiredSize.first) / desiredSize.second),
            .LightDirection  = glm::normalize(options.LightDirection),
            .LightColor      = options.LightColor,
            .ObjectColor     = options.ObjectColor,
            .Ambient         = options.Ambient,
            .HasTexCoord     = modelObject.IsTexCoordAvailable(),
            .Wireframe       = options.Wireframe,
            .Flat            = options.Flat,
        });

        if (options.Wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        } else {
            glEnable(GL_CULL_FACE);
        }

        modelObject.Draw({ _program.Use() });

        if (options.Wireframe) {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        } else {
            glDisable(GL_CULL_FACE);
        }

        glDisable(GL_DEPTH_TEST);

        return {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

	void Viewer::SetupRenderOptionsUI(RenderOptions & options, Common::OrbitCameraManager & cameraManager) {
		if (ImGui::CollapsingHeader("Control")) {
            ImGui::Checkbox("Ease Touch", &cameraManager.EnableDamping);
            ImGui::SliderFloat("Spin Speed", &cameraManager.AutoRotateSpeed, 0.0f, 20.0f, "%.0f");
        }
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Appearance")) {
            ImGui::ColorEdit3("Color", glm::value_ptr(options.ObjectColor));
            ImGui::Checkbox("Wireframes", &options.Wireframe);
            if (! options.Wireframe) {
                ImGui::Checkbox("Face Normal", &options.Flat);
                ImGui::SliderFloat("Ambient", &options.Ambient, 0.0f, 0.1f, "%.2f");
                ImGui::SliderFloat("Light", &options.LightDirScalar, 0.0f, 360.0f, "%.0f deg");
                options.LightDirection = glm::vec3(glm::cos(glm::radians(options.LightDirScalar)), -1.0f, glm::sin(glm::radians(options.LightDirScalar)));
            }
        }
        ImGui::Spacing();
	}
} // namespace VCX::Labs::GeometryProcessing
