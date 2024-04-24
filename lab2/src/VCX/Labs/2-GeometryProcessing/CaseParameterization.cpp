
#include <algorithm>
#include <array>
#include <iostream>

#include "Labs/2-GeometryProcessing/CaseParameterization.h"
#include "Labs/2-GeometryProcessing/Content.h"
#include "Labs/2-GeometryProcessing/tasks.h"
#include "Engine/loader.h"

namespace VCX::Labs::GeometryProcessing {

    static constexpr auto c_Size = std::pair<std::uint32_t, std::uint32_t> { 800U, 600U };

    CaseParameterization::CaseParameterization(Viewer & viewer):
        _viewer(viewer) {
        _cameraManager.EnablePan       = false;
        _cameraManager.AutoRotateSpeed = 0.f;
        _options.LightDirection = glm::vec3(glm::cos(glm::radians(_options.LightDirScalar)), -1.0f, glm::sin(glm::radians(_options.LightDirScalar)));
    }

    void CaseParameterization::OnSetupPropsUI() {
        Common::ImGuiHelper::SaveImage(_viewer.GetTexture(), _viewer.GetSize(), true);
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            _recompute |= ImGui::SliderInt("Iterations", &_numIterations, 0, 1000);
            if (_running) {
                static const std::string t = "Running.....";
                ImGui::Text(t.substr(0, 7 + (static_cast<int>(ImGui::GetTime() / 0.1f) % 6)).c_str());
            } else ImGui::NewLine();
        }
        ImGui::Spacing();

        Viewer::SetupRenderOptionsUI(_options, _cameraManager);
    }

    Common::CaseRenderResult CaseParameterization::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _task.Emplace([&]() {
                Engine::SurfaceMesh emptyMesh;
                Parameterization(Content::ModelMeshes[std::size_t(Assets::ExampleModel::Face)], emptyMesh, _numIterations);
                return emptyMesh;
            });
            _running = true;
        }
        if (_running && _task.HasValue()) {
            _running = false;
            _modelObject.ReplaceMesh(_task.Value());
        }
        return _viewer.Render(_options, _modelObject, _camera, _cameraManager, desiredSize);
    }

    void CaseParameterization::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }
} // namespace VCX::Labs::GeometryProcessing
