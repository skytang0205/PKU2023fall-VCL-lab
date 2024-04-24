
#include <algorithm>
#include <array>

#include "Labs/2-GeometryProcessing/CaseMarchingCubes.h"
#include "Labs/2-GeometryProcessing/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::GeometryProcessing {

    float CaseMarchingCubes::SphereSDF(const glm::vec3 & pos) {
        return glm::length(pos) - 0.50173;
    }

    float CaseMarchingCubes::TorusSDF(const glm::vec3 & pos) {
        const float a = 0.70243;
        const float r = 0.21029;
        float       x = sqrt(pos.x * pos.x + pos.z * pos.z);
        return sqrt((x - a) * (x - a) + pos.y * pos.y) - r;
    }

    CaseMarchingCubes::CaseMarchingCubes(Viewer & viewer):
        _viewer(viewer) {
        _cameraManager.EnablePan       = false;
        _cameraManager.AutoRotateSpeed = 0.f;
        _options.LightDirection = glm::vec3(glm::cos(glm::radians(_options.LightDirScalar)), -1.0f, glm::sin(glm::radians(_options.LightDirScalar)));
    }

    void CaseMarchingCubes::OnSetupPropsUI() {
        if (ImGui::BeginCombo("Geometry", _geometryTypeName[static_cast<std::uint32_t>(_type)].data())) {
            for (std::uint32_t i = 0; i < _geometryTypeName.size(); ++i) {
                bool selected = (static_cast<std::uint32_t>(_type) == i);
                if (ImGui::Selectable(_geometryTypeName[i].data(), selected)) {
                    if (! selected) {
                        _type      = ImplicitGeometryType(i);
                        _recompute = true;
                    }
                }
            }
            ImGui::EndCombo();
        }
        Common::ImGuiHelper::SaveImage(_viewer.GetTexture(), _viewer.GetSize(), true);
        ImGui::Spacing();

        if (ImGui::CollapsingHeader("Algorithm", ImGuiTreeNodeFlags_DefaultOpen)) {
            _recompute |= ImGui::SliderInt("Resolution", &_resolution, 10, 100);
            if (_running) {
                static const std::string t = "Running.....";
                ImGui::Text(t.substr(0, 7 + (static_cast<int>(ImGui::GetTime() / 0.1f) % 6)).c_str());
            } else ImGui::NewLine();
        }
        ImGui::Spacing();

        Viewer::SetupRenderOptionsUI(_options, _cameraManager);
    }

    Common::CaseRenderResult CaseMarchingCubes::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        if (_recompute) {
            _recompute = false;
            _task.Emplace([&]() {
                Engine::SurfaceMesh emptyMesh;
                if (_type == ImplicitGeometryType::Sphere)
                    MarchingCubes(emptyMesh, SphereSDF, glm::vec3 { -1, -1, -1 }, 2.f / _resolution, _resolution);
                else if (_type == ImplicitGeometryType::Torus)
                    MarchingCubes(emptyMesh, TorusSDF, glm::vec3 { -1, -1, -1 }, 2.f / _resolution, _resolution);
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

    void CaseMarchingCubes::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }
} // namespace VCX::Labs::GeometryProcessing
