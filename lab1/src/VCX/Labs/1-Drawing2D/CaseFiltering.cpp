#include <algorithm>
#include <array>

#include "Engine/loader.h"
#include "Labs/1-Drawing2D/CaseFiltering.h"
#include "Labs/1-Drawing2D/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Drawing2D {

    static constexpr auto c_Size = std::pair(640U, 640U);

    CaseFiltering::CaseFiltering():
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }),
        _input(Common::AlphaBlend(
            Engine::LoadImageRGBA("assets/images/dinosaur-640x640.png"),
            Common::CreatePureImageRGB(c_Size.first, c_Size.second, { 0, 0, 0 }))),
        _empty(Common::CreatePureImageRGB(c_Size.first, c_Size.second, { 0, 0, 0 })) {
    }

    void CaseFiltering::OnSetupPropsUI() {
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        ImGui::Text("Filter Type");
        FilterType oldType = _filterType;
        ImGui::Bullet();
        if (ImGui::Selectable("Original", _filterType == FilterType::Original)) _filterType = FilterType::Original;
        ImGui::Bullet();
        if (ImGui::Selectable("Blur", _filterType == FilterType::Blur)) _filterType = FilterType::Blur;
        ImGui::Bullet();
        if (ImGui::Selectable("Edge Detection", _filterType == FilterType::Edge)) _filterType = FilterType::Edge;
        Common::ImGuiHelper::SaveImage(_texture, c_Size);
        ImGui::Spacing();
        if (oldType != _filterType) _recompute = true;
    }

    Common::CaseRenderResult CaseFiltering::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        auto const [width, height] = c_Size;
        if (_recompute) {
            _recompute = false;
            switch (_filterType) {
            case FilterType::Original:
                _task.Emplace([&input = _input]() {
                    return input;
                });
                break;
            case FilterType::Blur:
                _task.Emplace([&input = _input]() {
                    Common::ImageRGB tex(c_Size.first, c_Size.second);
                    Blur(tex, input);
                    return tex;
                });
                break;
            case FilterType::Edge:
                _task.Emplace([&input = _input]() {
                    Common::ImageRGB tex(c_Size.first, c_Size.second);
                    Edge(tex, input);
                    return tex;
                });
                break;
            default:
                break;
            }
        }
        _texture.Update(_task.ValueOr(_empty));
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseFiltering::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
            ImGui::SetScrollX(window, window->Scroll.x - delta.x);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
            ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered()) {
            Common::ImGuiHelper::ZoomTooltip(_texture, c_Size, pos);
        }
    }
} // namespace VCX::Labs::Drawing2D
