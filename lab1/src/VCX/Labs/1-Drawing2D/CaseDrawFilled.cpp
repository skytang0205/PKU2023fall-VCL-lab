#include <algorithm>
#include <array>

#include "Labs/1-Drawing2D/CaseDrawFilled.h"
#include "Labs/1-Drawing2D/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Drawing2D {

    static constexpr auto c_Size = std::pair(320U, 320U);

    CaseDrawFilled::CaseDrawFilled():
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }),
        _empty(Common::CreateCheckboardImageRGB(c_Size.first, c_Size.second)) {
    }

    void CaseDrawFilled::OnSetupPropsUI() {
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        ImGui::Checkbox("Enable Left-Button-Drag", &_enableLeftDrag);
        ImGui::TextWrapped(
            _enableLeftDrag ? 
            "Hint: use the left mouse button to drag the point." :
            "Hint: use the right mouse button to drag the point."
        );
        Common::ImGuiHelper::SaveImage(_texture, c_Size);
        ImGui::Spacing();
    }

    Common::CaseRenderResult CaseDrawFilled::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        auto const [width, height] = c_Size;
        if (_recompute) {
            _recompute = false;
            auto tex { Common::CreateCheckboardImageRGB(c_Size.first, c_Size.second) };
            DrawTriangleFilled(tex, { 0.6, 0.2, 0.1 }, _vertices[0], _vertices[1], _vertices[2]);
            _texture.Update(tex);
        }
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseDrawFilled::OnProcessInput(ImVec2 const & pos) {
        auto         window  = ImGui::GetCurrentWindow();
        bool         hovered = false;
        bool         anyHeld = false;
        ImVec2 const delta   = ImGui::GetIO().MouseDelta;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hovered, &anyHeld);
        if (! hovered) return;
        if (ImGui::IsMouseDown(_enableLeftDrag ? ImGuiMouseButton_Left : ImGuiMouseButton_Right)) {
            _recompute = true;
            // find closest point
            if (_selectIdx == -1) {
                float minDist = 200;
                for (int i = 0; i < 3; ++i) {
                    float dist = (pos.x - _vertices[i].x) * (pos.x - _vertices[i].x) + (pos.y - _vertices[i].y) * (pos.y - _vertices[i].y);
                    if (dist < minDist) {
                        minDist    = dist;
                        _selectIdx = i;
                    }
                }
            }
            if (_selectIdx != -1) {
                _vertices[_selectIdx]   = _vertices[_selectIdx] + glm::ivec2(int(delta.x), int(delta.y));
                _vertices[_selectIdx].x = std::min(_vertices[_selectIdx].x, int(c_Size.first - 2));
                _vertices[_selectIdx].x = std::max(_vertices[_selectIdx].x, 1);
                _vertices[_selectIdx].y = std::min(_vertices[_selectIdx].y, int(c_Size.second - 2));
                _vertices[_selectIdx].y = std::max(_vertices[_selectIdx].y, 1);
            }
        } else {
            _selectIdx = -1;
        }
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.x != 0.f)
            ImGui::SetScrollX(window, window->Scroll.x - delta.x);
        if (ImGui::IsMouseDown(ImGuiMouseButton_Left) && delta.y != 0.f)
            ImGui::SetScrollY(window, window->Scroll.y - delta.y);
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_texture, c_Size, pos);
    }
} // namespace VCX::Labs::Drawing2D
