#include <algorithm>
#include <array>

#include "Labs/1-Drawing2D/CaseDrawBezier.h"
#include "Labs/1-Drawing2D/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Drawing2D {

    static constexpr auto c_Size = std::pair(320U, 320U);

    static void DrawPoint(Common::ImageRGB & canvas, glm::vec3 color, glm::ivec2 pos) {
        for (int dx = -2; dx <= 2; ++dx) {
            for (int dy = -2; dy <= 2; ++dy) {
                canvas.At(pos.x + dx, pos.y + dy) = color;
            }
        }
    }

    CaseDrawBezier::CaseDrawBezier():
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }),
        _empty(Common::CreateCheckboardImageRGB(c_Size.first, c_Size.second)) {
    }

    void CaseDrawBezier::OnSetupPropsUI() {
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

    Common::CaseRenderResult CaseDrawBezier::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        auto const [width, height] = c_Size;
        if (_recompute) {
            _recompute = false;
            auto                  tex { Common::CreateCheckboardImageRGB(c_Size.first, c_Size.second) };
            constexpr std::size_t n    = 20;
            glm::fvec2            prev = _handles[0];
            for (std::size_t i = 1; i <= n; ++i) {
                auto const curr = CalculateBezierPoint(
                    _handles,
                    float(i) / float(n));
                DrawLine(tex, { 0, 0, 0 }, prev, curr);
                prev = curr;
            }
            for (std::size_t i = 0; i < _handles.size(); ++i) {
                DrawPoint(tex, { 0.2, 0.7, 0.4 }, _handles[i]);
            }
            _texture.Update(tex);
        }

        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseDrawBezier::OnProcessInput(ImVec2 const & pos) {
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
                for (int i = 0; i < _handles.size(); ++i) {
                    float dist = (pos.x - _handles[i].x) * (pos.x - _handles[i].x) + (pos.y - _handles[i].y) * (pos.y - _handles[i].y);
                    if (dist < minDist) {
                        minDist    = dist;
                        _selectIdx = i;
                    }
                }
            }
            if (_selectIdx != -1) {
                _handles[_selectIdx]   = _handles[_selectIdx] + glm::fvec2(int(delta.x), int(delta.y));
                _handles[_selectIdx].x = std::min(_handles[_selectIdx].x, float(c_Size.first - 3));
                _handles[_selectIdx].x = std::max(_handles[_selectIdx].x, 2.f);
                _handles[_selectIdx].y = std::min(_handles[_selectIdx].y, float(c_Size.second - 3));
                _handles[_selectIdx].y = std::max(_handles[_selectIdx].y, 2.f);
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
