#include <algorithm>
#include <array>

#include "Engine/loader.h"
#include "Labs/1-Drawing2D/CaseSSAA.h"
#include "Labs/1-Drawing2D/tasks.h"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Drawing2D {

    static constexpr auto c_Size = std::pair(320U, 320U);

    CaseSSAA::CaseSSAA():
        _texture({ .MinFilter = Engine::GL::FilterMode::Linear, .MagFilter = Engine::GL::FilterMode::Nearest }),
        _input(Common::AlphaBlend(
            Engine::LoadImageRGBA("assets/images/tie.jpg"),
            Common::CreatePureImageRGB(2500, 2500, { 0, 0, 0 }))),
        _empty(Common::CreatePureImageRGB(c_Size.first, c_Size.second, { 0, 0, 0 })) {
    }

    void CaseSSAA::OnSetupPropsUI() {
        ImGui::Checkbox("Zoom Tooltip", &_enableZoom);
        int oldRate = _sampleRate;
        ImGui::SliderInt("Sample Rate", &_sampleRate, 1, 12);
        Common::ImGuiHelper::SaveImage(_texture, c_Size);
        ImGui::Spacing();
        if (_sampleRate != oldRate) _recompute = true;
    }

    Common::CaseRenderResult CaseSSAA::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        auto const [width, height] = c_Size;
        if (_recompute) {
            _recompute = false;
            _task.Emplace([=, &input = _input]() {
                Common::ImageRGB tex(c_Size.first, c_Size.second);
                Supersample(tex, input, _sampleRate);
                return tex;
            });
        }
        _texture.Update(_task.ValueOr(_empty));
        return Common::CaseRenderResult {
            .Fixed     = true,
            .Image     = _texture,
            .ImageSize = c_Size,
        };
    }

    void CaseSSAA::OnProcessInput(ImVec2 const & pos) {
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
        if (_enableZoom && ! anyHeld && ImGui::IsItemHovered())
            Common::ImGuiHelper::ZoomTooltip(_texture, c_Size, pos);
    }
} // namespace VCX::Labs::Drawing2D
