#include <algorithm>
#include <string>

#include <fmt/core.h>

#include "imgui_impl_opengl3.h"

#include "Engine/app.h"
#include "Labs/Common/ImGuiHelper.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Common {
    UI::UI(UIOptions && options):
        _options(options),
        _style(ImGui::GetStyle()) {
        glfwSetWindowUserPointer(glfwGetCurrentWindow(), this);

        glfwGetWindowContentScale(glfwGetCurrentWindow(), &_scale.x, &_scale.y);
        updateScaleUI();
        if (_scaleUI != 1.f)
            updateFonts();
        updateStyle();
        updateLayout();

        _prevWindowSizeCallback = glfwSetWindowSizeCallback(glfwGetCurrentWindow(), glfwWindowSizeCallback);
        _prevWindowContentScaleCallback = glfwSetWindowContentScaleCallback(glfwGetCurrentWindow(), glfwWindowContentScaleCallback);
    }

    void UI::Setup(std::span<std::reference_wrapper<Common::ICase>> cases, std::size_t &caseId) {
        auto newCaseId = caseId;
        if (! _layout.SideWindowHidden)
            newCaseId = setupSideWindow(cases, caseId);

        setupMainWindow(cases[caseId]);
        
        if (caseId != newCaseId)
            caseId = newCaseId;
        if (_layout.SideWindowHiddenToggle) {
            _layout.SideWindowHidden = ! _layout.SideWindowHidden;
            _layout.SideWindowHiddenToggle = false;
            updateLayout();
        }
    }

    std::size_t UI::setupSideWindow(std::span<std::reference_wrapper<Common::ICase>> cases, std::size_t const caseId) {
        std::size_t newCaseId = caseId;

        ImGui::PushStyleColor(ImGuiCol_WindowBg, 0xFF262525);
        ImGui::PushStyleColor(ImGuiCol_ScrollbarBg, 0xFF262525);
        ImGui::SetNextWindowPos(_layout.SideWindowPosition);
        ImGui::SetNextWindowSize(_layout.SideWindowSize);
        // clang-format off
        ImGui::Begin(
            "Side Window", nullptr,
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse);
        // clang-format on
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + _layout.Spacing * 6);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + _layout.Spacing * 3);
        ImGui::AlignTextToFramePadding();
        ImGui::Text("EXPLORER");

        ImGui::SetCursorScreenPos(_layout.CaseChildPosition);
        ImGui::BeginChild("Case Child", _layout.CaseChildSize);
        ImGui::PushStyleVar(ImGuiStyleVar_SelectableTextAlign, { .0f, .5f });
        for (std::size_t i = 0; i < cases.size(); ++i) {
            if (ImGui::Selectable(
                    (fmt::format("     Case {}: {}", i + 1, cases[i].get().GetName()) + '\0').c_str(),
                    caseId == i,
                    0,
                    ImVec2(0, ImGui::GetTextLineHeight() + 3 * _style.FramePadding.y)))
                newCaseId = i;
        }
        ImGui::PopStyleVar();
        ImGui::EndChild();

        ImGui::Separator();

        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + _layout.Spacing * 6);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + _layout.Spacing * 3);
        ImGui::AlignTextToFramePadding();
        ImGui::Text("PROPERTIES");

        ImGui::SetCursorScreenPos(_layout.UserChildPosition);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { _style.FramePadding.x * 2, 0 });
        ImGui::BeginChild("User Child", _layout.UserChildSize, false, ImGuiWindowFlags_AlwaysUseWindowPadding);
        ImGui::PushStyleVar(ImGuiStyleVar_IndentSpacing, _style.FramePadding.x * 2);
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, { _style.FramePadding.x * 2, _style.FramePadding.y * 2 });
        ImGui::PushStyleVar(ImGuiStyleVar_ItemInnerSpacing, { _style.FramePadding.x * 2, _style.FramePadding.y * 2 });
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { _style.FramePadding.x * 2, _style.FramePadding.y * 2 });

        cases[caseId].get().OnSetupPropsUI();

        ImGui::PopStyleVar(4);
        ImGui::EndChild();
        ImGui::PopStyleVar();

        ImGui::End();
        ImGui::PopStyleColor(2);

        return newCaseId;
    }

    void UI::setupMainWindow(Common::ICase & casei) {
        ImGui::PushStyleColor(ImGuiCol_WindowBg, 0xFF2D2D2D);
        ImGui::PushStyleColor(ImGuiCol_ChildBg, 0xFF1E1E1E);
        ImGui::PushStyleColor(ImGuiCol_ScrollbarBg, 0xFF1E1E1E);
        ImGui::SetNextWindowPos(_layout.MainWindowPosition);
        ImGui::SetNextWindowSize(_layout.MainWindowSize);
        // clang-format off
        ImGui::Begin(
            "Main Window", nullptr,
            ImGuiWindowFlags_NoTitleBar |
            ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove |
            ImGuiWindowFlags_NoCollapse |
            ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_NoScrollWithMouse |
            ImGuiWindowFlags_NoFocusOnAppearing);
        // clang-format on
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + _layout.Spacing);
        ImGui::SetCursorPosY(ImGui::GetCursorPosY() + _layout.Spacing);

        ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 255.f);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        if (_layout.SideWindowHidden) {
            if (ImGui::ArrowButton("##right", ImGuiDir_Right)) {
                _layout.SideWindowHiddenToggle = true;
            }
        } else {
        if (ImGui::ArrowButton("##left", ImGuiDir_Left)) {
                _layout.SideWindowHiddenToggle = true;
            }
        }
        ImGui::PopStyleColor();
        ImGui::PopStyleVar();

        ImGui::SameLine();
        ImGui::SetCursorPosX(_layout.MainWindowSize.x * .5f);
        ImGuiHelper::TextCentered("RESULT VIEWER");
        ImGui::SameLine();
        ImGui::SetCursorPosX(_layout.MainWindowSize.x - _layout.Spacing * 2);
        ImGui::PushFont(ImGui::GetIO().Fonts->Fonts[1]);
        ImGuiHelper::TextRight(fmt::format("FPS:{:>4.0f}", Engine::GetFramesPerSecond()) + '\0');
        ImGui::PopFont();

        auto const result = casei.OnRender({ std::uint32_t(_layout.ContentChildSize.x), std::uint32_t(_layout.ContentChildSize.y) });
        auto const canvasWidth = result.ImageSize.first;
        auto const canvasHeight = result.ImageSize.second;
        auto const tex = reinterpret_cast<void *>(std::uintptr_t(result.Image.Get()));

        ImVec2 canvasRelativePosition;

        if (result.Fixed) {
            canvasRelativePosition = {
                std::max(std::floor((_layout.ContentChildSize.x - canvasWidth ) * .5f) - _style.ScrollbarSize, .0f),
                std::max(std::floor((_layout.ContentChildSize.y - canvasHeight) * .5f) - _style.ScrollbarSize, .0f)
            };
            ImGui::GetWindowDrawList()->AddRectFilled(
                _layout.ContentChildPosition,
                { _layout.ContentChildPosition.x + _layout.ContentChildSize.x, _layout.ContentChildPosition.y + _layout.ContentChildSize.y },
                0xFF1E1E1E);
            ImGui::SetCursorScreenPos({ _layout.ContentChildPosition.x + _style.ScrollbarSize, _layout.ContentChildPosition.y + _style.ScrollbarSize });
            
            if (canvasRelativePosition.x != .0f && canvasRelativePosition.y != .0f) {
                ImGui::PushStyleColor(ImGuiCol_ScrollbarGrab, 0xFF1E1E1E);
                ImGui::PushStyleColor(ImGuiCol_ScrollbarGrabActive, {0, 0, 0, 0});
                ImGui::PushStyleColor(ImGuiCol_ScrollbarGrabHovered, {0, 0, 0, 0});
            }
            ImGui::BeginChild(
                "Content Child",
                { _layout.ContentChildSize.x - _style.ScrollbarSize, _layout.ContentChildSize.y - _style.ScrollbarSize },
                false,
                ImGuiWindowFlags_AlwaysHorizontalScrollbar | ImGuiWindowFlags_AlwaysVerticalScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        } else {
            canvasRelativePosition = { 0, 0 };
            ImGui::SetCursorScreenPos(_layout.ContentChildPosition);
            ImGui::BeginChild(
                "Content Child",
                _layout.ContentChildSize,
                false,
                ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
        }

        ImGui::SetCursorPos(canvasRelativePosition);
        ImVec2 cornerPos = ImGui::GetCursorScreenPos();
        if (result.Flipped)
            ImGui::Image(tex, { 1.f * canvasWidth, 1.f * canvasHeight }, { 0.f, 1.f }, { 1.f, 0.f });
        else
            ImGui::Image(tex, { 1.f * canvasWidth, 1.f * canvasHeight });
        ImGui::SetCursorPos(canvasRelativePosition);
        ImGui::InvisibleButton("##io", { 1.f * canvasWidth, 1.f * canvasHeight }, ImGuiButtonFlags_MouseButtonMask_);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, { _layout.Spacing * 2, _layout.Spacing * 2 });
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, { _layout.Spacing, _layout.Spacing });
 
        casei.OnProcessInput({ ImGui::GetIO().MousePos.x - cornerPos.x, ImGui::GetIO().MousePos.y - cornerPos.y });

        ImGui::PopStyleVar(2);
        ImGui::EndChild();
        
        if (result.Fixed) {
            if (canvasRelativePosition.x != .0f && canvasRelativePosition.y != .0f)
                ImGui::PopStyleColor(3);
        }

        ImGui::End();
        ImGui::PopStyleColor(3);
    }

    void UI::updateStyle() {
        _style                  = {};
        _style.WindowPadding    = { 0, 0 };
        _style.WindowBorderSize = 0;
        _style.WindowRounding   = 0;
        _style.ChildBorderSize  = 0;
        _style.ChildRounding    = 0;
        _style.ItemSpacing      = { 0, 0 };
        _style.ScrollbarSize    = 10;

        _style.ScaleAllSizes(_scaleUI);
    }

    void UI::updateFonts() {
        ImGui_ImplOpenGL3_DestroyFontsTexture();
        
        ImGui::GetIO().Fonts->Clear();
        float const fontSize = std::floor(ImGuiGetDefaultFontSize() * _scaleUI);
        for (auto const & fontPath : ImGuiGetDefaultFontFileNames()) {
            assert(*(fontPath.cend()) == '\0');
            ImGui::GetIO().Fonts->AddFontFromFileTTF(fontPath.data(), fontSize);
        }
        
        ImGui_ImplOpenGL3_CreateFontsTexture();
    }

    void UI::updateLayout() {
        const auto [windowWidth, windowHeight] = Engine::GetCurrentWindowSize();
        float const fontSize      = std::floor(ImGuiGetDefaultFontSize() * _scaleUI);
        ImVec2 const framePadding = _style.FramePadding;

        _layout.Spacing = std::floor(2 * _scaleUI);

        _layout.SideWindowPosition = { 0, 0 };
        if (_layout.SideWindowHidden) {
            _layout.SideWindowSize = { 0, 0 };
            _layout.CaseChildPosition = { 0, 0 };
            _layout.CaseChildSize     = { 0, 0 };
            _layout.UserChildPosition = { 0, 0 };
            _layout.UserChildSize     = { 0, 0 };
        } else {
            _layout.SideWindowSize = {
                std::floor(_options.SideWindowWidth * _scaleUI),
                1.f * windowHeight
            };
            _layout.CaseChildPosition = {
                _layout.SideWindowPosition.x,
                _layout.SideWindowPosition.y + fontSize + 2 * framePadding.y + 6 * _layout.Spacing
            };
            _layout.CaseChildSize = {
                _layout.SideWindowSize.x,
                std::floor(_layout.SideWindowSize.y * .5f) - _layout.CaseChildPosition.y
            };
            _layout.UserChildPosition = {
                _layout.SideWindowPosition.x,
                _layout.CaseChildPosition.y + _layout.CaseChildSize.y + fontSize + 2 * framePadding.y + 6 * _layout.Spacing
            };
            _layout.CaseChildSize = {
                _layout.SideWindowSize.x,
                _layout.SideWindowSize.y - _layout.UserChildPosition.y
            };
        }
        _layout.MainWindowPosition = {
            _layout.SideWindowPosition.x + _layout.SideWindowSize.x,
            0
        };
        _layout.MainWindowSize = {
            windowWidth - _layout.MainWindowPosition.x,
            1.f * windowHeight
        };
        _layout.ContentChildPosition = {
            _layout.MainWindowPosition.x,
            _layout.MainWindowPosition.y + fontSize + 2 * framePadding.y + 2 * _layout.Spacing
        };
        _layout.ContentChildSize = {
            _layout.MainWindowSize.x,
            _layout.MainWindowSize.y - _layout.ContentChildPosition.y
        };
    }

    void UI::updateScaleUI() {
        auto [frameWidth, frameHeight] = Engine::GetCurrentFrameSize();
        auto [windowWidth, windowHeight] = Engine::GetCurrentWindowSize();
        glm::fvec2 const scaleUI = _scale / glm::fvec2(1.f * frameWidth / windowWidth, 1.f * frameHeight / windowHeight);
        _scaleUI = std::min(scaleUI.x, scaleUI.y);
        glfwSetWindowSizeLimits(
            glfwGetCurrentWindow(),
            _options.SideWindowWidth * _scaleUI * 2,
            _options.SideWindowWidth * _scaleUI * 1.5,
            GLFW_DONT_CARE,
            GLFW_DONT_CARE);
    }

    void UI::glfwWindowSizeCallback(GLFWwindow * window, int width, int height) {
        if (_prevWindowSizeCallback)
            _prevWindowSizeCallback(window, width, height);

        auto ui = reinterpret_cast<UI *>(glfwGetWindowUserPointer(window));
        ui->updateLayout();
    }
    
    void UI::glfwWindowContentScaleCallback(GLFWwindow * window, float xScale, float yScale) {
        if (_prevWindowContentScaleCallback)
            _prevWindowContentScaleCallback(window, xScale, yScale);

        auto ui = reinterpret_cast<UI *>(glfwGetWindowUserPointer(window));
        ui->_scale = { xScale, yScale };
        ui->updateScaleUI();
        ui->updateFonts();
        ui->updateStyle();
        ui->updateLayout();
    }
}
