#pragma once

#include <span>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <imgui.h>

#include "Labs/Common/ICase.h"

namespace VCX::Labs::Common {
    struct UIOptions {
        float SideWindowWidth { 300 };
    };

    class UI {
    public:
        UI(UIOptions && options);

        void Setup(std::span<std::reference_wrapper<Common::ICase>> cases, std::size_t &caseId);

    protected:
        UIOptions const _options;

        ImGuiStyle & _style;

        struct Layout {
            float Spacing;

            ImVec2 SideWindowPosition;
            ImVec2 SideWindowSize;
            ImVec2 CaseChildPosition;
            ImVec2 CaseChildSize;
            ImVec2 UserChildPosition;
            ImVec2 UserChildSize;

            ImVec2 MainWindowPosition;
            ImVec2 MainWindowSize;
            ImVec2 ContentChildPosition;
            ImVec2 ContentChildSize;

            bool SideWindowHidden { false };
            bool SideWindowHiddenToggle { false };
        } _layout;

    private:
        std::size_t setupSideWindow(std::span<std::reference_wrapper<Common::ICase>> cases, std::size_t const caseId);
        void setupMainWindow(Common::ICase & casei);

        void updateStyle();
        void updateFonts();
        void updateLayout();

        glm::fvec2 _scale;
        float      _scaleUI;

        void updateScaleUI();

        inline static GLFWwindowsizefun _prevWindowSizeCallback = nullptr;
        inline static GLFWwindowcontentscalefun _prevWindowContentScaleCallback = nullptr;

        static void glfwWindowSizeCallback(struct GLFWwindow * window, int width, int height);
        static void glfwWindowContentScaleCallback(struct GLFWwindow * window, float xScale, float yScale);
    };
} // namespace VCX::Labs::Common
