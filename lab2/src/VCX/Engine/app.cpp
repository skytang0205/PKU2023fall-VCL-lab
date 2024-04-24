#include <cassert>
#include <cstdlib>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <spdlog/spdlog.h>
#include <stb_image.h>

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "Engine/App.h"

static GLFWwindow *                      g_glfwWindow;
static std::function<void()>             g_glfwWindowRefreshCallback;
static std::uint32_t                     g_ImGuiDefaultFontSize;
static std::span<std::string_view const> g_ImGuiDefaultFontFileNames;

GLFWwindow *                      glfwGetCurrentWindow() { return g_glfwWindow; }
std::uint32_t                     ImGuiGetDefaultFontSize() { return g_ImGuiDefaultFontSize; }
std::span<std::string_view const> ImGuiGetDefaultFontFileNames() { return g_ImGuiDefaultFontFileNames; }

namespace VCX::Engine {
    static std::pair<std::uint32_t, std::uint32_t> g_WindowSize;
    static std::pair<std::uint32_t, std::uint32_t> g_FrameSize;

    static decltype(glfwGetTime())                 g_LastTime        = 0;
    static float                                   g_DeltaTime       = 0;

    static std::uint32_t                           g_FramesCnt       = 0;
    static decltype(glfwGetTime())                 g_LastTotTime     = 0;
    static float                                   g_FramesPerSecond = 0;
    
    float                                   GetDeltaTime() { return g_DeltaTime; }
    float                                   GetFramesPerSecond() { return g_FramesPerSecond; }
    std::pair<std::uint32_t, std::uint32_t> GetCurrentWindowSize() { return g_WindowSize; }
    std::pair<std::uint32_t, std::uint32_t> GetCurrentFrameSize() { return g_FrameSize; }
}

namespace VCX::Engine::Internal {
    static void glfwErrorCallback(int const error, char const * const description) {
        spdlog::error("GLFW Error {}: {}", error, description);
        std::exit(EXIT_FAILURE);
    }

    static void glfwWindowRefreshCallback(GLFWwindow * const _) {
        g_glfwWindowRefreshCallback();
    }

    static void glfwWindowSizeCallback(GLFWwindow * const _, int const width, int const height) {
        assert(width >= 0 && height >= 0);
        g_WindowSize = { width, height };
    }

    static void glfwFramebufferSizeCallback(GLFWwindow * const _, int const width, int const height) {
        assert(width >= 0 && height >= 0);
        g_FrameSize = { width, height };
    }

    static void RunApp_InitGLFW(AppContextOptions const &);
    static void RunApp_InitGLFWWindowIcons(AppContextOptions const &);
    static void RunApp_InitGLFWWindowCallbacks(IApp &);
    static void RunApp_InitGLAD();
    static void RunApp_InitImGui(AppContextOptions const &);
    static void RunApp_Frame(IApp &);

    void RunApp_Init(AppContextOptions const & options) {
        RunApp_InitGLFW(options);
        RunApp_InitGLFWWindowIcons(options);
        RunApp_InitGLAD();
        RunApp_InitImGui(options);
    }

    void RunApp_Main(IApp && app) {
        RunApp_InitGLFWWindowCallbacks(app);
        glfwShowWindow(g_glfwWindow);
        while (! glfwWindowShouldClose(g_glfwWindow)) {
            RunApp_Frame(app);
            glfwPollEvents();
        }
    }

    void RunApp_Shutdown() {
        ImGui_ImplGlfw_Shutdown();
        ImGui_ImplOpenGL3_Shutdown();
        glfwTerminate();
    }

    static void RunApp_InitGLFW(AppContextOptions const & options) {
        glfwSetErrorCallback(glfwErrorCallback);
        if (glfwInit()) {
            spdlog::trace("GLFW: glfwInit()");
        } else {
            spdlog::error("GLFW: glfwInit() failed.");
            exit(EXIT_FAILURE);
        }

        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for macos
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
        assert(*(options.Title.cend()) == '\0');
        g_glfwWindow = glfwCreateWindow(
            options.WindowSize.first,
            options.WindowSize.second,
            options.Title.data(),
            nullptr,
            nullptr);
        if (g_glfwWindow) {
            spdlog::trace("GLFW: glfwCreateWindow(..)");
        } else {
            spdlog::error("GLFW: glfwCreateWindow(..) failed.");
            glfwTerminate();
            exit(EXIT_FAILURE);
        }

        int width;
        int height;
        glfwGetWindowSize(g_glfwWindow, &width, &height);
        g_WindowSize = { width, height };
        glfwGetFramebufferSize(g_glfwWindow, &width, &height);
        g_FrameSize = { width, height };

        glfwSetWindowRefreshCallback(g_glfwWindow, glfwWindowRefreshCallback);
        glfwSetWindowSizeCallback(g_glfwWindow, glfwWindowSizeCallback);
        glfwSetFramebufferSizeCallback(g_glfwWindow, glfwFramebufferSizeCallback);

        glfwMakeContextCurrent(g_glfwWindow);
        glfwSwapInterval(1);
    }

    static void RunApp_InitGLFWWindowIcons(AppContextOptions const & options) {
        auto const             iconsCount { options.IconFileNames.size() };
        std::vector<GLFWimage> icons;
        icons.resize(iconsCount);
        for (std::size_t i = 0; i < iconsCount; ++i) {
            auto const & iconFileName { options.IconFileNames[i] };
            auto &       icon { icons[i] };
            assert(*(iconFileName.cend()) == '\0');
            icon.pixels = stbi_load(iconFileName.data(), &icon.width, &icon.height, nullptr, 4);
        }
        glfwSetWindowIcon(
            g_glfwWindow,
            icons.size(),
            icons.data());
        for (auto const & icon : icons) {
            stbi_image_free(icon.pixels);
        }
    }

    static void RunApp_InitGLAD() {
        if (gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
            spdlog::trace("GLAD: gladLoadGLLoader(..)");
        } else {
            spdlog::error("GLAD: gladLoadGLLoader(..) failed.");
            glfwTerminate();
            exit(EXIT_FAILURE);
        }
    }

    static void RunApp_InitImGui(AppContextOptions const & options) {
        ImGui::CreateContext();
        for (auto const & fontFileName : options.FontFileNames) {
            assert(*(fontFileName.cend()) == '\0');
            ImGui::GetIO().Fonts->AddFontFromFileTTF(fontFileName.data(), options.FontSize);
        }
        ImGui::GetIO().IniFilename = nullptr;
        ImGui::GetIO().LogFilename = nullptr;

        ImGui_ImplOpenGL3_Init();
        ImGui_ImplGlfw_InitForOpenGL(g_glfwWindow, true);

        g_ImGuiDefaultFontSize      = options.FontSize;
        g_ImGuiDefaultFontFileNames = options.FontFileNames;
    }

    static void RunApp_InitGLFWWindowCallbacks(IApp & app) {
        g_glfwWindowRefreshCallback = [&app]() { RunApp_Frame(app); };
    }

    static void RunApp_Frame(IApp & app) {
        auto const currentTime = glfwGetTime();
        g_DeltaTime = currentTime - g_LastTime;
        g_LastTime  = currentTime;
        if (g_FramesCnt++; currentTime - g_LastTotTime >= 1.) {
            g_FramesPerSecond = float(g_FramesCnt / (currentTime - g_LastTotTime));
            g_FramesCnt = 0;
            g_LastTotTime = currentTime;
        }

        glViewport(0, 0, g_FrameSize.first, g_FrameSize.second);
        glClearColor(0, 0, 0, 1);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        app.OnFrame();

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(g_glfwWindow);
    }
} // namespace VCX::Engine::Internal
