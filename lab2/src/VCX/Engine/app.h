#pragma once

#include <cstdlib>
#include <span>
#include <string_view>

struct GLFWwindow *               glfwGetCurrentWindow();
std::uint32_t                     ImGuiGetDefaultFontSize();
std::span<std::string_view const> ImGuiGetDefaultFontFileNames();

namespace VCX::Engine {
    float                                   GetDeltaTime();
    float                                   GetFramesPerSecond();
    std::pair<std::uint32_t, std::uint32_t> GetCurrentWindowSize();
    std::pair<std::uint32_t, std::uint32_t> GetCurrentFrameSize();

    /**
     * @brief The interface of applications, used as the base class.
     * 
     * \sa RunApp()
     */
    class IApp {
    public:
        /**
         *  @brief Make draw calls, ImGui calls and other per-frame updates.
         */
        virtual void OnFrame() = 0;
    };

    /**
     * @brief Structure of options required to create context of the application.
     * 
     * \note The data of \c std::string_view in fields of this struct will be passed directly to third-party libraries as C-style strings,
     * so they must be null-terminated, and the caller should handle encoding and path-related issues.
     */
    struct AppContextOptions {
        std::string_view                        Title;         /**< The title of the application, used by the main window. */
        std::pair<std::uint32_t, std::uint32_t> WindowSize;    /**< The desired size of the main window, in the form of (width, height). */
        std::uint32_t                           FontSize;      /**< The desired font size for rendering ImGui texts. */
        std::span<std::string_view const>       IconFileNames; /**< The icons for the main window. Multiple icons of different resolutions can be provided for better visual results. */
        std::span<std::string_view const>       FontFileNames; /**< The fonts for rendering ImGui texts. */
    };
} // namespace VCX::Engine

namespace VCX::Engine::Internal {
    void RunApp_Init(AppContextOptions const &);
    void RunApp_Main(IApp &&);
    void RunApp_Shutdown();
}

namespace VCX::Engine {
    /**
     * @brief  Create a graphics (GLFW + GLAD + ImGui) context, construct an instance of TApp,
     *         runs a render-loop and calls IApp::OnFrame(), etc., on each frame.
     * 
     * @tparam TApp    A derived class of IApp implementing IApp::OnFrame(), which should have a constructor without parameters.
     * @param  options The options used to create application context.
     * @return         0 indicating successful exiting.
     */
    template<typename TApp>
        requires std::is_base_of_v<IApp, TApp> && std::is_constructible_v<TApp>
    int RunApp(AppContextOptions const & options) {
        Internal::RunApp_Init(options);
        Internal::RunApp_Main(TApp());
        Internal::RunApp_Shutdown();
        return 0;
    }
}
