#include <algorithm>

#include <stb_image_write.h>

#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Common::ImGuiHelper {
    void TextCentered(std::string_view const text) {
        auto const textWidth   = ImGui::CalcTextSize(text.data()).x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - textWidth * .5f);
        ImGui::Text(text.data());
    }

    void TextRight(std::string_view const text) {
        auto const textWidth   = ImGui::CalcTextSize(text.data()).x;
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() - textWidth);
        ImGui::Text(text.data());
    }

    void ZoomTooltip(
        Engine::GL::UniqueTexture2D const &     tex,
        std::pair<std::uint32_t, std::uint32_t> texSize,
        ImVec2 const &                          pos,
        bool const                              flipped,
        float const                             regionSize,
        float const                             zoomLevel) {
        auto const [width, height] = texSize;
        ImGui::BeginTooltip();
        float regionX = std::clamp(pos.x - regionSize * .5f, 0.f, width - regionSize);
        float regionY = std::clamp(pos.y - regionSize * .5f, 0.f, height - regionSize);
        ImGui::Text("Min: (%.0f, %.0f)", regionX, regionY);
        ImGui::Text("Max: (%.0f, %.0f)", regionX + regionSize, regionY + regionSize);
        ImVec2 uv0 = ImVec2(regionX / width, flipped ? 1 - regionY / height : regionY / height);
        ImVec2 uv1 = ImVec2((regionX + regionSize) / width, flipped ? 1 - (regionY + regionSize) / height : (regionY + regionSize) / height);
        ImGui::Image(
            reinterpret_cast<void *>(std::uintptr_t(tex.Get())),
            ImVec2(regionSize * zoomLevel, regionSize * zoomLevel),
            uv0,
            uv1,
            { 1, 1, 1, 1 },
            { 1, 1, 1, .5f });
        ImGui::EndTooltip();
    }

    void SaveImage(
        Engine::GL::UniqueTexture2D const &     tex,
        std::pair<std::uint32_t, std::uint32_t> texSize,
        bool const                              flipped) {
        static char path[128]   = "a.png";
        bool        enableWrite = ImGui::Button("Save PNG Image");
        static bool saving      = true;
        ImGui::SameLine();
        ImGui::InputText("", path, IM_ARRAYSIZE(path));

        if (enableWrite) {
            saving = true;
            gl_using(tex);
            char * rawImg = new char[sizeof(char) * texSize.first * texSize.second * 3];
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, rawImg);
            glPixelStorei(GL_PACK_ALIGNMENT, 4);
            stbi_flip_vertically_on_write(flipped);
            stbi_write_png(path, texSize.first, texSize.second, 3, rawImg, 3 * texSize.first);
            delete[] rawImg;
            ImGui::OpenPopup("Saved");
        }

        ImVec2 center = ImGui::GetMainViewport()->GetCenter();
        ImGui::SetNextWindowPos(center, ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));

        if (ImGui::BeginPopupModal("Saved", &saving, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("Image saved to %s", path);
            ImGui::EndPopup();
        }
    }
} // namespace VCX::Labs::Common::ImGuiHelper
