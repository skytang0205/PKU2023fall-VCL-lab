#pragma once

#include <functional>
#include <span>

#include "Labs/Common/ImageRGB.h"

namespace VCX::Labs::Drawing2D {
    void      DrawLine(Common::ImageRGB &, glm::vec3, glm::ivec2, glm::ivec2);
    void      DrawTriangleFilled(Common::ImageRGB &, glm::vec3, glm::ivec2, glm::ivec2, glm::ivec2);
    void      Supersample(Common::ImageRGB &, Common::ImageRGB const &, int rate);
    glm::vec2 CalculateBezierPoint(std::span<glm::vec2>, float);
    void      Blur(Common::ImageRGB &, Common::ImageRGB const &);
    void      Edge(Common::ImageRGB &, Common::ImageRGB const &);
    void      DitheringThreshold(Common::ImageRGB &, Common::ImageRGB const &);
    void      DitheringRandomUniform(Common::ImageRGB &, Common::ImageRGB const &);
    void      DitheringRandomBlueNoise(Common::ImageRGB &, Common::ImageRGB const &, Common::ImageRGB const &);
    void      DitheringOrdered(Common::ImageRGB &, Common::ImageRGB const &);
    void      DitheringErrorDiffuse(Common::ImageRGB &, Common::ImageRGB const &);
    void      Inpainting(Common::ImageRGB &, Common::ImageRGB const &, Common::ImageRGB const &, const glm::ivec2 & offset);
}
