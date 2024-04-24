#pragma once

#include <filesystem>

#include "Engine/GL/resource.hpp"
#include "Engine/prelude.hpp"

namespace VCX::Engine::GL {
    enum class ShaderType : GLenum {
        Vertex         = GL_VERTEX_SHADER,
        TessControl    = GL_TESS_CONTROL_SHADER,
        TessEvaluation = GL_TESS_EVALUATION_SHADER,
        Geometry       = GL_GEOMETRY_SHADER,
        Fragment       = GL_FRAGMENT_SHADER,
    };

    struct ShaderTrait {
        static auto constexpr & Delete = glDeleteShader;
    };

    class SharedShader : public Shared<ShaderTrait> {
    public:
        // clang-format off
        SharedShader(                  std::filesystem::path  const &);
        SharedShader(ShaderType const, std::filesystem::path  const &);
        SharedShader(ShaderType const, std::vector<std::byte> const &);
        // clang-format on
    };
} // namespace VCX::Engine::GL
