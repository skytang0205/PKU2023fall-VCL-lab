#include <array>

#include <spdlog/spdlog.h>

#include "Engine/GL/Shader.h"
#include "Engine/loader.h"

namespace VCX::Engine::GL {
    static ShaderType ShaderTypeFromExtension(std::filesystem::path const &);
    static void CheckShader(GLuint);

    SharedShader::SharedShader(std::filesystem::path const & fileName):
        SharedShader(ShaderTypeFromExtension(fileName.extension()), fileName) {}

    SharedShader::SharedShader(
        ShaderType const              type,
        std::filesystem::path const & fileName):
        SharedShader(type, LoadBytes(fileName)) {}

    SharedShader::SharedShader(
        ShaderType const               type,
        std::vector<std::byte> const & blob):
        Shared(glCreateShader(GLenum(type))) {
        std::array<GLchar const *, 1> sources { reinterpret_cast<GLchar const *>(blob.data()) };
        std::array<GLint, 1>          lengths { GLint(blob.size()) };
        auto const                    shader { Get() };
        glShaderSource(shader, 1, sources.data(), lengths.data());
        glCompileShader(shader);
        CheckShader(shader);
    }

    static ShaderType ShaderTypeFromExtension(std::filesystem::path const & ext) {
             if (ext == ".vert") return ShaderType::Vertex;
        else if (ext == ".tesc") return ShaderType::TessControl;
        else if (ext == ".tese") return ShaderType::TessEvaluation;
        else if (ext == ".geom") return ShaderType::Geometry;
        else if (ext == ".frag") return ShaderType::Fragment;
        else {
            spdlog::error("VCX::Engine::GL::ShaderTypeFromExtension({}): undetermined shader type.", ext.string());
            std::exit(EXIT_FAILURE);
        }
    }

    static void CheckShader(GLuint const shader) {
        GLint success;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
        if (success) {
            spdlog::trace("glCompileShader({})", shader);
        } else {
            spdlog::error("glCompileShader({}): failed.", shader);
            std::array<GLchar, 1024> buf;
            glGetShaderInfoLog(shader, buf.size(), nullptr, buf.data());
            spdlog::error("\t{}", buf.data());
            std::exit(EXIT_FAILURE);
        }
    }
} // namespace VCX::Engine::GL
