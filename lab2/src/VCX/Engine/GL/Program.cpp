#include <array>

#include <spdlog/spdlog.h>

#include "Engine/GL/Program.h"

namespace VCX::Engine::GL {
    static void CheckProgram(GLuint);

    void UniqueProgram::BindUniformBlock(char const * const name, std::uint32_t const bindingPoint) const {
        glUniformBlockBinding(Get(), glGetUniformBlockIndex(Get(), name), bindingPoint);
    }

    GLuint UniqueProgram::CreateProgramFromShaders(std::initializer_list<SharedShader> const & shaders) {
        auto const program { glCreateProgram() };
        for (auto const & shader : shaders) {
            glAttachShader(program, shader.Get());
        }
        glLinkProgram(program);
        CheckProgram(program);
        return program;
    }

    static void CheckProgram(GLuint const program) {
        GLint success;
        glGetProgramiv(program, GL_LINK_STATUS, &success);
        if (success) {
            spdlog::trace("glLinkProgram({})", program);
        } else {
            spdlog::error("glLinkProgram({}): failed.", program);
            std::array<GLchar, 1024> buf;
            glGetProgramInfoLog(program, buf.size(), nullptr, buf.data());
            spdlog::error("\t{}", buf.data());
            std::exit(EXIT_FAILURE);
        }
    }
} // namespace VCX::Engine::GL
