#pragma once

#include <initializer_list>
#include <unordered_map>

#include <glm/gtc/type_ptr.hpp>

#include "Engine/GL/resource.hpp"
#include "Engine/GL/Shader.h"
#include "Engine/GL/uniform.hpp"
#include "Engine/prelude.hpp"

namespace VCX::Engine::GL {
    struct ProgramTrait {
        static auto constexpr & Create = glCreateProgram;
        static auto constexpr & Delete = glDeleteProgram;
        static auto constexpr & Bind   = glUseProgram;
    };

    class UniqueProgram : public Unique<ProgramTrait> {
    public:
        UniqueProgram(std::initializer_list<SharedShader> && shaders):
            Unique(CreateProgramFromShaders(shaders)),
            _uniforms(Get()) {
        }

        UniformCollection & GetUniforms() { return _uniforms; }

        void BindUniformBlock(char const * const name, std::uint32_t const bindingPoint) const;

    private:
        static GLuint CreateProgramFromShaders(std::initializer_list<SharedShader> const &);

        UniformCollection _uniforms;
    };
} // namespace VCX::Engine::GL
