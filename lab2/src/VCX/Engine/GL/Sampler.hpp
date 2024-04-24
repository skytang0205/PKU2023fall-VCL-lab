#pragma once

#include "Engine/GL/resource.hpp"

namespace VCX::Engine::GL {
    struct SamplerTrait {
        static auto constexpr & CreateMany = glGenSamplers;
        static auto constexpr & DeleteMany = glDeleteSamplers;
    };
    
    enum class WrapMode : GLenum {
        Clamp  = GL_CLAMP_TO_EDGE,
        Repeat = GL_REPEAT,
    };

    enum class FilterMode : GLenum {
        Nearest   = GL_NEAREST,
        Linear    = GL_LINEAR,
        Bilinear  = GL_LINEAR_MIPMAP_NEAREST,
        Trilinear = GL_LINEAR_MIPMAP_LINEAR,
    };

    struct SamplerOptions {
        WrapMode   WrapU     = WrapMode::Clamp;
        WrapMode   WrapV     = WrapMode::Clamp;
        WrapMode   WrapW     = WrapMode::Clamp;
        FilterMode MinFilter;
        FilterMode MagFilter;
    };

    class UniqueSampler : public Unique<SamplerTrait> {
    public:
        explicit UniqueSampler(std::uint32_t const unit = 0) : _unit(unit) { }
        
        explicit UniqueSampler(SamplerOptions && options, std::uint32_t const unit = 0) :
            _unit(unit) {
            Update(std::forward<SamplerOptions>(options));
        }

        scope_t Use() const {
            glBindSampler(_unit, Get());
            return scope_t([=]() {
                glBindSampler(_unit, 0);
            });
        }

        void Update(SamplerOptions && options) const {
            auto const useThis { Use() };
            glSamplerParameteri(Get(), GL_TEXTURE_WRAP_S, GLenum(options.WrapU));
            glSamplerParameteri(Get(), GL_TEXTURE_WRAP_T, GLenum(options.WrapV));
            glSamplerParameteri(Get(), GL_TEXTURE_WRAP_R, GLenum(options.WrapW));
            glSamplerParameteri(Get(), GL_TEXTURE_MIN_FILTER, GLenum(options.MinFilter));
            glSamplerParameteri(Get(), GL_TEXTURE_MAG_FILTER, GLenum(options.MagFilter));
        }

    private:
        std::uint32_t _unit = 0;
    };
}
