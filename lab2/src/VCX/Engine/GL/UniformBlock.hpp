#pragma once

#include "Engine/GL/resource.hpp"

namespace VCX::Engine::GL {
    template<typename T>
    class UniqueUniformBlock : public UniqueUniformBuffer {
    public:
        UniqueUniformBlock(std::uint32_t const bindingPoint, DrawFrequency const frequency) {
            auto const useThis { Use() };
            glBufferData(GL_UNIFORM_BUFFER, sizeof(T), nullptr, GLenum(frequency));
            glBindBufferBase(GL_UNIFORM_BUFFER, bindingPoint, Get());
        }

        void Update(T const & block) const {
            auto const useThis { Use() };
            glBufferSubData(GL_UNIFORM_BUFFER, 0, sizeof(block), &block);
        }

        template<typename TField>
        void Update(TField T::* const field, T const & block) const {
            auto const useThis { Use() };
            glBufferSubData(GL_UNIFORM_BUFFER, GLintptr(&((*(T *) 0).*field)), sizeof(TField), &(block.*field));
        }

        template<typename TField>
        void Update(TField T::* const field, TField const & val) const {
            auto const useThis { Use() };
            glBufferSubData(GL_UNIFORM_BUFFER, GLintptr(&((*(T *) 0).*field)), sizeof(TField), &val);
        }
    };
}
