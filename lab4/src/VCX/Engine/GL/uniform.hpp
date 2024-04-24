#pragma once

namespace VCX::Engine::GL {
    template<typename T>
    struct UniformTrait {
        static void Set(int, T const &) = delete;
    };

    template<typename T, void (*&Func)(GLint, T)>
    struct ScalarUniformTrait {
        static void Set(int const loc, T const & o) {
            Func(loc, o);
        }
    };

    template<
        typename T,
        typename Element,
        void (*&Func)(GLint, GLsizei, Element const *)>
    struct VectorUniformTrait {
        static void Set(int const loc, T const & o) {
            Func(loc, 1, glm::value_ptr(o));
        }

        template<std::size_t N>
        static void Set(int const loc, std::array<T, N> const & o) {
            Func(loc, N, glm::value_ptr(o[0]));
        }
    };

    template<
        typename T,
        typename Element,
        void (*&Func)(GLint, GLsizei, GLboolean, Element const *)>
    struct MatrixUniformTrait {
        static void Set(int const loc, T const & o) {
            Func(loc, 1, GL_FALSE, glm::value_ptr(o));
        }

        template<std::size_t N>
        static void Set(int const loc, std::array<T, N> const & o) {
            Func(loc, N, GL_FALSE, glm::value_ptr(o[0]));
        }
    };

#define DECLARE_SCALAR_UNIFORM_TRAIT(type, func) \
    template<>                                   \
    struct UniformTrait<type> : public ScalarUniformTrait<type, func> {};
#define DECLARE_VECTOR_UNIFORM_TRAIT(type, element, func) \
    template<>                                            \
    struct UniformTrait<type> : public VectorUniformTrait<type, element, func> {};
#define DECLARE_MATRIX_UNIFORM_TRAIT(type, element, func) \
    template<>                                            \
    struct UniformTrait<type> : public MatrixUniformTrait<type, element, func> {};

    // clang-format off
    DECLARE_SCALAR_UNIFORM_TRAIT(int       , glUniform1i);
    DECLARE_SCALAR_UNIFORM_TRAIT(float     , glUniform1f);
    DECLARE_VECTOR_UNIFORM_TRAIT(glm::vec2 , float, glUniform2fv);
    DECLARE_VECTOR_UNIFORM_TRAIT(glm::vec3 , float, glUniform3fv);
    DECLARE_VECTOR_UNIFORM_TRAIT(glm::vec4 , float, glUniform4fv);
    DECLARE_VECTOR_UNIFORM_TRAIT(glm::ivec2, int  , glUniform2iv);
    DECLARE_VECTOR_UNIFORM_TRAIT(glm::ivec3, int  , glUniform3iv);
    DECLARE_VECTOR_UNIFORM_TRAIT(glm::ivec4, int  , glUniform4iv);
    DECLARE_MATRIX_UNIFORM_TRAIT(glm::mat2 , float, glUniformMatrix2fv);
    DECLARE_MATRIX_UNIFORM_TRAIT(glm::mat3 , float, glUniformMatrix3fv);
    DECLARE_MATRIX_UNIFORM_TRAIT(glm::mat4 , float, glUniformMatrix4fv);
    // clang-format on

#undef DECLARE_SCALAR_UNIFORM_TRAIT
#undef DECLARE_VECTOR_UNIFORM_TRAIT
#undef DECLARE_MATRIX_UNIFORM_TRAIT

    class UniformCollection {
        friend class UniqueProgram;

    private:
        UniformCollection(GLuint const program):
            _program(program) {
        }

    public:

        template<typename T>
        void SetByName(
            char const * const name,
            T const &          value) {
            glUseProgram(_program);
            UniformTrait<T>::Set(GetLocationByName(name), value);
            glUseProgram(0);
        }

        template<typename T, std::size_t N>
        void SetByName(
            char const * const name,
            std::array<T, N> const &          value) {
            glUseProgram(_program);
            UniformTrait<T>::template Set<N>(GetLocationByName(name), value);
            glUseProgram(0);
        }

        template<typename T>
        void SetByLocation(
            int       location,
            T const & value) {
            glUseProgram(_program);
            if (location >= 0)
                UniformTrait<T>::Set(location, value);
            glUseProgram(0);
        }

        template<typename T, std::size_t N>
        void SetByLocation(
            int       location,
            std::array<T, N> const &          value) {
            glUseProgram(_program);
            UniformTrait<T>::template Set<N>(location, value);
            glUseProgram(0);
        }

    private:
        GLuint                                 _program;
        std::unordered_map<std::string, GLint> _locations;

        GLint GetLocationByName(char const * const name) {
            if (auto const iter { _locations.find(name) };
                iter != _locations.cend())
                return iter->second;
            if (auto const loc { glGetUniformLocation(_program, name) };
                loc >= 0) {
                _locations[name] = loc;
                return loc;
            }
            return -1;
        }
    };
} // namespace VCX::Engine::GL
