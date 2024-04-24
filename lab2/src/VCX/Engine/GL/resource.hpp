#pragma once

#include <concepts>
#include <functional>
#include <memory>

#include <glad/glad.h>

#include "scope.hpp"

#define gl_using(var) \
    auto const __use_##var { (var).Use() }

namespace VCX::Engine::GL {
    using scope_t = scope::scope_exit<std::function<void ()>>;

    enum class DrawFrequency : GLenum {
        Stream  = GL_STREAM_DRAW,
        Static  = GL_STATIC_DRAW,
        Dynamic = GL_DYNAMIC_DRAW,
    };

    // clang-format off
    template<typename T> concept ResourceCreate = requires {
        { T::Create() } -> std::same_as<GLuint>;
    };
    template<typename T> concept ResourceCreateMany = requires (GLsizei n, GLuint * ptr) {
        { T::CreateMany(n, ptr) } -> std::same_as<void>;
    };
    template<typename T> concept ResourceDelete = requires (GLuint o) {
        { T::Delete(o) } -> std::same_as<void>;
    };
    template<typename T> concept ResourceDeleteMany = requires (GLsizei n, GLuint * ptr) {
        { T::DeleteMany(n, ptr) } -> std::same_as<void>;
    };
    template<typename T> concept ResourceBind = requires (GLuint o) {
        { T::Bind(o) } -> std::same_as<void>;
    };
    template<typename T> concept ResourceBindToTarget = requires (GLenum target, GLuint o) {
        std::is_same_v<std::remove_const_t<decltype(T::BindTarget)>, GLenum>;
        { T::Bind(target, o) } -> std::same_as<void>;
    };

    template<typename T> concept ResourceTrait = (ResourceDelete<T> || ResourceDeleteMany<T>);
    // clang-format on

    template<ResourceTrait Trait>
    struct ResourceMethods {
        static GLuint Create() {
            if constexpr (ResourceCreate<Trait>)
                return Trait::Create();
            if constexpr (ResourceCreateMany<Trait>) {
                GLuint o { 0 };
                Trait::CreateMany(1, &o);
                return o;
            }
        }

        static void Delete(GLuint const o) {
            if constexpr (ResourceDelete<Trait>)
                return Trait::Delete(o);
            if constexpr (ResourceDeleteMany<Trait>)
                return Trait::DeleteMany(1, &o);
        }

        static void Bind(GLuint const o) {
            if constexpr (ResourceBind<Trait>)
                Trait::Bind(o);
            if constexpr (ResourceBindToTarget<Trait>)
                Trait::Bind(Trait::BindTarget, o);
        }

        static void Unbind() {
            if constexpr (ResourceBind<Trait>)
                Trait::Bind(0);
            if constexpr (ResourceBindToTarget<Trait>)
                Trait::Bind(Trait::BindTarget, 0);
        }
    };

    template<ResourceTrait Trait>
    class Unique {
    private:
        scope::unique_resource<GLuint, void (*)(GLuint)> _o;

    public:
        Unique()
            requires (ResourceCreate<Trait> || ResourceCreateMany<Trait>) :
            _o(scope::make_unique_resource(
                ResourceMethods<Trait>::Create(),
                ResourceMethods<Trait>::Delete)) {}

        explicit Unique(GLuint const o):
            _o(scope::make_unique_resource(
                o,
                ResourceMethods<Trait>::Delete)) {}

        GLuint Get() const { return _o.get(); }

        scope_t Use() const
            requires (ResourceBind<Trait> || ResourceBindToTarget<Trait>) {
            ResourceMethods<Trait>::Bind(_o.get());
            return scope_t(ResourceMethods<Trait>::Unbind);
        }
    };

    template<ResourceTrait Trait>
    class Shared {
    private:
        std::shared_ptr<scope::unique_resource<GLuint, void (*)(GLuint)>> _o;

    public:
        Shared()
            requires (ResourceCreate<Trait> || ResourceCreateMany<Trait>) :
            _o(std::make_shared<scope::unique_resource<GLuint, void (*)(GLuint)>>(scope::make_unique_resource(
                ResourceMethods<Trait>::Create(),
                ResourceMethods<Trait>::Delete))) {}

        explicit Shared(GLuint const o) :
            _o(std::make_shared<scope::unique_resource<GLuint, void (*)(GLuint)>>(scope::make_unique_resource(
                o,
                ResourceMethods<Trait>::Delete))) {}

        GLuint Get() const { return _o->get(); }
            
        scope_t Use() const
            requires (ResourceBind<Trait> || ResourceBindToTarget<Trait>) {
            ResourceMethods<Trait>::Bind(_o.get());
            return scope_t(ResourceMethods<Trait>::Unbind);
        }
    };

    struct VertexArrayTrait {
        static auto constexpr & CreateMany = glGenVertexArrays;
        static auto constexpr & DeleteMany = glDeleteVertexArrays;
        static auto constexpr & Bind       = glBindVertexArray;
    };

    struct ArrayBufferTrait {
        static auto constexpr & CreateMany = glGenBuffers;
        static auto constexpr & DeleteMany = glDeleteBuffers;
        static auto constexpr & Bind       = glBindBuffer;
        static GLenum constexpr BindTarget = GL_ARRAY_BUFFER;
    };

    struct ElementArrayBufferTrait {
        static auto constexpr & CreateMany = glGenBuffers;
        static auto constexpr & DeleteMany = glDeleteBuffers;
        static auto constexpr & Bind       = glBindBuffer;
        static GLenum constexpr BindTarget = GL_ELEMENT_ARRAY_BUFFER;
    };

    struct FramebufferTrait {
        static auto constexpr & CreateMany = glGenFramebuffers;
        static auto constexpr & DeleteMany = glDeleteFramebuffers;
        static auto constexpr & Bind       = glBindFramebuffer;
        static GLenum constexpr BindTarget = GL_FRAMEBUFFER;
    };

    struct RenderbufferTrait {
        static auto constexpr & CreateMany = glGenRenderbuffers;
        static auto constexpr & DeleteMany = glDeleteRenderbuffers;
        static auto constexpr & Bind       = glBindRenderbuffer;
        static GLenum constexpr BindTarget = GL_RENDERBUFFER;
    };

    struct UniformBufferTrait {
        static auto constexpr & CreateMany = glGenBuffers;
        static auto constexpr & DeleteMany = glDeleteBuffers;
        static auto constexpr & Bind       = glBindBuffer;
        static GLenum constexpr BindTarget = GL_UNIFORM_BUFFER;
    };

    struct Texture2DMultisample {
        static auto constexpr & CreateMany = glGenTextures;
        static auto constexpr & DeleteMany = glDeleteTextures;
        static auto constexpr & Bind       = glBindTexture;
        static GLenum constexpr BindTarget = GL_TEXTURE_2D_MULTISAMPLE;
    };

    // clang-format on

    using UniqueVertexArray          = Unique<VertexArrayTrait>;
    using UniqueArrayBuffer          = Unique<ArrayBufferTrait>;
    using UniqueElementArrayBuffer   = Unique<ElementArrayBufferTrait>;
    using UniqueFramebuffer          = Unique<FramebufferTrait>;
    using UniqueRenderbuffer         = Unique<RenderbufferTrait>;
    using UniqueUniformBuffer        = Unique<UniformBufferTrait>;
    using UniqueTexture2DMultiSample = Unique<Texture2DMultisample>;
} // namespace VCX::Engine::GL
