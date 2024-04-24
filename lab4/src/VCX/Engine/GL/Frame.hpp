#pragma once

#include <optional>

#include <spdlog/spdlog.h>

#include "Engine/GL/resource.hpp"
#include "Engine/GL/Texture.hpp"
#include "Engine/prelude.hpp"

namespace VCX::Engine::GL {
    template<typename TextureType, typename DepthStencilFormat, TextureFormat... TexFormats>
        requires requires {
            std::is_same_v<TextureType, UniqueTexture2D> || std::is_same_v<TextureType, UniqueTextureCubeMap>;
            std::is_same_v<DepthStencilFormat, void> || std::is_same_v<DepthStencilFormat, Formats::D24S8> || std::is_same_v<DepthStencilFormat, Formats::D32>;
        }
    class UniqueFrame {
    public:
        UniqueFrame() {
            { // Non-multisample frame buffer.
                gl_using(_fbo);
                for (std::size_t i = 0; i < sizeof...(TexFormats); ++i) {
                    _colorAttachments[i].UpdateSampler({ .MinFilter = FilterMode::Linear, .MagFilter = FilterMode::Nearest });
                    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, _colorAttachments[i].Get(), 0);
                }
                if constexpr (! std::is_same_v<DepthStencilFormat, void>) {
                    _depthStencilAttachment.UpdateSampler(SamplerOptions{ .MinFilter = FilterMode::Linear, .MagFilter = FilterMode::Nearest });
                    if constexpr (std::is_same_v<DepthStencilFormat, Formats::D32>) {
                        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, _depthStencilAttachment.Get(), 0);
                    } else if constexpr (std::is_same_v<DepthStencilFormat, Formats::D24S8>) {
                        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, _depthStencilAttachment.Get(), 0);
                    }
                }
                if constexpr (! sizeof...(TexFormats)) {
                    glDrawBuffer(GL_NONE);
                    glReadBuffer(GL_NONE);
                }
            }
            if constexpr (std::is_same_v<TextureType, UniqueTexture2D>) { // Multisample frame buffer.
                gl_using(_fboMS);
                for (std::size_t i = 0; i < sizeof...(TexFormats); ++i) {
                    auto const useTexMS { _colorAttachmentsMS[i].Use() };
                    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0 + i, _colorAttachmentsMS[i].Get(), 0);
                }
                if constexpr (! std::is_same_v<DepthStencilFormat, void>) {
                    auto const useTexMS { _depthStencilAttachmentMS.Use() };
                    if constexpr (std::is_same_v<DepthStencilFormat, Formats::D32>) {
                        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, _depthStencilAttachmentMS.Get(), 0);
                    } else if constexpr (std::is_same_v<DepthStencilFormat, Formats::D24S8>) {
                        glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, _depthStencilAttachmentMS.Get(), 0);
                    }
                }
                if constexpr (! sizeof...(TexFormats)) {
                    glDrawBuffer(GL_NONE);
                    glReadBuffer(GL_NONE);
                }
            }
        }

        void Resize(std::pair<std::uint32_t, std::uint32_t> const size, std::size_t const samples = 1)
            requires (! std::is_same_v<TextureType, UniqueTexture2D>) {
            auto const [width, height] = size;
            if (_size != size) { // Default frame buffer.
                if constexpr (sizeof...(TexFormats)) ResizeColorAttachments<TexFormats...>(size);
                if constexpr (std::is_same_v<DepthStencilFormat, Formats::D32> || std::is_same_v<DepthStencilFormat, Formats::D24S8>)
                    _depthStencilAttachment.template Resize<DepthStencilFormat>(width, height);
            }
            _size = size;
        }

        void Resize(std::pair<std::uint32_t, std::uint32_t> const size, std::size_t const samples = 1)
            requires std::is_same_v<TextureType, UniqueTexture2D> {
            auto const [width, height] = size;
            if (_size != size) { // Default frame buffer.
                if constexpr (sizeof...(TexFormats)) ResizeColorAttachments<TexFormats...>(size);
                if constexpr (std::is_same_v<DepthStencilFormat, Formats::D32> || std::is_same_v<DepthStencilFormat, Formats::D24S8>)
                    _depthStencilAttachment.template Resize<DepthStencilFormat>(width, height);
            }
            if (_size != size || _samples != samples) { // Multisample frame buffer.
                if constexpr (sizeof...(TexFormats)) ResizeColorAttachmentsMS<TexFormats...>(size, samples);
                if constexpr (std::is_same_v<DepthStencilFormat, Formats::D32> || std::is_same_v<DepthStencilFormat, Formats::D24S8>) {
                    auto const useTexMS { _depthStencilAttachmentMS.Use() };
                    glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, FormatEnumOf<DepthStencilFormat>, width, height, GL_TRUE);
                }
            }
            _size    = size;
            _samples = samples;
        }
        
        scope_t Use() const {
            glBindFramebuffer(GL_FRAMEBUFFER, _samples == 1 ? _fbo.Get() : _fboMS.Get());
            glViewport(0, 0, _size.first, _size.second);
            glClearColor(0, 0, 0, 1);
            glClear(GetBufferBits());
            if (_samples == 1) {
                return scope_t([](){
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                });
            } else {
                return scope_t([=](){
                    glBindFramebuffer(GL_FRAMEBUFFER, 0);
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, _fboMS.Get());
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, _fbo.Get());
                    if constexpr (GetBufferBits() & ~GL_COLOR_BUFFER_BIT) {
                        glReadBuffer(GL_NONE);
                        glDrawBuffer(GL_NONE);
                        glBlitFramebuffer(0, 0, _size.first, _size.second, 0, 0, _size.first, _size.second, GetBufferBits() & ~GL_COLOR_BUFFER_BIT, GL_NEAREST);
                    }
                    for (std::size_t i = sizeof...(TexFormats); i > 0; --i) {
                        glReadBuffer(GL_COLOR_ATTACHMENT0 + i - 1);
                        glDrawBuffer(GL_COLOR_ATTACHMENT0 + i - 1);
                        glBlitFramebuffer(0, 0, _size.first, _size.second, 0, 0, _size.first, _size.second, GL_COLOR_BUFFER_BIT, GL_NEAREST);
                    }
                    glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);
                    glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
                });
            }
        }

        template<std::size_t Idx = 0>
            requires (Idx < sizeof...(TexFormats))
        TextureType & GetColorAttachment() {
            return _colorAttachments[Idx];
        }

        TextureType & GetDepthStencilAttachment()
            requires (! std::is_same_v<DepthStencilFormat, void>) {
            return _depthStencilAttachment;
        }

        std::pair<std::uint32_t, std::uint32_t> GetSize() const { return _size; }

    private:
        template<TextureFormat First, TextureFormat... Args>
        void ResizeColorAttachments(std::pair<std::uint32_t, std::uint32_t> const size) {
            _colorAttachments[sizeof...(TexFormats) - sizeof...(Args) - 1].template Resize<First>(size.first, size.second);
            if constexpr (sizeof...(Args)) ResizeColorAttachments<Args...>(size);
        }

        template<TextureFormat First, TextureFormat... Args>
        void ResizeColorAttachmentsMS(std::pair<std::uint32_t, std::uint32_t> const size, std::size_t const samples)
            requires std::is_same_v<TextureType, UniqueTexture2D> {
            {
                auto const useTexMS { _colorAttachmentsMS[sizeof...(TexFormats) - sizeof...(Args) - 1].Use() };
                glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, samples, FormatEnumOf<First>, size.first, size.second, GL_TRUE);
            }
            if constexpr (sizeof...(Args)) ResizeColorAttachmentsMS<Args...>(size, samples);
        }

        static constexpr GLenum GetBufferBits() {
            GLenum bits = 0;
            if constexpr (sizeof...(TexFormats))
                bits |= GL_COLOR_BUFFER_BIT;
            if constexpr (std::is_same_v<DepthStencilFormat, Formats::D32>)
                bits |= GL_DEPTH_BUFFER_BIT;
            if constexpr (std::is_same_v<DepthStencilFormat, Formats::D24S8>)
                bits |= GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT;
            return bits;
        }

        std::pair<std::uint32_t, std::uint32_t> _size    = { 0, 0 };
        std::size_t                             _samples = 1;

        // Non-multisample variables.
        UniqueFramebuffer                              _fbo;
        std::array<TextureType, sizeof...(TexFormats)> _colorAttachments;
        TextureType                                    _depthStencilAttachment;
 
        // Multisample variables.
        UniqueFramebuffer                                             _fboMS;
        std::array<UniqueTexture2DMultiSample, sizeof...(TexFormats)> _colorAttachmentsMS;
        UniqueTexture2DMultiSample                                    _depthStencilAttachmentMS;
    };

    using UniqueRenderFrame = UniqueFrame<UniqueTexture2D, Formats::D32, Formats::RGBA8>;
    using UniqueDepthFrame = UniqueFrame<UniqueTexture2D, Formats::D32>;
    using UniqueDepthCubeFrame = UniqueFrame<UniqueTextureCubeMap, Formats::D32>;
}
