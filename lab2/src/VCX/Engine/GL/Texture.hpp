#pragma once

#include "Engine/GL/Sampler.hpp"
#include "Engine/TextureND.hpp"

namespace VCX::Engine::GL {
    template<TextureFormat Format> inline constexpr GLenum InternalFormatEnumOf = 0;
    template<> inline constexpr GLenum InternalFormatEnumOf<Formats::R8>    = GL_R8;
    template<> inline constexpr GLenum InternalFormatEnumOf<Formats::RGB8>  = GL_RGB8;
    template<> inline constexpr GLenum InternalFormatEnumOf<Formats::RGBA8> = GL_RGBA8;
    template<> inline constexpr GLenum InternalFormatEnumOf<Formats::R16>   = GL_R16;
    template<> inline constexpr GLenum InternalFormatEnumOf<Formats::D32>   = GL_DEPTH_COMPONENT32;
    template<> inline constexpr GLenum InternalFormatEnumOf<Formats::D24S8> = GL_DEPTH24_STENCIL8;


    template<TextureFormat Format> inline constexpr GLenum FormatEnumOf = 0;
    template<> inline constexpr GLenum FormatEnumOf<Formats::R8>    = GL_RED;
    template<> inline constexpr GLenum FormatEnumOf<Formats::RGB8>  = GL_RGB;
    template<> inline constexpr GLenum FormatEnumOf<Formats::RGBA8> = GL_RGBA;
    template<> inline constexpr GLenum FormatEnumOf<Formats::R16>   = GL_RED;
    template<> inline constexpr GLenum FormatEnumOf<Formats::D32>   = GL_DEPTH_COMPONENT;
    template<> inline constexpr GLenum FormatEnumOf<Formats::D24S8> = GL_DEPTH_STENCIL;

    template<TextureFormat Format> inline constexpr GLenum PixelTypeEnumOf = 0;
    template<> inline constexpr GLenum PixelTypeEnumOf<Formats::R8>    = GL_UNSIGNED_BYTE;
    template<> inline constexpr GLenum PixelTypeEnumOf<Formats::RGB8>  = GL_UNSIGNED_BYTE;
    template<> inline constexpr GLenum PixelTypeEnumOf<Formats::RGBA8> = GL_UNSIGNED_BYTE;
    template<> inline constexpr GLenum PixelTypeEnumOf<Formats::R16>   = GL_UNSIGNED_SHORT;
    template<> inline constexpr GLenum PixelTypeEnumOf<Formats::D32>   = GL_UNSIGNED_INT;
    template<> inline constexpr GLenum PixelTypeEnumOf<Formats::D24S8> = GL_UNSIGNED_INT_24_8;
    
    struct Texture2DTrait {
        static auto constexpr & CreateMany = glGenTextures;
        static auto constexpr & DeleteMany = glDeleteTextures;
        static auto constexpr & Bind       = glBindTexture;
        static GLenum constexpr BindTarget = GL_TEXTURE_2D;
    };

	struct TextureCubeMapTrait {
		static auto constexpr & CreateMany = glGenTextures;
		static auto constexpr & DeleteMany = glDeleteTextures;
        static auto constexpr & Bind       = glBindTexture;
        static GLenum constexpr BindTarget = GL_TEXTURE_CUBE_MAP;
	};

    template<typename TypeTrait>
        requires(TypeTrait::BindTarget == GL_TEXTURE_2D || TypeTrait::BindTarget == GL_TEXTURE_CUBE_MAP)
    class UniqueTexture : public Unique<TypeTrait> {
    public:
        static GLenum constexpr TypeEnum = TypeTrait::BindTarget;

        explicit UniqueTexture(std::uint32_t const unit = 0) : _unit(unit) { }

        explicit UniqueTexture(SamplerOptions && options, std::uint32_t const unit = 0) :
            _unit(unit) {
            UpdateSampler(std::forward<SamplerOptions>(options));
        }

        template<typename Content>
        UniqueTexture(Content const & texture, std::uint32_t const unit = 0) : _unit(unit) {
            Update(texture);
        }

        template<typename Content>
        UniqueTexture(Content const & texture, SamplerOptions && options, std::uint32_t const unit = 0) : _unit(unit) {
            UpdateSampler(std::forward<SamplerOptions>(options));
            Update(texture);
        }

        scope_t Use() const {
            glActiveTexture(GL_TEXTURE0 + _unit);
            glBindTexture(TypeEnum, Unique<TypeTrait>::Get());
            return scope_t([=]() {
                glActiveTexture(GL_TEXTURE0 + _unit);
                glBindTexture(TypeEnum, 0);
            });
        }

        void UpdateSampler(SamplerOptions && options) const {
            auto const useThis { Use() };
            glTexParameteri(TypeEnum, GL_TEXTURE_WRAP_S, GLenum(options.WrapU));
            glTexParameteri(TypeEnum, GL_TEXTURE_WRAP_T, GLenum(options.WrapV));
            glTexParameteri(TypeEnum, GL_TEXTURE_WRAP_R, GLenum(options.WrapW));
            glTexParameteri(TypeEnum, GL_TEXTURE_MIN_FILTER, GLenum(options.MinFilter));
            glTexParameteri(TypeEnum, GL_TEXTURE_MAG_FILTER, GLenum(options.MagFilter));
        }

        template<TextureFormat Format>
        void Resize(std::size_t const width, std::size_t const height) const
            requires std::is_same_v<TypeTrait, Texture2DTrait> {
            auto const useThis { Use() };
            glTexImage2D(
                GL_TEXTURE_2D, 0,
                InternalFormatEnumOf<Format>,
                width, height, 0,
                FormatEnumOf<Format>, PixelTypeEnumOf<Format>,
                nullptr);
        }
        
        template<TextureFormat Format>
        void Resize(std::size_t const width, std::size_t const height) const
            requires std::is_same_v<TypeTrait, TextureCubeMapTrait> {
            auto const useThis { Use() };
            for (std::size_t i = 0; i < 6; ++i) {
                glTexImage2D(
                    GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0,
                    InternalFormatEnumOf<Format>,
                    width, height, 0,
                    FormatEnumOf<Format>, PixelTypeEnumOf<Format>,
                    nullptr);
            }
        }

        template<typename Content>
        void Update(Content const & texture) const {
            auto const useThis { Use() };
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            UpdateImpl(texture);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
            glGenerateMipmap(TypeEnum);
        }

        template<TextureFormat Format>
        auto Download() const {
            auto const useThis { Use() };
            glPixelStorei(GL_PACK_ALIGNMENT, 1);
            auto ret = DownloadImpl<Format>();
            glPixelStorei(GL_PACK_ALIGNMENT, 4);
            return ret;
        }

        void GenerateMipmap() const {
            auto const useThis { Use() };
            glGenerateMipmap(TypeEnum);
        }

        std::uint32_t GetUnit() const { return _unit; }

        void SetUnit(std::uint32_t const unit) { _unit = unit; }
    
    private:
        std::uint32_t _unit = 0;

        template<TextureFormat Format>
        void UpdateImpl(Texture2D<Format> const & texture) const
            requires std::is_same_v<TypeTrait, Texture2DTrait> {
            glTexImage2D(
                GL_TEXTURE_2D, 0,
                InternalFormatEnumOf<Format>,
                texture.GetSizeX(), texture.GetSizeY(), 0,
                FormatEnumOf<Format>, PixelTypeEnumOf<Format>,
                texture.GetBytes().data());
        }

        template<TextureFormat Format>
        void UpdateImpl(std::array<Texture2D<Format>, 6> const & texture) const
            requires std::is_same_v<TypeTrait, TextureCubeMapTrait> {
            for (std::size_t i = 0; i < 6; ++i) {
                glTexImage2D(
                    GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0,
                    InternalFormatEnumOf<Format>,
                    texture[i].GetSizeX(), texture[i].GetSizeY(), 0,
                    FormatEnumOf<Format>, PixelTypeEnumOf<Format>,
                    texture[i].GetBytes().data());
            }
        }

        template<TextureFormat Format>
        Texture2D<Format> DownloadImpl() const
            requires std::is_same_v<TypeTrait, Texture2DTrait> {
            int width;
            int height;
            glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
            glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);
            Texture2D<Format> ret(width, height);
            glGetTexImage(GL_TEXTURE_2D, 0,
                          FormatEnumOf<Format>, PixelTypeEnumOf<Format>,
                          reinterpret_cast<void *>(const_cast<std::byte *>(ret.GetBytes().data())));
            return ret;
        }
    };

    using UniqueTexture2D      = UniqueTexture<Texture2DTrait>;
    using UniqueTextureCubeMap = UniqueTexture<TextureCubeMapTrait>;
}
