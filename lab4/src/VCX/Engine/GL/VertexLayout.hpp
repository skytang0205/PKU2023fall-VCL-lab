#pragma once

#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "Engine/GL/resource.hpp"
#include "Engine/type.hpp"

namespace VCX::Engine::GL {
    template<typename T> inline constexpr GLenum TypeEnumOf = 0;
    template<> inline constexpr GLenum TypeEnumOf<char>           = GL_BYTE;
    template<> inline constexpr GLenum TypeEnumOf<unsigned char>  = GL_UNSIGNED_BYTE;
    template<> inline constexpr GLenum TypeEnumOf<short>          = GL_SHORT;
    template<> inline constexpr GLenum TypeEnumOf<unsigned short> = GL_UNSIGNED_SHORT;
    template<> inline constexpr GLenum TypeEnumOf<int>            = GL_INT;
    template<> inline constexpr GLenum TypeEnumOf<unsigned int>   = GL_UNSIGNED_INT;
    template<> inline constexpr GLenum TypeEnumOf<float>          = GL_FLOAT;
    template<> inline constexpr GLenum TypeEnumOf<double>         = GL_DOUBLE;

	struct VertexAttrib {
        GLuint  Location;
        GLuint  Offset;
        GLenum  Type;
        GLuint  Size;
        bool    Normalized;
	};

	struct VertexAttribBlock {
		DrawFrequency             Frequency;
		std::size_t               Stride;
		std::vector<VertexAttrib> Attributes;
	};

    class VertexLayout {
	public:
		std::vector<VertexAttribBlock> AttribBlocks;

		template<typename T>
        VertexLayout Add(
			char const *     const name,
			DrawFrequency    const frequency,
			std::uint32_t    const location,
			bool             const normalized = false) && {
			_indices[name] = AttribBlocks.size();
			AttribBlocks.push_back({
				.Frequency  = frequency,
				.Stride     = sizeof(T),
				.Attributes = {
					VertexAttrib {
						.Location   = location,
						.Offset     = 0,
						.Type       = TypeEnumOf<glm_type_of<T>>,
						.Size       = glm_size_of_v<T>,
						.Normalized = normalized,
					},
				},
			});

			return std::move(*this);
		}

		template<typename T>
        VertexLayout Add(
			char const *  const name,
			DrawFrequency const frequency) && {
			_indices[name] = AttribBlocks.size();
			AttribBlocks.push_back({
				.Frequency = frequency,
				.Stride    = sizeof(T),
			});

			return std::move(*this);
		}

		template<typename T, typename TField>
        VertexLayout At(
			std::uint32_t const location,
		    TField T::*   const field,
			bool          const normalized = false) && {
            AttribBlocks.back().Attributes.push_back({
                .Location   = location,
                .Offset     = GLuint(std::intptr_t(&((*(T *) 0).*field))),
                .Type       = TypeEnumOf<glm_type_of<TField>>,
                .Size       = glm_size_of_v<TField>,
                .Normalized = normalized,
            });

			return std::move(*this);
		}

		std::size_t GetIndexByName(char const * const name) const {
			if (auto const iter = _indices.find(name); iter != _indices.end())
				return iter->second;
			throw std::logic_error("undefined VertexAttribBlock name.");
		}

	private:
		std::unordered_map<std::string, std::size_t> _indices;
    };

} // namespace VCX::Engine::GL
