#pragma once

#include <optional>

#include "Engine/GL/VertexLayout.hpp"
#include "Engine/prelude.hpp"

namespace VCX::Engine::GL {
    enum class PrimitiveType : GLenum {
        Points                 = GL_POINTS,
        Lines                  = GL_LINES,
        LineLoop               = GL_LINE_LOOP,
        LineStrip              = GL_LINE_STRIP,
        Triangles              = GL_TRIANGLES,
        TriangleStrip          = GL_TRIANGLE_STRIP,
        TriangleFan            = GL_TRIANGLE_FAN,
        LinesAdjacency         = GL_LINES_ADJACENCY,
        LineStripAdjacency     = GL_LINE_STRIP_ADJACENCY,
        TrianglesAdjacency     = GL_TRIANGLES_ADJACENCY,
        TriangleStripAdjacency = GL_TRIANGLE_STRIP_ADJACENCY,
    };

    class UniqueRenderItem {
    public:
        UniqueRenderItem(
            VertexLayout  const & layout,
            PrimitiveType const   primitiveType = PrimitiveType::Triangles);

        void UpdateVertexBuffer(char const * const name, std::span<std::byte const> const & data);

        void Draw(
            std::initializer_list<scope_t>       && scopes,
            std::size_t                    const    count         = 0,
            int                            const    first         = 0,
            int                            const    instanceCount = 1) const;

    protected:
        VertexLayout                   _layout;
        GLenum                         _mode;

        UniqueVertexArray              _vao;
        std::vector<UniqueArrayBuffer> _vbos;

        std::size_t                    _vtxCount = 0;
    };

    class UniqueIndexedRenderItem : protected UniqueRenderItem {
    public:
        UniqueIndexedRenderItem(
            VertexLayout  const & layout,
            PrimitiveType const   primitiveType = PrimitiveType::Triangles);
        
        using UniqueRenderItem::UpdateVertexBuffer;

        void UpdateElementBuffer(std::span<std::uint32_t const> const & data);
        void Draw(
            std::initializer_list<scope_t>       && scopes,
            std::size_t                    const    count         = 0,
            int                            const    baseVertex    = 0,
            int                            const    instanceCount = 1) const;

    private:
        UniqueElementArrayBuffer _ebo;
        std::size_t              _idxCount;
    };
} // namespace VCX::Engine::GL
