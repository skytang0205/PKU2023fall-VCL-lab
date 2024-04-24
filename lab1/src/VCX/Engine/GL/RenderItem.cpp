#include "Engine/GL/RenderItem.h"

#include <spdlog/spdlog.h>

namespace VCX::Engine::GL {

    UniqueRenderItem::UniqueRenderItem(
        VertexLayout  const & layout,
        PrimitiveType const   primitiveType) :
        _layout(layout),
        _mode(GLenum(primitiveType)) {
        gl_using(_vao);
        for (auto const & attrBlock : _layout.AttribBlocks) {
            _vbos.emplace_back();
            auto const useVbo { _vbos.back().Use() };
            for (auto const & attr : attrBlock.Attributes) {
                glVertexAttribPointer(
                attr.Location, attr.Size, attr.Type, attr.Normalized, attrBlock.Stride,
                    reinterpret_cast<void *>(std::uintptr_t(attr.Offset)));
                glEnableVertexAttribArray(attr.Location);
            }
        }
    }

    void UniqueRenderItem::UpdateVertexBuffer(char const * const name, std::span<std::byte const> const & data) {
        auto const idx = _layout.GetIndexByName(name);
        auto const useVbo { _vbos[idx].Use() };
        glBufferData(GL_ARRAY_BUFFER, data.size(), data.data(), GLenum(_layout.AttribBlocks[idx].Frequency));
        _vtxCount = data.size() / _layout.AttribBlocks[idx].Stride;
    }

    void UniqueRenderItem::Draw(
        std::initializer_list<scope_t>       && scopes,
        std::size_t                    const    count,
        int                            const    first,
        int                            const    instanceCount) const {
        gl_using(_vao);
        glDrawArraysInstanced(_mode, first, count ? count : _vtxCount, instanceCount);
    }

    UniqueIndexedRenderItem::UniqueIndexedRenderItem(
        VertexLayout  const & layout,
        PrimitiveType const   primitiveType) :
        UniqueRenderItem(layout, primitiveType) {
        glBindVertexArray(_vao.Get());
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _ebo.Get());
        glBindVertexArray(0);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    }

    void UniqueIndexedRenderItem::UpdateElementBuffer(std::span<std::uint32_t const> const & data) {
        gl_using(_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, data.size_bytes(), data.data(), GL_STATIC_DRAW);
        _idxCount = data.size();
    }

    void UniqueIndexedRenderItem::Draw(
        std::initializer_list<scope_t>       && scopes,
        std::size_t                    const    count ,
        int                            const    baseVertex,
        int                            const    instanceCount) const {
        gl_using(_vao);
        glDrawElementsInstancedBaseVertex(_mode, count ? count : _idxCount, GL_UNSIGNED_INT, nullptr, instanceCount, baseVertex);
    }
}
