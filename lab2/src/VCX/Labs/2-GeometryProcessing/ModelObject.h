#pragma once

#include <optional>

#include <glm/ext.hpp>

#include "Engine/GL/RenderItem.h"
#include "Engine/SurfaceMesh.h"

namespace VCX::Labs::GeometryProcessing {
    class ModelObject {
    public:
        ModelObject() { }
        ModelObject(Engine::SurfaceMesh const & mesh) { ReplaceMesh(mesh); }

        void ReplaceMesh(Engine::SurfaceMesh const & mesh);
        void Draw(std::initializer_list<Engine::GL::scope_t> && scopes);

        bool        IsTexCoordAvailable() const { return _texCoordAvailable; }
        glm::mat4   GetTransform() const { return glm::translate(glm::mat4(1.f), _translate) * _rotate * glm::scale(glm::mat4(1.f), _scale); }
        void        Rotate(float const deg, glm::vec3 const & axis) { _rotate = glm::rotate(glm::mat4(1.f), glm::radians(deg), axis) * _rotate; }
        void        RotateX(float const deg) { _rotate = glm::rotate(glm::mat4(1.f), glm::radians(deg), glm::vec3(1.f, 0.f, 0.f)) * _rotate; }
        void        RotateY(float const deg) { _rotate = glm::rotate(glm::mat4(1.f), glm::radians(deg), glm::vec3(0.f, 1.f, 0.f)) * _rotate; }
        void        RotateZ(float const deg) { _rotate = glm::rotate(glm::mat4(1.f), glm::radians(deg), glm::vec3(0.f, 0.f, 1.f)) * _rotate; }
        void        Translate(glm::vec3 const & offset) { _translate += offset; }
        void        Scale(glm::vec3 const & scale) { _scale *= scale; }

    private:

        std::optional<Engine::GL::UniqueIndexedRenderItem> _item;
        bool                                               _texCoordAvailable { false };
        glm::mat4                                          _rotate            { 1.f };
        glm::vec3                                          _translate         { 0.f };
        glm::vec3                                          _scale             { 1.f };
    };
}
