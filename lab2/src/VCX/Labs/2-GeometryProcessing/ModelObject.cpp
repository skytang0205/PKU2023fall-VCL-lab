#include "Labs/2-GeometryProcessing/ModelObject.h"

namespace VCX::Labs::GeometryProcessing {
    void ModelObject::ReplaceMesh(Engine::SurfaceMesh const & mesh) {
        auto layout = Engine::GL::VertexLayout()
            .Add<glm::vec3>("position", Engine::GL::DrawFrequency::Static, 0)
            .Add<glm::vec3>("normal", Engine::GL::DrawFrequency::Static, 1);

        if (mesh.IsTexCoordAvailable())
            layout = std::move(layout).Add<glm::vec2>("texcoord", Engine::GL::DrawFrequency::Static, 2);
            
        Engine::GL::UniqueIndexedRenderItem item(layout, Engine::GL::PrimitiveType::Triangles);
        item.UpdateVertexBuffer("position", Engine::make_span_bytes<glm::vec3>(mesh.Positions));
        item.UpdateVertexBuffer("normal", Engine::make_span_bytes<glm::vec3>(mesh.IsNormalAvailable() ? mesh.Normals : mesh.ComputeNormals()));
        
        if (mesh.IsTexCoordAvailable())
            item.UpdateVertexBuffer("texcoord", Engine::make_span_bytes<glm::vec2>(mesh.TexCoords));
        item.UpdateElementBuffer(mesh.Indices);

        _item.emplace(std::move(item));
        _texCoordAvailable = mesh.IsTexCoordAvailable();
    }

    void ModelObject::Draw(std::initializer_list<Engine::GL::scope_t> && scopes) {
        if (_item.has_value()) _item->Draw({});
    }
}
