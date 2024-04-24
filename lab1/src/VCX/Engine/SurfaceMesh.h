#pragma once

#include <vector>
#include <glm/glm.hpp>

namespace VCX::Engine {
    class SurfaceMesh {
    public:
        std::vector<glm::vec3>     Positions;
        std::vector<glm::vec3>     Normals;
        std::vector<glm::vec2>     TexCoords;
        std::vector<std::uint32_t> Indices;

        std::size_t GetVertexCount() const { return Positions.size(); }
        bool IsNormalAvailable() const { return Normals.size() == Positions.size(); }
        bool IsTexCoordAvailable() const { return TexCoords.size() == Positions.size(); }

        std::vector<glm::vec3> ComputeNormals() const;
        std::vector<glm::vec2> GetEmptyTexCoords() const;
        std::vector<glm::vec3> ComputeTangents() const;

        std::pair<glm::vec3, glm::vec3> GetAxisAlignedBoundingBox() const;

        void NormalizePositions(glm::vec3 const & minAABB = glm::vec3(-0.5f), glm::vec3 const & maxAABB = glm::vec3(0.5f));
    };
}
