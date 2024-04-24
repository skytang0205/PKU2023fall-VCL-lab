#include <algorithm>

#include <spdlog/spdlog.h>

#include "Engine/SurfaceMesh.h"

namespace VCX::Engine {
    std::vector<glm::vec3> SurfaceMesh::ComputeNormals() const {
        std::vector<glm::vec3> normals(Positions.size(), glm::vec3(0));
        for (std::size_t i = 0; i + 2 < Indices.size(); i += 3) {
            std::uint32_t const * face   = Indices.data() + i;
            glm::vec3 const &     p1     = Positions[face[0]];
            glm::vec3 const &     p2     = Positions[face[1]];
            glm::vec3 const &     p3     = Positions[face[2]];
            glm::vec3             normal = glm::cross(p2 - p1, p3 - p1);
            normals[face[0]] += normal;
            normals[face[1]] += normal;
            normals[face[2]] += normal;
        }
        for (glm::vec3 & normal : normals)
            normal = glm::normalize(normal);
        return normals;
    }

    std::vector<glm::vec2> SurfaceMesh::GetEmptyTexCoords() const {
        std::vector<glm::vec2> texCoords(Positions.size(), { .5f, .5f });
        return texCoords;
    }

    std::vector<glm::vec3> SurfaceMesh::ComputeTangents() const {
        std::vector<glm::vec3> tangents(Positions.size(), glm::vec3(0));
        if (! IsTexCoordAvailable()) return tangents;
        for (std::size_t i = 0; i + 2 < Indices.size(); i += 3) {
            std::uint32_t const * face = Indices.data() + i;
            glm::vec3 const & pos1 = Positions[face[0]];
            glm::vec3 const & pos2 = Positions[face[1]];
            glm::vec3 const & pos3 = Positions[face[2]];
            glm::vec2 const & uv1  = TexCoords[face[0]]; 
            glm::vec2 const & uv2  = TexCoords[face[1]]; 
            glm::vec2 const & uv3  = TexCoords[face[2]]; 
            glm::vec3 const   edge1    = pos2 - pos1;
            glm::vec3 const   edge2    = pos3 - pos1;
            glm::vec2 const   deltaUV1 = uv2 - uv1;
            glm::vec2 const   deltaUV2 = uv3 - uv1;
            float const f = 1.f / (deltaUV1.x * deltaUV2.y - deltaUV2.x * deltaUV1.y);
            glm::vec3 const tangent = f * (deltaUV2.y * edge1 - deltaUV1.y * edge2);
            tangents[face[0]] += tangent;
            tangents[face[1]] += tangent;
            tangents[face[2]] += tangent;
        }
        for (glm::vec3 & tangent : tangents)
            tangent = glm::normalize(tangent);
        return tangents;
    }

    std::pair<glm::vec3, glm::vec3> SurfaceMesh::GetAxisAlignedBoundingBox() const {
        glm::vec3 minAABB(std::numeric_limits<float>::max());
        glm::vec3 maxAABB(std::numeric_limits<float>::min());
        for (auto const & pos : Positions) {
            maxAABB = glm::max(maxAABB, pos);
            minAABB = glm::min(minAABB, pos);
        }
        return { minAABB, maxAABB };
    }

    void SurfaceMesh::NormalizePositions(glm::vec3 const & minAABB, glm::vec3 const & maxAABB) {
        auto const [currMinAABB, currMaxAABB] = GetAxisAlignedBoundingBox();
        glm::vec3 currCenter   = (currMaxAABB + currMinAABB) * 0.5f;
        glm::vec3 targetCenter = (maxAABB + minAABB) * 0.5f;
        glm::vec3 currScale    = currMaxAABB - currMinAABB;
        glm::vec3 targetScale  = maxAABB - minAABB;

        glm::vec3 relativeScale(1.0f);
        if (Positions.size() > 1 && targetScale.x > 0 && targetScale.y > 0 && targetScale.z > 0) {
            relativeScale = targetScale / currScale;
        }
        float uniformScale = glm::min(glm::min(relativeScale.x, relativeScale.y), relativeScale.z);

        for (auto & pos : Positions) {
            pos = (pos - currCenter) * uniformScale + targetCenter;
        }
    }
}
