#include "Engine/Scene.h"

namespace VCX::Engine {
    std::pair<glm::vec3, glm::vec3> Scene::GetAxisAlignedBoundingBox() const {
        glm::vec3 minAABB(std::numeric_limits<float>::max());
        glm::vec3 maxAABB(std::numeric_limits<float>::min());
        for (const auto & model : Models) {
            const auto modelAABB = model.Mesh.GetAxisAlignedBoundingBox();
            maxAABB = glm::max(maxAABB, modelAABB.second);
            minAABB = glm::min(minAABB, modelAABB.first);
        }
        return { minAABB, maxAABB };
    }
}