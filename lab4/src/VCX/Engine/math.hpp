#pragma once

#include <algorithm>

#include <glm/glm.hpp>

namespace VCX::Engine {
    struct Spherical {
        float Radius = 1.f;
        float Phi    = 0.f;
        float Theta  = 0.f;

        Spherical() {}

        Spherical(glm::vec3 const & v):
            Radius(glm::length(v)),
            Phi(Radius == 0.f ? 0.f : glm::acos(glm::clamp(v.y / Radius, -1.f, 1.f))),
            Theta(Radius == 0.f ? 0.f : glm::atan(v.x, v.z)) {
        }

        void MakeSafe() {
            Phi = std::max(glm::epsilon<float>(), std::min(glm::pi<float>() - glm::epsilon<float>(), Phi));
        }

        glm::vec3 Vec() const {
            const float sinPhiRadius = glm::sin(Phi) * Radius;
            return glm::vec3(sinPhiRadius * glm::sin(Theta), glm::cos(Phi) * Radius, sinPhiRadius * glm::cos(Theta));
        }
    };
}
