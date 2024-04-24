#pragma once

#include <utility>
#include <vector>

#include <glm/glm.hpp>

namespace VCX::Labs::Animation {
    struct MassSpringSystem {
        struct Spring {
            std::pair<std::size_t, std::size_t> AdjIdx;
            float                               RestLength;
        };

        std::vector<glm::vec3>   Positions;
        std::vector<glm::vec3>   Velocities;
        std::vector<int>         Fixed;
        float                    Mass      { 1 };

        std::vector<Spring>      Springs;
        float                    Stiffness { 100 };
        float                    Damping   { .2f };
        float                    Gravity   { .3f };

        void AddParticle(glm::vec3 const & position, glm::vec3 const & velocity = glm::vec3(0)) {
            Positions.push_back(position);
            Velocities.push_back(velocity);
            Fixed.push_back(false);
        }

        void AddSpring(std::size_t const adjIdx0, std::size_t const adjIdx1, float const restLength = -1) {
            Springs.push_back({
                .AdjIdx     { adjIdx0, adjIdx1 },
                .RestLength { restLength < 0 ? glm::length(Positions[adjIdx0] - Positions[adjIdx1]) : restLength } 
            });
        }
    };
}
