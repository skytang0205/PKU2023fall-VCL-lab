#pragma once

#include <glm/ext.hpp>

namespace VCX::Engine {
    struct Camera {
        float     Fovy   { 60.f };
        float     ZNear  { 0.01f };
        float     ZFar   { 100.f };
        glm::vec3 Eye    { 0, 0, 1 };
        glm::vec3 Target { 0, 0, 0 };
        glm::vec3 Up     { 0, 1, 0 };

        glm::mat4 GetProjectionMatrix(float const aspect) const {
            return glm::perspective(glm::radians(Fovy), aspect, ZNear, ZFar);
        }

        glm::mat4 GetViewMatrix() const {
            return glm::lookAt(Eye, Target, Up);
		}

        glm::mat4 GetTransformationMatrix(float const aspect) const {
            return GetProjectionMatrix(aspect) * GetViewMatrix();
        }
    };

	class ICameraManager {
	public:
        virtual void Update(Camera & camera) = 0;
	};
}
