#pragma once

#include <vector>
#include <memory>
#include <glm/glm.hpp>
#include <glm/ext/quaternion_float.hpp>
#include <glm/ext.hpp>
#include <glm/gtx/quaternion.hpp>


namespace VCX::Labs::Animation {
    extern const char * draw_items[];

    struct IKSystem {
        using Vec3Arr     = std::vector<glm::vec3>;
        using Vec3ArrPtr  = std::shared_ptr<std::vector<glm::vec3>>;

        IKSystem();
        glm::vec3 &       GetTarget();
        glm::vec3 &       EndEffectorPosition();
        int               NumJoints() const;
        void              BuildEndPosition();
        Vec3ArrPtr        GetTargetPositionList();
        static Vec3ArrPtr BuildCustomTargetPosition();

        const std::vector<glm::vec3> InitJointPosition {
            glm::vec3(0.0f, 1.7f, 0.0f),
            glm::vec3(0.0f, 0.9f, 0.0f),
            glm::vec3(0.0f, 0.5f, 0.0f),
            glm::vec3(0.0f, 0.3f, 0.0f),
            glm::vec3(0.0f, -0.9f, 0.0f)
        };

        std::vector<glm::vec3>   JointLocalOffset;
        std::vector<float>       JointOffsetLength;
        std::vector<glm::vec3>   JointGlobalPosition;
        std::vector<glm::quat>   JointLocalRotation;
        std::vector<glm::quat>   JointGlobalRotation;

        // here we should prebuild the end positions.
        std::vector<Vec3ArrPtr>  TargetPositionList;
        int                      TargetPositionIndex = 0;

        std::vector<glm::vec3>   EndPositionHistory;
        int                      CurrIndex = 0;
    };
}
