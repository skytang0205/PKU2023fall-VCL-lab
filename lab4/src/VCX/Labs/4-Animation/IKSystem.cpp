#include "IKSystem.h"
#include <glm/gtc/constants.hpp>


namespace VCX::Labs::Animation {
    const char * draw_items[] = {
        "circle",
        "square",
        "heart",
        "triangle",
        "star",
        "custom"
    };

    IKSystem::IKSystem() {
        for (int i = 0; i < 5; i++) {
            JointLocalRotation.emplace_back(glm::quat(1, 0, 0, 0));
            JointGlobalRotation.emplace_back(glm::quat(1, 0, 0, 0));
            JointGlobalPosition.push_back(0.0f + InitJointPosition[i]);
        }
        EndPositionHistory.resize(5000);
        JointLocalOffset.resize(5);
        JointLocalOffset[0] = InitJointPosition[0];
        JointOffsetLength.resize(5, 0.0f);
        for (int i = 1; i < 5; i++) {
            JointLocalOffset[i] = InitJointPosition[i] - InitJointPosition[i - 1];
            JointOffsetLength[i] = glm::l2Norm(JointLocalOffset[i]);
        }
        this->BuildEndPosition();
    }

    glm::vec3 & IKSystem::GetTarget() {
        Vec3ArrPtr target = TargetPositionList[TargetPositionIndex];
        glm::vec3 & res = (*target)[CurrIndex];
        this->CurrIndex = (CurrIndex + 1) % target->size();
        if (this->CurrIndex == 0) {
            for (int i = 0; i < 5; i++) {
                JointLocalRotation[i] = glm::quat(1, 0, 0, 0);
                JointGlobalRotation[i] = glm::quat(1, 0, 0, 0);
                JointGlobalPosition[i] = 0.0f + InitJointPosition[i];
            }
        }
        return res;
    }

    glm::vec3& IKSystem::EndEffectorPosition() {
        return *JointGlobalPosition.rbegin();
    }

    int IKSystem::NumJoints() const{
        return static_cast<int>(InitJointPosition.size());
    }

    void IKSystem::BuildEndPosition() {
        // build circle
        {
            int nums = 150;
            Vec3ArrPtr circle(new Vec3Arr(nums)); 
            const float radius = 0.6f;
            for (int i = 0; i < nums; i++) {
                (*circle)[i] = glm::vec3(radius * std::cos(2 * glm::pi<float>() * i / nums), 0.0f, radius * std::sin(2 * glm::pi<float>() * i / nums));
            }
            TargetPositionList.emplace_back(circle);
        }
        
        // build square
        {
            int nums = 150;
            Vec3ArrPtr square(new Vec3Arr(nums));
            const float width = 0.4f;
            int index1 = nums / 4, index2 = 2 * nums / 4, index3 = 3 * nums / 4;
            for (int i = 0; i < index1; i++) {
                (*square)[i] = glm::vec3(-0.5f * width + width * i / index1, 0.0f, 0.5 * width);
            }
            for (int i = index1; i < index2; i++) {
                (*square)[i] = glm::vec3(0.5 * width, 0.0f, 0.5 * width - width * (i - index1) / (index2 - index1));
            }
            for (int i = index2; i < index3; i++) {
                (*square)[i] = glm::vec3(0.5 * width - width * (i - index2) / (index3 - index2), 0.0f, -0.5f * width);
            }
            for (int i = index3; i < nums; i++) {
                (*square)[i] = glm::vec3(-0.5f * width, 0.0f, -0.5 * width + width * (i - index3) / (nums - index3));
            }
            TargetPositionList.emplace_back(square);
        }

        // build heart
        {
            int nums = 150;
            Vec3ArrPtr heart(new Vec3Arr(nums));
            for (int i = 0; i < nums; i++) {
                auto theta  = 2 * glm::pi<float>() * i / nums;
                auto r = 0.5f * (1 - std::sin(theta));
                (*heart)[i] = glm::vec3(r * std::cos(theta), 0.0f, r * std::sin(theta));
            }
            TargetPositionList.emplace_back(heart);
        }

        // build triangle
        {
            int nums = 150;
            Vec3ArrPtr triangle(new Vec3Arr(nums));
            int index1 = nums / 3, index2 = 2 * nums / 3;
            for (int i = 0; i < index1; i++) {
                (*triangle)[i] = glm::vec3(0.5f * i / index1, 0.0f, 0.0f);
            }
            for (int i = index1; i < index2; i++) {
                (*triangle)[i] = glm::vec3(0.5f - 0.5f * (i - index1) / (index2 - index1), 0.0f, 0.5f * (i - index1) / (index2 - index1));
            }
            for (int i = index2; i < nums; i++) {
                (*triangle)[i] = glm::vec3(0.0f, 0.0f, 0.5f - 0.5f * (i - index2) / (nums - index2));
            }
            TargetPositionList.emplace_back(triangle);
        }

        // build star
        {
            int nums = 400;
            Vec3ArrPtr star(new Vec3Arr(nums));
            const float radius = 0.4f;
            Vec3Arr pos_circle(5);
            for (int i = 0; i < 5; i++) {
                pos_circle[i] = glm::vec3(radius * std::cos(2 * glm::pi<float>() * i / 5), 0.0f, radius * std::sin(2 * glm::pi<float>() * i / 5));
            }
            int index1 = nums / 5, index2 = 2 * nums / 5, index3 = 3 * nums / 5, index4 = 4 * nums / 5;
            for (int i = 0; i < index1; i++) {
                (*star)[i] = float(index1 - i) / index1 * pos_circle[4] + float(i) / index1 * pos_circle[1];
            }
            for (int i = index1; i < index2; i++) {
                (*star)[i] = float(index2 - i) / (index2 - index1) * pos_circle[1] + float(i - index1) / (index2 - index1) * pos_circle[3];
            }
            for (int i = index2; i < index3; i++) {
                (*star)[i] = float(index3 - i) / (index3 - index2) * pos_circle[3] + float(i - index2) / (index3 - index2) * pos_circle[0];
            }
            for (int i = index3; i < index4; i++) {
                (*star)[i] = float(index4 - i) / (index4 - index3) * pos_circle[0] + float(i - index3) / (index4 - index3) * pos_circle[2];
            }
            for (int i = index4; i < nums; i++) {
                (*star)[i] = float(nums - i) / (nums - index4) * pos_circle[2] + float(i - index4) / (nums - index4) * pos_circle[4];
            }
            TargetPositionList.emplace_back(star);
        }

        // custom shape
        {
            // here build shape by yourself.
            auto custom = 0;
            TargetPositionList.emplace_back(BuildCustomTargetPosition());
        }
    }

    IKSystem::Vec3ArrPtr IKSystem::GetTargetPositionList() {
        return TargetPositionList[TargetPositionIndex];
    }
}