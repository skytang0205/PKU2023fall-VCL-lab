#pragma once

#include "Labs/4-Animation/MassSpringSystem.h"
#include "IKSystem.h"

namespace VCX::Labs::Animation {
    // lab4 inverse kinematics
    void ForwardKinematics(IKSystem & ik, int StartIndex);
    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps);
    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps);
    
    // lab4 mass spring system
    void AdvanceMassSpringSystem(MassSpringSystem &, float const);
}
