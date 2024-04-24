#pragma once

#include "Engine/Camera.hpp"
#include "Engine/math.hpp"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Common {
    class FlyCameraManager : public Engine::ICameraManager {
    public:
        FlyCameraManager(glm::vec3 const & eye = glm::vec3(0.f, 0.f, 1.f), glm::vec3 const & target = glm::vec3(0)):
            _position0(eye),
            _target0(target),
            _fov0(45.f) {
        }

        float MinFOV = 10.0f;
        float MaxFOV = 90.0f;

        // How far you can dolly in and out ( PerspectiveCamera only )
        float MinDistance = std::numeric_limits<float>::epsilon();
        float MaxDistance = std::numeric_limits<float>::infinity();

        // How far you can orbit vertically, upper and lower limits.
        // Range is 0 to Math.PI radians.
        float MinPolarAngle = 0.f;
        float MaxPolarAngle = glm::pi<float>();

        // Affect moving
        float DampingFactor = 0.2f;

        // This option actually enables dollying in and out; left as "zoom" for backwards compatibility.
        // Set to false to disable zooming
        bool  EnableZoom = true;
        float ZoomSpeed  = 1.f;

        // Set to false to disable rotating
        float RotateSpeed = 1.f;

        // Set to false to disable panning
        float PanSpeed = 1.f;


        void Save(Engine::Camera const & camera);
        void Reset(Engine::Camera & camera);
        virtual void Update(Engine::Camera & camera) override;
        void ProcessInput(Engine::Camera & camera, ImVec2 const & mousePos);
    
    private:

        enum StateBits {
            StateNone   = 0,
            StateRotate = 1 << 0,
            StateDolly  = 1 << 1,
            StatePan    = 1 << 2,
            StateHold   = 1 << 3
        };

        glm::vec3         _target0;
        glm::vec3         _position0;
        float             _fov0;
        glm::vec3         _panOffset = glm::vec3(0.f);
        float             _deltaFOV  = 0.f;
        int               _state     = StateNone;
        Engine::Spherical _spDelta;
    };
}
