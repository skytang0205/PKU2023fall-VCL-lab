#pragma once

#include "Engine/Camera.hpp"
#include "Engine/math.hpp"
#include "Labs/Common/ImGuiHelper.h"

namespace VCX::Labs::Common {
    class OrbitCameraManager : public Engine::ICameraManager {
    public:
        OrbitCameraManager(glm::vec3 const & eye = glm::vec3(0.f, 0.f, 1.f), glm::vec3 const & target = glm::vec3(0)) :
            _position0(eye),
            _target0(target) {
        }

        // How far you can dolly in and out ( PerspectiveCamera only )
        float MinDistance = std::numeric_limits<float>::epsilon();
        float MaxDistance = std::numeric_limits<float>::infinity();

        // How far you can orbit vertically, upper and lower limits.
        // Range is 0 to Math.PI radians.
        float MinPolarAngle = 0.f;
        float MaxPolarAngle = glm::pi<float>();

        // How far you can orbit horizontally, upper and lower limits.
        // If set, the interval [ min, max ] must be a sub-interval of [ - 2 PI, 2 PI ], with ( max - min < 2 PI )
        float MinAzimuthAngle = -std::numeric_limits<float>::infinity();
        float MaxAzimuthAngle = std::numeric_limits<float>::infinity();
        
        // Set to true to enable damping (inertia)
        bool  EnableDamping = true;
        float DampingFactor = 0.1f;

        // This option actually enables dollying in and out; left as "zoom" for backwards compatibility.
        // Set to false to disable zooming
        bool  EnableZoom = true;
        float ZoomSpeed  = 1.f;

        // Set to false to disable rotating
        bool  EnableRotate = true;
        float RotateSpeed  = 1.f;

        // Set to false to disable panning
        bool  EnablePan          = true;
        float PanSpeed           = 1.f;
        bool  ScreenSpacePanning = false; // if false, pan orthogonal to world-space direction camera.up

        bool  AutoRotate      = true;
        float AutoRotateSpeed = 2.f;

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
        glm::vec3         _panOffset    = glm::vec3(0.f);
        float             _logScale     = 0.f;
        int               _state        = StateNone;
        Engine::Spherical _spDelta;
    };
}
