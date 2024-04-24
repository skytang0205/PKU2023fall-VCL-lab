#pragma once

#include <imgui_internal.h>

#include "Engine/app.h"
#include "Labs/Common/FlyCameraManager.h"

namespace VCX::Labs::Common {
    void FlyCameraManager::Save(Engine::Camera const & camera) {
        _position0 = camera.Eye;
        _target0   = camera.Target;
        _fov0      = camera.Fovy;
    }

    void FlyCameraManager::Reset(Engine::Camera & camera) {
        Engine::Spherical sp0 = _target0 - _position0;
        Engine::Spherical sp  = camera.Target - camera.Eye;
        _spDelta.Phi          = sp0.Phi - sp.Phi;
        _spDelta.Theta        = sp0.Theta - sp.Theta;
        _panOffset            = _position0 - camera.Eye;
        _deltaFOV             = _fov0 - camera.Fovy;
    }

    void FlyCameraManager::Update(Engine::Camera & camera) {
        glm::quat q;
        {
            glm::vec3 up   = glm::vec3(0, 1, 0);
            glm::vec3 axis = glm::cross(camera.Up, up);
            float     s    = glm::sqrt(2 * (1 + glm::dot(camera.Up, up)));
            axis /= s;
            q = glm::quat(s / 2, axis.x, axis.y, axis.z);
        }
        const glm::quat qInv = glm::inverse(q);

        // so camera.up is the orbit axis
        // rotate offset to "y-axis-is-up" space
        // angle from z-axis around y-axis
        Engine::Spherical sp = q * (camera.Target - camera.Eye);
        sp.Theta += _spDelta.Theta;
        sp.Phi += _spDelta.Phi;

        // restrict phi to be between desired limits
        sp.Phi = glm::max(MinPolarAngle, glm::min(MaxPolarAngle, sp.Phi));
        sp.MakeSafe();

        // restrict radius to be between desired limits
        sp.Radius = std::max(MinDistance, std::min(MaxDistance, sp.Radius));

        // move target to panned location
        camera.Eye += _panOffset * DampingFactor;

        // rotate offset back to "camera-up-vector-is-up" space
        camera.Target = camera.Eye + qInv * sp.Vec();

        camera.Fovy += _deltaFOV * DampingFactor;
        camera.Fovy = std::max(MinFOV, std::min(MaxFOV, camera.Fovy));
        
        const float factor = 1 - DampingFactor;
        _spDelta           = Engine::Spherical();
        _panOffset *= factor;
        _deltaFOV *= factor;
    }

    void FlyCameraManager::ProcessInput(Engine::Camera & camera, ImVec2 const & mousePos) {
        auto            window  = ImGui::GetCurrentWindow();
        ImGuiIO const & io      = ImGui::GetIO();
        bool            anyHeld = false;
        bool            hover   = false;
        ImGui::ButtonBehavior(window->Rect(), window->GetID("##io"), &hover, &anyHeld);
        bool         leftHeld       = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Left);
        bool         rightHeld      = anyHeld && ImGui::IsMouseDown(ImGuiMouseButton_Right);
        ImVec2 const delta          = io.MouseDelta;
        float        wheel          = io.MouseWheel;
        bool         wheeling       = EnableZoom && wheel != 0.f && hover;
        bool         moving         = (delta.x != 0.f || delta.y != 0.f) && hover;
        float        heightNorm     = 1.f / window->Rect().GetHeight();
        bool         rotating       = moving && leftHeld;

        bool panningByKeyBoard[6] = {
            ImGui::IsItemFocused() && (ImGui::IsKeyDown(ImGuiKey_W) || ImGui::IsKeyDown(ImGuiKey_UpArrow)),
            ImGui::IsItemFocused() && (ImGui::IsKeyDown(ImGuiKey_S) || ImGui::IsKeyDown(ImGuiKey_DownArrow)),
            ImGui::IsItemFocused() && (ImGui::IsKeyDown(ImGuiKey_A) || ImGui::IsKeyDown(ImGuiKey_LeftArrow)),
            ImGui::IsItemFocused() && (ImGui::IsKeyDown(ImGuiKey_D) || ImGui::IsKeyDown(ImGuiKey_RightArrow)),
            ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_Q),
            ImGui::IsItemFocused() && ImGui::IsKeyDown(ImGuiKey_E)
        };

        if (ImGui::IsItemFocused() && ImGui::IsKeyPressed(ImGuiKey_R)) {
            Reset(camera);
            return;
        }

        _state = StateNone;
        if (anyHeld) {
            _state |= StateHold;
        }
        if (panningByKeyBoard[0] || panningByKeyBoard[1] || panningByKeyBoard[2] || panningByKeyBoard[3] || panningByKeyBoard[4] || panningByKeyBoard[5]) {
            // perspective
            glm::vec3 direction      = camera.Target - camera.Eye;
            float     targetDistance = glm::length(direction);
            glm::quat q              = glm::quatLookAt(direction / targetDistance, camera.Up);
            // half of the fov is center to top of screen
            targetDistance *= glm::tan(glm::radians(camera.Fovy) * 0.5f);
            float panX = 0.f;
            float panY = 0.f;
            float panZ = 0.f;
            if (panningByKeyBoard[0]) panZ -= 0.04f;
            if (panningByKeyBoard[1]) panZ += 0.04f;
            if (panningByKeyBoard[2]) panX -= 0.04f;
            if (panningByKeyBoard[3]) panX += 0.04f;
            if (panningByKeyBoard[4]) panY -= 0.04f;
            if (panningByKeyBoard[5]) panY += 0.04f;
            // we use only clientHeight here so aspect ratio does not distort speed
            float     panLeft  = 2 * panX * PanSpeed * targetDistance;
            float     panUp    = 2 * panY * PanSpeed * targetDistance;
            float     panFront = 2 * panZ * PanSpeed * targetDistance;
            glm::vec3 front    = glm::cross(q * glm::vec3(1, 0, 0), camera.Up);
            _panOffset         = q * glm::vec3(panLeft, 0.f, 0.f) + panFront * front + panUp * camera.Up;
            _state |= StatePan;
        }
        if (rotating) {
            _spDelta.Theta -= delta.x * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
            _spDelta.Phi += delta.y * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
            _state |= StateRotate;
        }
        if (wheeling) {
            _deltaFOV -= ZoomSpeed * wheel * 2.f;
            _state |= StateDolly;
        }
    }
}
