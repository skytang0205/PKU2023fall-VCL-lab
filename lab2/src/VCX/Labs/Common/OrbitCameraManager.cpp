#pragma once

#include <imgui_internal.h>
#include <iostream>

#include "Engine/app.h"
#include "Labs/Common/OrbitCameraManager.h"

namespace VCX::Labs::Common {
    void OrbitCameraManager::Save(Engine::Camera const & camera) {
        _position0 = camera.Eye;
        _target0   = camera.Target;
    }

    void OrbitCameraManager::Reset(Engine::Camera & camera) {
        Engine::Spherical sp0  = _position0 - _target0;
        Engine::Spherical sp   = camera.Eye - camera.Target;
        _spDelta.Phi           = sp0.Phi - sp.Phi;
        _spDelta.Theta         = sp0.Theta - sp.Theta;
        _panOffset             = _target0 - camera.Target;
        _logScale              = glm::log(sp0.Radius / sp.Radius) / glm::log(0.9f);
    }

    void OrbitCameraManager::Update(Engine::Camera & camera) {
        float const dt = Engine::GetDeltaTime();

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
        Engine::Spherical sp = q * (camera.Eye - camera.Target);
        if (AutoRotate && _state == StateNone) {
            sp.Theta -= 2 * glm::pi<float>() / 60 * AutoRotateSpeed * dt;
        }
        if (EnableDamping) {
            sp.Theta += _spDelta.Theta * DampingFactor;
            sp.Phi += _spDelta.Phi * DampingFactor;
        } else {
            sp.Theta += _spDelta.Theta;
            sp.Phi += _spDelta.Phi;
        }

        // restrict theta to be between desired limits
        if (! std::isinf(MinAzimuthAngle) && ! std::isinf(MaxAzimuthAngle)) {
            if (MinAzimuthAngle < -glm::pi<float>()) MinAzimuthAngle += glm::pi<float>() * 2;
            else if (MinAzimuthAngle > glm::pi<float>()) MinAzimuthAngle -= glm::pi<float>() * 2;
            if (MaxAzimuthAngle < -glm::pi<float>()) MaxAzimuthAngle += glm::pi<float>() * 2;
            else if (MaxAzimuthAngle > glm::pi<float>()) MaxAzimuthAngle -= glm::pi<float>() * 2;
            if (MinAzimuthAngle <= MaxAzimuthAngle) {
                sp.Theta = glm::max(MinAzimuthAngle, glm::min(MinAzimuthAngle, sp.Theta));
            } else {
                sp.Theta = sp.Theta > (MinAzimuthAngle + MinAzimuthAngle) / 2 ? glm::max(MinAzimuthAngle, sp.Theta) : glm::min(MinAzimuthAngle, sp.Theta);
            }
        }

        // restrict phi to be between desired limits
        sp.Phi = glm::max(MinPolarAngle, glm::min(MaxPolarAngle, sp.Phi));
        sp.MakeSafe();
        if (EnableDamping) {
            sp.Radius *= glm::pow(0.9f, _logScale * DampingFactor);
        } else {
            sp.Radius *= glm::pow(0.9f, _logScale);
        }

        // restrict radius to be between desired limits
        sp.Radius = std::max(MinDistance, std::min(MaxDistance, sp.Radius));

        // move target to panned location
        if (EnableDamping) {
            camera.Target += _panOffset * DampingFactor;
        } else {
            camera.Target += _panOffset;
        }

        // rotate offset back to "camera-up-vector-is-up" space
        camera.Eye = camera.Target + qInv * sp.Vec();
        if (EnableDamping) {
            const float factor = 1 - DampingFactor;
            _spDelta.Theta *= factor;
            _spDelta.Phi *= factor;
            _panOffset *= factor;
            _logScale *= factor;
        } else {
            _spDelta   = Engine::Spherical();
            _panOffset = glm::vec3(0.f);
            _logScale  = 0.f;
        }
    }

    void OrbitCameraManager::ProcessInput(Engine::Camera & camera, ImVec2 const & mousePos) {
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
        bool         funcKey        = io.KeyCtrl || io.KeyShift;
        float        heightNorm     = 1.f / window->Rect().GetHeight();
        bool         rotating       = EnableRotate && moving && ((! funcKey && leftHeld) || (funcKey && rightHeld));
        bool         panningByMouse = EnablePan && moving && ((funcKey && leftHeld) || (! funcKey && rightHeld));

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
        if (panningByMouse || (EnablePan && (panningByKeyBoard[0] || panningByKeyBoard[1] || panningByKeyBoard[2] || panningByKeyBoard[3] || panningByKeyBoard[4] || panningByKeyBoard[5]))) {
            // perspective
            glm::vec3 direction      = camera.Target - camera.Eye;
            float     targetDistance = glm::length(direction);
            glm::quat q              = glm::quatLookAt(direction / targetDistance, camera.Up);
            // half of the fov is center to top of screen
            targetDistance *= glm::tan(glm::radians(camera.Fovy) * 0.5f);
            float panX = panningByMouse ? -delta.x * heightNorm : 0.f;
            float panY = panningByMouse ? delta.y * heightNorm : 0.f;
            float panZ = 0.f;
            if (panningByKeyBoard[0]) panZ -= 0.02f;
            if (panningByKeyBoard[1]) panZ += 0.02f;
            if (panningByKeyBoard[2]) panX -= 0.02f;
            if (panningByKeyBoard[3]) panX += 0.02f;
            if (panningByKeyBoard[4]) panY -= 0.02f;
            if (panningByKeyBoard[5]) panY += 0.02f;
            // we use only clientHeight here so aspect ratio does not distort speed
            float     panLeft  = 2 * panX * PanSpeed * targetDistance;
            float     panUp    = 2 * panY * PanSpeed * targetDistance;
            float     panFront = 2 * panZ * PanSpeed * targetDistance;
            glm::vec3 front    = glm::cross(q * glm::vec3(1, 0, 0), camera.Up);
            if (ScreenSpacePanning) {
                _panOffset += q * glm::vec3(panLeft, panUp, 0.f) + panFront * front;
            } else {
                _panOffset += q * glm::vec3(panLeft, 0.f, 0.f) + panUp * camera.Up + panFront * front;
            }
            _state |= StatePan;
        }
        if (rotating) {
            _spDelta.Theta -= delta.x * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
            _spDelta.Phi -= delta.y * RotateSpeed * heightNorm * (glm::pi<float>() * 2);
            _state |= StateRotate;
        }
        if (wheeling) {
            _logScale += ZoomSpeed * wheel;
            _state |= StateDolly;
        }
    }
}
