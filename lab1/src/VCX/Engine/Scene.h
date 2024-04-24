#pragma once

#include <optional>
#include <variant>

#include "Engine/Camera.hpp"
#include "Engine/SurfaceMesh.h"
#include "Engine/TextureND.hpp"

namespace VCX::Engine {
    struct Skybox {
        static constexpr auto c_PositionData = std::to_array<glm::vec3>({
            { -1.0f,  1.0f, -1.0f }, { -1.0f, -1.0f, -1.0f }, {  1.0f, -1.0f, -1.0f },
            {  1.0f, -1.0f, -1.0f }, {  1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f },
            { -1.0f, -1.0f,  1.0f }, { -1.0f, -1.0f, -1.0f }, { -1.0f,  1.0f, -1.0f },
            { -1.0f,  1.0f, -1.0f }, { -1.0f,  1.0f,  1.0f }, { -1.0f, -1.0f,  1.0f },
            {  1.0f, -1.0f, -1.0f }, {  1.0f, -1.0f,  1.0f }, {  1.0f,  1.0f,  1.0f },
            {  1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f, -1.0f }, {  1.0f, -1.0f, -1.0f },
            { -1.0f, -1.0f,  1.0f }, { -1.0f,  1.0f,  1.0f }, {  1.0f,  1.0f,  1.0f },
            {  1.0f,  1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f }, { -1.0f, -1.0f,  1.0f },
            { -1.0f,  1.0f, -1.0f }, {  1.0f,  1.0f, -1.0f }, {  1.0f,  1.0f,  1.0f },
            {  1.0f,  1.0f,  1.0f }, { -1.0f,  1.0f,  1.0f }, { -1.0f,  1.0f, -1.0f },
            { -1.0f, -1.0f, -1.0f }, { -1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f, -1.0f },
            {  1.0f, -1.0f, -1.0f }, { -1.0f, -1.0f,  1.0f }, {  1.0f, -1.0f,  1.0f },
        });
        std::array<Texture2D<Formats::RGB8>, 6> Images;
    };

    enum class LightType {
        Point,
        Spot,
        Directional,
        Area,
    };

    struct Light {
        LightType Type        { LightType::Point };
        glm::vec3 Intensity   { 1, 1, 1 };
        glm::vec3 Direction   { 1, 0, 0 };
        glm::vec3 Position    { 0, 0, 0 };
        float     CutOff      { 0 };
        float     OuterCutOff { 0 };
    };

    enum class BlendMode {
        Opaque,
        Transparent,
    };

    struct Material {
        BlendMode                     Blend;
        Texture2D<Formats::RGBA8>     Albedo   { 1, 1 };
        // Phong/Blinn-Phong:  MetaSpec = [Specular (RGB), Shininess  (Alpha)]
        // PBR workflow I:     MetaSpec = [Metallic (Red), Smoothness (Alpha)]
        // PBR workflow II:    MetaSpec = [Specular (RGB), Glossiness (Alpha)]
        Texture2D<Formats::RGBA8>     MetaSpec { 1, 1 };
        Texture2D<Formats::R8>        Height   { 1, 1 };
    };

    struct Model {
        SurfaceMesh   Mesh          { };
        std::uint32_t MaterialIndex { 0 };
    };

    enum class ReflectionType {
        Empirical,
        PhysicalMetallic,
        PhysicalSpecular,
    };

    struct Scene {
        ReflectionType        Reflection       { ReflectionType::Empirical };
        glm::vec3             AmbientIntensity { 0.1, 0.1, 0.1 };
        std::vector<Skybox>   Skyboxes;
        std::vector<Camera>   Cameras          { Camera() };
        std::vector<Light>    Lights;
        std::vector<Material> Materials;
        std::vector<Model>    Models;

        std::pair<glm::vec3, glm::vec3> GetAxisAlignedBoundingBox() const;
    };
} // namespace VCX::Engine
