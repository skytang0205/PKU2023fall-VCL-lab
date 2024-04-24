#include <filesystem>
#include <fstream>

#include <spdlog/spdlog.h>
#include <stb_image.h>
#include <tiny_obj_loader.h>
#include <yaml-cpp/yaml.h>

#include "Engine/loader.h"

namespace std {
    template<>
    struct hash<tuple<int, int, int>> {
        size_t operator()(tuple<int, int, int> const & val) const {
            auto [i1, i2, i3] = val;
            return ((i1 ^ (i2 << 1)) >> 1) ^ (i3 << 1);
        }
    };
}

namespace YAML {
    template<>
    struct convert<VCX::Engine::ReflectionType> {
        static bool decode(Node const & node, VCX::Engine::ReflectionType & rhs) {
            std::string str { node.as<std::string>() };
            if (str == "Empirical") rhs = VCX::Engine::ReflectionType::Empirical;
            else if (str == "PhysicalMetallic") rhs = VCX::Engine::ReflectionType::PhysicalMetallic;
            else if (str == "PhysicalSpecular") rhs = VCX::Engine::ReflectionType::PhysicalSpecular;
            else return false;
            return true;
        }
    };

    template<>
    struct convert<VCX::Engine::LightType> {
        static bool decode(Node const & node, VCX::Engine::LightType & rhs) {
            std::string str { node.as<std::string>() };
            if (str == "Point") rhs = VCX::Engine::LightType::Point;
            else if (str == "Spot") rhs = VCX::Engine::LightType::Spot;
            else if (str == "Directional") rhs = VCX::Engine::LightType::Directional;
            else if (str == "Area") rhs = VCX::Engine::LightType::Area;
            else return false;
            return true;
        }
    };

    template<>
    struct convert<VCX::Engine::BlendMode> {
        static bool decode(Node const & node, VCX::Engine::BlendMode & rhs) {
            std::string str { node.as<std::string>() };
            if (str == "Opaque") rhs = VCX::Engine::BlendMode::Opaque;
            else if (str == "Transparent") rhs = VCX::Engine::BlendMode::Transparent;
            else return false;
            return true;
        }
    };

    template<glm::length_t N, typename T, glm::qualifier Q>
        requires std::is_arithmetic_v<T>
    struct convert<glm::vec<N, T, Q>> {
        static bool decode(Node const & node, glm::vec<N, T, Q> & rhs) {
            if (! node.IsSequence() || node.size() > N)
		        return false;
            rhs = glm::vec<N, T, Q>(1);
            for (std::size_t i = 0; i < node.size(); i++)
                rhs[i] = node[i].as<T>();
            return true;
        }
    };

    template<glm::length_t N, glm::length_t M, typename T, glm::qualifier Q>
        requires std::is_arithmetic_v<T>
    struct convert<glm::mat<N, M, T, Q>> {
        static bool decode(Node const & node, glm::mat<N, M, T, Q> & rhs) {
            if (! node.IsSequence() || node.size() > M)
		        return false;
            for (std::size_t i = 0; i < node.size(); i++)
                if (! node[i].IsSequence() || node[i].size() > N)
                    return false;
            rhs = glm::mat<N, M, T, Q>(1);
            for (std::size_t i = 0; i < node.size(); i++)
                for (std::size_t j = 0; j < node[i].size(); j++)
                    rhs[j][i] = node[i][j].as<T>();
            return true;
        }
    };
}

namespace VCX::Engine {
    std::vector<std::byte> LoadBytes(std::filesystem::path const & fileName) {
        if (std::filesystem::exists(fileName)) {
            std::ifstream file(fileName, std::ios::binary);
            file.seekg(0, std::ios::end);
            const auto fileSize { static_cast<std::size_t>(file.tellg()) };
            file.seekg(0, std::ios::beg);
            std::vector<std::byte> blob;
            blob.resize(fileSize);
            file.read(reinterpret_cast<char *>(blob.data()), blob.size());
            file.close();
            spdlog::trace("VCX::Engine::LoadBytes(\"{}\")", fileName.filename().string());
            return blob;
        } else {
            spdlog::error("VCX::Engine::LoadBytes(\"{}\"): not found.", fileName.filename().string());
            return {};
        }
    }

    Texture2D<Formats::R8> LoadImageGray(std::filesystem::path const & fileName, bool const flipped) {
        auto const buf { LoadBytes(fileName) };
        int        width {}, height {}, channels {};
        stbi_set_flip_vertically_on_load(flipped);
        auto const image {
            stbi_load_from_memory(
                reinterpret_cast<const stbi_uc *>(buf.data()),
                buf.size(),
                &width,
                &height,
                &channels,
                1)
        };
        Texture2D<Formats::R8> texture(width, height);
        std::memcpy(
            reinterpret_cast<void *>(const_cast<std::byte *>(texture.GetBytes().data())),
            image,
            texture.GetBytes().size());
        stbi_image_free(image);
        return texture;
    }

    Texture2D<Formats::RGB8> LoadImageRGB(std::filesystem::path const & fileName, bool const flipped) {
        auto const buf { LoadBytes(fileName) };
        int        width {}, height {}, channels {};
        stbi_set_flip_vertically_on_load(flipped);
        auto const image {
            stbi_load_from_memory(
                reinterpret_cast<const stbi_uc *>(buf.data()),
                buf.size(),
                &width,
                &height,
                &channels,
                3)
        };
        Texture2D<Formats::RGB8> texture(width, height);
        std::memcpy(
            reinterpret_cast<void *>(const_cast<std::byte *>(texture.GetBytes().data())),
            image,
            texture.GetBytes().size());
        stbi_image_free(image);
        return texture;
    }

    Texture2D<Formats::RGBA8> LoadImageRGBA(std::filesystem::path const & fileName, bool const flipped) {
        auto const buf { LoadBytes(fileName) };
        int        width {}, height {}, channels {};
        stbi_set_flip_vertically_on_load(flipped);
        auto const image {
            stbi_load_from_memory(
                reinterpret_cast<const stbi_uc *>(buf.data()),
                buf.size(),
                &width,
                &height,
                &channels,
                4)
        };
        Texture2D<Formats::RGBA8> texture(width, height);
        std::memcpy(
            reinterpret_cast<void *>(const_cast<std::byte *>(texture.GetBytes().data())),
            image,
            texture.GetBytes().size());
        stbi_image_free(image);
        return texture;
    }

    static void AddUniqueVertices(
        tinyobj::attrib_t                                            const & attrib,
        std::vector<tinyobj::index_t>                                const & indices,
        std::unordered_map<std::tuple<int, int, int>, std::uint32_t>         vtxHashList,
        SurfaceMesh                                                        & mesh,
        bool                                                         const   simplified     = false) {
        for (auto const & index : indices) {
            auto const vertex = simplified
                ? std::tuple(index.vertex_index, -1, -1)
                : std::tuple(index.vertex_index, index.normal_index, index.texcoord_index);
            if (auto const iter = vtxHashList.find(vertex); iter != vtxHashList.end()) {
                mesh.Indices.push_back(iter->second);
            } else {
                mesh.Indices.push_back(vtxHashList[vertex] = std::uint32_t(mesh.Positions.size()));

                mesh.Positions.emplace_back(attrib.vertices[index.vertex_index * 3 + 0], attrib.vertices[index.vertex_index * 3 + 1], attrib.vertices[index.vertex_index * 3 + 2]);
                if (index.normal_index >= 0)
                    mesh.Normals.emplace_back(attrib.normals[index.normal_index * 3 + 0], attrib.normals[index.normal_index * 3 + 1], attrib.normals[index.normal_index * 3 + 2]);
                if (index.texcoord_index >= 0)
                    mesh.TexCoords.emplace_back(attrib.texcoords[index.texcoord_index * 2 + 0], 1 - attrib.texcoords[index.texcoord_index * 2 + 1]);
            }
        }
    }

    static SurfaceMesh LoadSurfaceMeshOBJ(std::filesystem::path const & fileName, bool const simplified) {
        tinyobj::attrib_t                attrib;
        std::vector<tinyobj::shape_t>    shapes;
        std::vector<tinyobj::material_t> materials;

        std::string warn;
        std::string err;

        std::ifstream file(fileName);
        if (! file) {
            spdlog::error("VCX::Engine::LoadSurfaceMeshOBJ(\"{}\"): not found.", fileName.filename().string());
            return {};
        }

        bool const ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, &file);
        if (! warn.empty())
            spdlog::warn("VCX::Engine::LoadSurfaceMeshOBJ(\"{}\"): {}", fileName.filename().string(), warn);
        if (! err.empty())
            spdlog::error("VCX::Engine::LoadSurfaceMeshOBJ(\"{}\"): {}", fileName.filename().string(), err);
        if (! ret)
            return {};

        spdlog::trace("VCX::Engine::LoadSurfaceMeshOBJ(\"{}\")", fileName.filename().string());

        std::unordered_map<std::tuple<int, int, int>, std::uint32_t> vtxHashList;
        SurfaceMesh                                                  mesh;

        for (auto const & shape : shapes)
            AddUniqueVertices(attrib, shape.mesh.indices, vtxHashList, mesh, simplified);

        return mesh;
    }

    SurfaceMesh LoadSurfaceMesh(std::filesystem::path const & fileName, bool const simplified) {
        auto const ext = fileName.extension();
        if (ext == ".obj") {
            return LoadSurfaceMeshOBJ(fileName, simplified);
        } else {
            spdlog::error("VCX::Engine::LoadSurfaceMesh(\"{}\"): undertermined file format.", fileName.filename().string());
            return {};
        }
    }

    static void LoadComplexModelsOBJ(std::filesystem::path const & fileName, std::vector<Material> & materials, std::vector<Model> & models) {
        auto const directory = fileName.parent_path();

        tinyobj::attrib_t                attrib;
        std::vector<tinyobj::shape_t>    shapes;
        std::vector<tinyobj::material_t> mats;

        std::string warn;
        std::string err;

        bool const ret = tinyobj::LoadObj(&attrib, &shapes, &mats, &warn, &err, fileName.string().c_str(), directory.string().c_str());
        if (! warn.empty())
            spdlog::warn("VCX::Engine::LoadModelsOBJ(\"{}\"): {}", fileName.filename().string(), warn);
        if (! err.empty())
            spdlog::error("VCX::Engine::LoadModelsOBJ(\"{}\"): {}", fileName.filename().string(), err);
        if (! ret)
            return;

        spdlog::trace("VCX::Engine::LoadModelsOBJ(\"{}\")", fileName.filename().string());

        std::vector<std::vector<tinyobj::index_t>> perMatFaces(mats.size());

        for (auto const & shape : shapes) {
            for (std::size_t i = 0; i < shape.mesh.indices.size(); i += 3)
                if (auto const mat = shape.mesh.material_ids[i / 3]; mat >= 0) {
                    perMatFaces[mat].push_back(shape.mesh.indices[i + 0]);
                    perMatFaces[mat].push_back(shape.mesh.indices[i + 1]);
                    perMatFaces[mat].push_back(shape.mesh.indices[i + 2]);
                }
        }

        auto const SetMap1 = [&]<typename T>(T & val, std::string const & str) { if (str != "") val = LoadImageGray(directory / str); };
        auto const SetMap4 = [&]<typename T>(T & val, std::string const & str) { if (str != "") val = LoadImageRGBA(directory / str); };

        for (std::size_t i = 0; i < mats.size(); i++) {
            if (perMatFaces[i].empty()) continue;

            models.emplace_back();
            auto & model = models.back();
            model.MaterialIndex = std::uint32_t(materials.size());

            materials.emplace_back();
            auto & material = materials.back();

            material.Blend = BlendMode::Opaque;

            material.Albedo.Fill(glm::vec4(mats[i].diffuse[0], mats[i].diffuse[1], mats[i].diffuse[2], mats[i].dissolve));
            SetMap4(material.Albedo, mats[i].diffuse_texname);

            material.MetaSpec.Fill(glm::vec4(mats[i].specular[0], mats[i].specular[1], mats[i].specular[2], mats[i].shininess / 256.f));
            SetMap4(material.MetaSpec, mats[i].specular_texname);

            material.Height.Fill(0);
            SetMap1(material.Height, mats[i].bump_texname);

            std::unordered_map<std::tuple<int, int, int>, std::uint32_t> vtxHashList;
            AddUniqueVertices(attrib, perMatFaces[i], vtxHashList, model.Mesh);
        }
    }

    static void LoadComplexModels(std::filesystem::path const & fileName, std::vector<Material> & materials, std::vector<Model> & models) {
        auto const ext = fileName.extension();
        if (ext == ".obj") {
            LoadComplexModelsOBJ(fileName, materials, models);
        } else {
            spdlog::error("VCX::Engine::LoadModels(\"{}\"): undertermined file format.", fileName.filename().string());
        }
    }

    Scene LoadScene(std::filesystem::path const & fileName) {
		std::ifstream fin(fileName);
		if (!fin) {
            spdlog::error("VCX::Engine::LoadScene(\"{}\"): not found.", fileName.filename().string());
            return {};
		}
        spdlog::trace("VCX::Engine::LoadScene(\"{}\")", fileName.filename().string());
        auto const directory = fileName.parent_path();
		auto const root = YAML::Load(fin);

        auto constexpr SetValue = [] <typename T>(T & val, YAML::Node const & node) { if (node) val = node.as<T>(); };
        auto const     SetMap1  = [&]<typename T>(T & val, YAML::Node const & node) { if (node) val = LoadImageGray(directory / node.as<std::string>()); };
        auto const     SetMap4  = [&]<typename T>(T & val, YAML::Node const & node) { if (node) val = LoadImageRGBA(directory / node.as<std::string>()); };

        Scene scene;
        SetValue(scene.Reflection      , root["Reflection"]);
        SetValue(scene.AmbientIntensity, root["AmbientIntensity"]);

        scene.Skyboxes.clear();
        if (root["Skyboxes"]) {
            for (auto const & skyboxNode : root["Skyboxes"]) {
                Skybox skybox;
                for (std::size_t i = 0; i < 6; ++i)
                    skybox.Images[i] = LoadImageRGB(directory / skyboxNode[i].as<std::string>());
                scene.Skyboxes.push_back(std::move(skybox));
            }
        }

        scene.Cameras.clear();
        if (root["Cameras"]) {
            for (auto const & cameraNode : root["Cameras"]) {
                Camera camera;
                SetValue(camera.Fovy  , cameraNode["Fovy"]);
                SetValue(camera.ZNear , cameraNode["ZNear"]);
                SetValue(camera.ZFar  , cameraNode["ZFar"]);
                SetValue(camera.Eye   , cameraNode["Eye"]);
                SetValue(camera.Target, cameraNode["Target"]);
                SetValue(camera.Up    , cameraNode["Up"]);
                scene.Cameras.push_back(std::move(camera));
            }
        }

        scene.Lights.clear();
        if (root["Lights"]) {
            for (auto const & lightNode : root["Lights"]) {
                Light light;
                SetValue(light.Type       , lightNode["Type"]);
                SetValue(light.Intensity  , lightNode["Intensity"]);
                SetValue(light.Direction  , lightNode["Direction"]);
                light.Direction = glm::normalize(light.Direction);
                SetValue(light.Position   , lightNode["Position"]);
                SetValue(light.CutOff     , lightNode["CutOff"]);
                SetValue(light.OuterCutOff, lightNode["OuterCutOff"]);
                scene.Lights.push_back(std::move(light));
            }
        }

        std::unordered_map<std::string, std::uint32_t> uniqueMaterials;

        scene.Materials.clear();
        if (root["Materials"]) {
            for (auto const & materialNode : root["Materials"]) {
                Material material;
                if (materialNode["Name"])
                    uniqueMaterials[materialNode["Name"].as<std::string>()] = std::uint32_t(scene.Materials.size());

                SetValue(material.Blend, materialNode["Blend"]);

                glm::vec4 albedoFactor(1);
                SetValue(albedoFactor, materialNode["Diffuse"]);
                SetValue(albedoFactor, materialNode["Albedo"]);
                SetValue(albedoFactor, materialNode["BaseColor"]);
                material.Albedo.Fill(albedoFactor);
                SetMap4(material.Albedo, materialNode["DiffuseMap"]  );
                SetMap4(material.Albedo, materialNode["AlbedoMap"]   );
                SetMap4(material.Albedo, materialNode["BaseColorMap"]);

                glm::vec4 metaSpecFactor(0);
                SetValue(metaSpecFactor  , materialNode["Specular"]);
                SetValue(metaSpecFactor  , materialNode["Metallic"]);
                SetValue(metaSpecFactor.a, materialNode["Shininess"]);
                SetValue(metaSpecFactor.a, materialNode["Glossiness"]);
                SetValue(metaSpecFactor.a, materialNode["Smoothness"]);
                metaSpecFactor.a /= 256;
                material.MetaSpec.Fill(metaSpecFactor);
                SetMap4(material.MetaSpec, materialNode["SpecularMap"]);
                SetMap4(material.MetaSpec, materialNode["MetallicMap"]);

                material.Height.Fill(0);
                SetMap1(material.Height, materialNode["HeightMap"]);

                scene.Materials.push_back(std::move(material));
            }
        }

        scene.Models.clear();
        if (root["Models"]) {
            for (auto const & modelNode : root["Models"]) {
                if (! modelNode["Mesh"]) continue;
                Model model;
                model.Mesh = LoadSurfaceMesh(directory / modelNode["Mesh"].as<std::string>());
                if (modelNode["Material"])
                    model.MaterialIndex = uniqueMaterials[modelNode["Material"].as<std::string>()];
                glm::vec3 translation(0.);
                glm::mat3 rotation(1.);
                glm::vec3 scale(1.);
                if (modelNode["Translation"])
                   translation = modelNode["Translation"].as<glm::vec3>();
                if (modelNode["Rotation"])
                   rotation = modelNode["Rotation"].as<glm::mat3>();
                if (modelNode["Scale"])
                   scale = modelNode["Scale"].as<glm::vec3>();
                for (auto & pos : model.Mesh.Positions) {
                  pos = translation + rotation * scale * pos;
                }
                for (auto & norm : model.Mesh.Normals) {
                  norm = rotation * glm::vec3(norm.x / scale.x, norm.y / scale.y, norm.z / scale.z);
                  norm = glm::normalize(norm);
                }
                scene.Models.push_back(std::move(model));
            }
        } 
        
        if (root["ComplexModels"]) {
            for (auto const & modelNode : root["ComplexModels"]) {
                if (! modelNode["Mesh"]) continue;
                LoadComplexModels(directory / modelNode["Mesh"].as<std::string>(), scene.Materials, scene.Models);
            }
        }

        return scene;
    }
} // namespace VCX::Engine
