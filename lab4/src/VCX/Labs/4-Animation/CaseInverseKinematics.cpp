#include <iostream>
#include "Engine/app.h"
#include "Labs/4-Animation/CaseInverseKinematics.h"
#include "tasks.h"
#include "Labs/Common/ImGuiHelper.h"


namespace VCX::Labs::Animation {

    BackGroundRender::BackGroundRender(IKSystem & ik_system_):
        ik_system(ik_system_),
        LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines),
        GTPointItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points),
        PointItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Points) {
        const std::uint32_t index[] = { 0, 1, 0, 2, 0, 3 };
        float pos[12] = { 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f };
        LineItem.UpdateElementBuffer(index);
        LineItem.UpdateVertexBuffer("position", std::span<std::byte>(reinterpret_cast<std::byte *>(pos), reinterpret_cast<std::byte *>(pos + 12)));
    }

    void BackGroundRender::render(Engine::GL::UniqueProgram & program) {
        LineItem.Draw({ program.Use() });
        program.GetUniforms().SetByName("u_Color", glm::vec3(0.0f, 0.8f, 0.0f));
        GTPointItem.Draw({ program.Use() });
        program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 0.0f, 0.0f));
        PointItem.Draw({ program.Use() });
    }

    BoxRenderer::BoxRenderer():
        CenterPosition(0, 0, 0),
        MainAxis(0, 1, 0),
        BoxItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Triangles),
        LineItem(Engine::GL::VertexLayout().Add<glm::vec3>("position", Engine::GL::DrawFrequency::Stream, 0), Engine::GL::PrimitiveType::Lines) {
        //     3-----2
        //    /|    /|
        //   0 --- 1 |
        //   | 7 - | 6
        //   |/    |/
        //   4 --- 5
        VertsPosition.resize(8);
        const std::vector<std::uint32_t> line_index = {0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7}; // line index
        LineItem.UpdateElementBuffer(line_index);

        const std::vector<std::uint32_t> tri_index = { 0, 1, 2, 0, 2, 3, 1, 4, 0, 1, 4, 5, 1, 6, 5, 1, 2, 6, 2, 3, 7, 2, 6, 7, 0, 3, 7, 0, 4, 7, 4, 5, 6, 4, 6, 7};
        BoxItem.UpdateElementBuffer(tri_index);
    }

    void BoxRenderer::render(Engine::GL::UniqueProgram & program)
    {
        auto span_bytes = Engine::make_span_bytes<glm::vec3>(VertsPosition);
        
        program.GetUniforms().SetByName("u_Color", glm::vec3(121.0f / 255, 207.0f / 255, 171.0f / 255));
        BoxItem.UpdateVertexBuffer("position", span_bytes);
        BoxItem.Draw({ program.Use() });

        program.GetUniforms().SetByName("u_Color", glm::vec3(1.0f, 1.0f, 1.0f));
        LineItem.UpdateVertexBuffer("position", span_bytes);
        LineItem.Draw({ program.Use() });
    }

    void BoxRenderer::calc_vert_position() {
        glm::vec3 new_y = glm::normalize(MainAxis);
        glm::quat quat = glm::rotation(glm::vec3(0, 1, 0), new_y);
        glm::vec3 new_x = quat * glm::vec3(0.5f * width, 0.0f, 0.0f);
        glm::vec3 new_z = quat * glm::vec3(0.0f, 0.0f, 0.5f * width);
        const glm::vec3 & c = CenterPosition;
        new_y *= 0.5 * length;
        VertsPosition[0] = c - new_x + new_y + new_z;
        VertsPosition[1] = c + new_x + new_y + new_z;
        VertsPosition[2] = c + new_x + new_y - new_z;
        VertsPosition[3] = c - new_x + new_y - new_z;
        VertsPosition[4] = c - new_x - new_y + new_z;
        VertsPosition[5] = c + new_x - new_y + new_z;
        VertsPosition[6] = c + new_x - new_y - new_z;
        VertsPosition[7] = c - new_x - new_y - new_z;
    }

    CaseInverseKinematics::CaseInverseKinematics():
        BackGround(ik_system),
        _program(
            Engine::GL::UniqueProgram({ Engine::GL::SharedShader("assets/shaders/flat.vert"),
                                        Engine::GL::SharedShader("assets/shaders/flat.frag") }))
    {
        _cameraManager.AutoRotate = false;
        _cameraManager.Save(_camera);
        for (int i = 0; i < 4; i++) {
            arms.emplace_back(BoxRenderer());
        }
        
        ResetSystem();
    }

    void CaseInverseKinematics::BuildByJointPosition(const std::vector<glm::vec3> & JointPosition) {
        for (int i = 0; i < JointPosition.size() - 1; i++) {
            auto MainAxis = JointPosition[i + 1] - JointPosition[i];
            arms[i].length = glm::l2Norm(MainAxis);
            arms[i].MainAxis = MainAxis / arms[i].length;
            arms[i].CenterPosition = 0.5f * (JointPosition[i] + JointPosition[i + 1]);
        }
    }

    void CaseInverseKinematics::OnSetupPropsUI() {
            const char * ik_items[] = { "ccd_ik", "fabr_ik"};
            if (ImGui::BeginCombo("Algorithm", ik_items[ik_type])) 
            {
                for (int i = 0; i < 2; i++) {
                    bool selected = i == ik_type;
                    if (ImGui::Selectable(ik_items[i], selected)) {
                        if (! selected) ik_type = i;
                    }
                }
                ImGui::EndCombo();
            }
            if (ImGui::Button(_stopped ? "Start" : "Pause")) _stopped = ! _stopped;
            ImGui::SameLine();
            if (ImGui::Button("Reset")) ResetSystem();
            if (ImGui::BeginCombo("Shape", draw_items[ik_system.TargetPositionIndex])) {
                for (int i = 0; i < 6; i++) {
                    bool selected = i == ik_system.TargetPositionIndex;
                    if (ImGui::Selectable(draw_items[i], selected)) {
                        if (! selected) {
                            ik_system.TargetPositionIndex = i;
                            ik_system.CurrIndex            = 0;
                        }
                    }
                }
                ImGui::EndCombo();
            }
    }

    Common::CaseRenderResult CaseInverseKinematics::OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) {
        _frame.Resize(desiredSize);

        _cameraManager.Update(_camera);
        _program.GetUniforms().SetByName("u_Projection", _camera.GetProjectionMatrix((float(desiredSize.first) / desiredSize.second)));
        _program.GetUniforms().SetByName("u_View", _camera.GetViewMatrix());

        gl_using(_frame);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(0.5f);
        glPointSize(4.f);

        if (!_stopped) {
            auto index  = ik_system.CurrIndex;
            auto & target = ik_system.GetTarget();
            if (ik_type == IKType::CCD_IK) {
                InverseKinematicsCCD(ik_system, target, 100, 1e-4f);
            } else if (ik_type == IKType::FABR_IK) {
                InverseKinematicsFABR(ik_system, target, 100, 1e-4f);
            }
            ik_system.EndPositionHistory[index] = ik_system.EndEffectorPosition();
            BuildByJointPosition(ik_system.JointGlobalPosition);
            auto * ptr = ik_system.EndPositionHistory.data();
            auto hist_span = std::span<const std::byte>(reinterpret_cast<const std::byte *> (ptr), reinterpret_cast<const std::byte *>(ptr + ik_system.CurrIndex));
            BackGround.PointItem.UpdateVertexBuffer("position", hist_span);
            BackGround.GTPointItem.UpdateVertexBuffer("position", Engine::make_span_bytes<const glm::vec3>(*ik_system.GetTargetPositionList()));
        }
        
        for (std::size_t i = 0; i < arms.size(); i++) {
            arms[i].calc_vert_position();
            arms[i].render(_program);
        }
        BackGround.render(_program);

        glLineWidth(1.f);
        glPointSize(1.f);
        glDisable(GL_LINE_SMOOTH);

        return Common::CaseRenderResult {
            .Fixed     = false,
            .Flipped   = true,
            .Image     = _frame.GetColorAttachment(),
            .ImageSize = desiredSize,
        };
    }

    void CaseInverseKinematics::OnProcessInput(ImVec2 const & pos) {
        _cameraManager.ProcessInput(_camera, pos);
    }

    void CaseInverseKinematics::ResetSystem() {
        ik_system.CurrIndex = 0;
        BuildByJointPosition(ik_system.InitJointPosition);
    }
} // namespace VCX::Labs::Animation
