#pragma once
#include "Engine/GL/Frame.hpp"
#include "Engine/GL/Program.h"
#include "Engine/GL/RenderItem.h"
#include "Engine/loader.h"
#include "Labs/Common/ICase.h"
#include "Labs/Common/ImageRGB.h"
#include "Labs/Common/OrbitCameraManager.h"
#include "IKSystem.h"


namespace VCX::Labs::Animation {

    enum IKType {
        CCD_IK = 0,
        FABR_IK = 1
    };

    class BoxRenderer {
    public:
        BoxRenderer();

        void render(Engine::GL::UniqueProgram & program);
        void calc_vert_position();

        std::vector<glm::vec3>              VertsPosition;
        glm::vec3                           CenterPosition;
        glm::vec3                           MainAxis;
        float                               width = 0.05f;
        float                               length = 0.2f;

        Engine::GL::UniqueIndexedRenderItem BoxItem;  // render the box
        Engine::GL::UniqueIndexedRenderItem LineItem; // render line on box
    };

    // render x, y, z axis
    class BackGroundRender {
    public:
        BackGroundRender(IKSystem & ik_system_);
        void render(Engine::GL::UniqueProgram & program);

    public:
        Engine::GL::UniqueIndexedRenderItem LineItem;
        Engine::GL::UniqueRenderItem        PointItem;
        Engine::GL::UniqueRenderItem        GTPointItem;
        IKSystem &                          ik_system;
    };

    class CaseInverseKinematics : public Common::ICase {
    public:
        CaseInverseKinematics();

        virtual std::string_view const GetName() override { return "Inverse Kinematics System"; }

        virtual void                        OnSetupPropsUI() override;
        virtual Common::CaseRenderResult    OnRender(std::pair<std::uint32_t, std::uint32_t> const desiredSize) override;
        virtual void                        OnProcessInput(ImVec2 const & pos) override;

    private:
        void                                BuildByJointPosition(const std::vector<glm::vec3> & joint_pos_);
        void                                ResetSystem();

    private:
        Engine::GL::UniqueProgram           _program;
        Engine::GL::UniqueRenderFrame       _frame;
        Engine::Camera                      _camera { .Eye = glm::vec3(-3, 3, 3) };
        Common::OrbitCameraManager          _cameraManager;
        bool                                _stopped { false };

        BackGroundRender                    BackGround;
        std::vector<BoxRenderer>            arms; // for render the arm
        IKSystem                            ik_system;
        int                                 ik_type = IKType::CCD_IK;
        
    };
} // namespace VCX::Labs::Animation
