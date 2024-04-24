#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/4-Animation/CaseMassSpring.h"
#include "Labs/4-Animation/CaseInverseKinematics.h"
#include "Labs/Common/UI.h"

namespace VCX::Labs::Animation {

    class App : public Engine::IApp {
    private:
        Common::UI             _ui;

        CaseMassSpring         _caseMassSpring;
        CaseInverseKinematics  _caseInverseKinematics;

        std::size_t        _caseId = 0;

        std::vector<std::reference_wrapper<Common::ICase>> _cases = {
            _caseInverseKinematics, _caseMassSpring
        };

    public:
        App();
        void OnFrame() override;
    };
}
