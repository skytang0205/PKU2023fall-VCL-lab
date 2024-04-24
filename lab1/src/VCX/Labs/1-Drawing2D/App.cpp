#include "Labs/1-Drawing2D/App.h"

namespace VCX::Labs::Drawing2D {
    App::App():
        _ui(
            Labs::Common::UIOptions { }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}
