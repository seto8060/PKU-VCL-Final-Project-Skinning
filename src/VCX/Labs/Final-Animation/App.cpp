#include "Labs/Final-Animation/App.h"

namespace VCX::Labs::FinalAnimation {
    App::App():
        _ui(
            Labs::Common::UIOptions { }) {
    }

    void App::OnFrame() {
        _ui.Setup(_cases, _caseId);
    }
}

