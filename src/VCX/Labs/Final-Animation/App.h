#pragma once

#include <vector>

#include "Engine/app.h"
#include "Labs/Common/UI.h"
#include "Labs/Final-Animation/CaseFinalAnimation.h"

namespace VCX::Labs::FinalAnimation {

class App : public VCX::Engine::IApp {
private:
    Common::UI _ui;

    CaseFinalAnimation _case;
    std::size_t _caseId = 0;

    std::vector<std::reference_wrapper<Common::ICase>> _cases = {
        _case
    };

public:
    App();
    void OnFrame() override;
};

}
