#include "Assets/bundled.h"
#include "Labs/Final-Animation/App.h"

int main() {
    using namespace VCX;

    return Engine::RunApp<Labs::FinalAnimation::App>(
        Engine::AppContextOptions {
            .Title      = "VCX Final Project: Animation System",
            .WindowSize = { 1280, 800 },
            .FontSize   = 16,

            .IconFileNames = Assets::DefaultIcons,
            .FontFileNames = Assets::DefaultFonts,
        }
    );
}
