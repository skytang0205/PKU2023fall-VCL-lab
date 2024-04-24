#include "Assets/bundled.h"
#include "Labs/1-Drawing2D/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::Drawing2D::App>(Engine::AppContextOptions {
        .Title      = "VCX Labs 1: Drawing 2D",
        .WindowSize = { 1024, 768 },
        .FontSize   = 16,

        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
