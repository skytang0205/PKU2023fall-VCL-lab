#include "Assets/bundled.h"
#include "Labs/2-GeometryProcessing/App.h"

int main() {
    using namespace VCX;
    return Engine::RunApp<Labs::GeometryProcessing::App>(Engine::AppContextOptions {
        .Title      = "VCX Labs 2: Geometry Processing",
        .WindowSize = { 1024, 768 },
        .FontSize   = 16,

        .IconFileNames = Assets::DefaultIcons,
        .FontFileNames = Assets::DefaultFonts,
    });
}
