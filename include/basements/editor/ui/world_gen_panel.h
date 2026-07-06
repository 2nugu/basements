#pragma once

#include "basements/engine/procedural/world_generator.h"
#include "basements/engine/scene_graph.h"

namespace basements {

    class WorldGenPanel {
    public:
        WorldGenPanel();
        
        // Pass the active scene graph so we can add the generated terrain to it
        void on_render(editor::SceneGraph* scene);

    private:
        HeightmapParams params;
        char name_buffer[128] = "ProceduralTerrain";
    };

}
