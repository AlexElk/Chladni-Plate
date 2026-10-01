#include "core/ScreenManager.h"       // Provides the owner/dispatcher for application screens.
#include "screens/SimulationScreen.h" // Provides the initial Chladni simulation view.

#include <memory> // Provides std::make_unique for transferring screen ownership.

int main()
{
    // Allow users to resize the window before entering fullscreen.
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    // Create the initial desktop window and graphics context.
    InitWindow(1100, 720, "Chladni Plate");
    // Keep the controls usable when the window is made smaller.
    SetWindowMinSize(760, 520);

    // Limit presentation to 30 frames per second; physics uses its own fixed rate.
    SetTargetFPS(30);

    // The manager owns the active screen and keeps the main loop independent of screen details.
    ScreenManager screens;
    // Transfer ownership of the initial simulation screen to the manager.
    screens.SetScreen(std::make_unique<SimulationScreen>());

    // Continue until the user requests that Raylib close the window.
    while (!WindowShouldClose())
    {
        // Toggle fullscreen without changing the fixed-step physics configuration.
        if (IsKeyPressed(KEY_F11))
        {
            ToggleFullscreen();
        }

        // Pass real elapsed time to the simulation's fixed-step accumulator.
        screens.Update(GetFrameTime());

        // Begin a fresh 2D/3D drawing frame.
        BeginDrawing();
        // Let the active screen render its 3D scene and controls.
        screens.Draw();
        // Present the completed frame to the window.
        EndDrawing();
    }

    // Release the active screen before destroying the Raylib window and graphics context.
    screens.SetScreen(nullptr);
    // Destroy the window and its graphics resources after screen-owned state is released.
    CloseWindow();

    // Report successful application shutdown to the operating system.
    return 0;
}