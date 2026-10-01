#pragma once // Prevent duplicate declarations when this header is included from multiple files.

#include "Screen.h" // Use the shared contract implemented by every application screen.

#include <memory> // Store exclusive ownership of the active screen with std::unique_ptr.

// Owns the active screen and forwards the application's update and draw calls to it.
class ScreenManager
{
public:
    // Replace the current screen; passing nullptr clears it during application shutdown.
    void SetScreen(std::unique_ptr<Screen> screen); // Replace the active view and release the previous one.

    // Update the active screen when one has been installed.
    void Update(float deltaTime); // Forward elapsed time to the active view when present.

    // Draw the active screen when one has been installed.
    void Draw() const; // Forward the current frame's drawing pass to the active view.

private:
    std::unique_ptr<Screen> activeScreen_; // Own exactly one current view; nullptr means no view is active.
};