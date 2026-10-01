#include "ScreenManager.h" // Declares the manager and the abstract Screen interface.

void ScreenManager::SetScreen(std::unique_ptr<Screen> screen)
{
    // Move the new owner into the manager; unique_ptr destroys the prior screen automatically.
    activeScreen_ = std::move(screen);
}

void ScreenManager::Update(float deltaTime)
{
    // A null active screen is valid during startup/shutdown, so only dispatch when populated.
    if (activeScreen_)
    {
        // Preserve the elapsed frame time when forwarding to the current screen.
        activeScreen_->Update(deltaTime);
    }
}

void ScreenManager::Draw() const
{
    // Avoid calling virtual drawing methods when no screen is installed.
    if (activeScreen_)
    {
        // Let the active screen render without exposing its type to the application loop.
        activeScreen_->Draw();
    }
}