#pragma once // Prevent this interface from being included more than once per translation unit.

// A screen represents one application view and its per-frame behavior.
class Screen
{
public:
    // Ensure derived screens are destroyed correctly through a base-class pointer.
    virtual ~Screen() = default;

    // Process input and update this screen using the elapsed frame time.
    virtual void Update(float deltaTime) = 0;

    // Draw this screen inside the active Raylib drawing pass.
    virtual void Draw() const = 0;
};