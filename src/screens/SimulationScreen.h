#pragma once // Make this class declaration safe to include from multiple source files.

#include "../core/Screen.h" // Inherit the application's common update/draw screen contract.

#include "raylib.h" // Supply vectors, camera state, and Raylib input/rendering types.

#include <vector> // Store particles and the cached modal displacement field.

// Shows the rotatable plate, frequency controls, and a lightweight sand simulation.
class SimulationScreen final : public Screen // The single interactive Chladni plate view.
{
public:
    // Prepare the camera and particle pool for the initial view.
    SimulationScreen(); // Configure initial defaults and allocate reusable simulation storage.

    // Handle camera input, frequency changes, simulation toggles, and particle motion.
    void Update(float deltaTime) override; // Consume input and advance fixed-step physics.

    // Render the 3D plate and the on-screen controls.
    void Draw() const override; // Render the current world and sidebar without mutating simulation state.

private:
    struct Particle
    {
        Vector3 position{};         // World-space center used for integration, contact tests, and rendering.
        Vector3 velocity{};         // World-space velocity advanced by gravity and collision impulses.
        bool active = false;        // False particles are unused slots available to the emission queue.
        bool touchingPlate = false; // Marks grains resting on the plate for redistribution and coloring.
    };

    // Update orbit-camera values from mouse drag and wheel input.
    void UpdateCamera(); // Convert yaw, pitch, and zoom distance into a Raylib perspective camera.

    // Emit grains at the plate center and advance the simulation at a fixed time step.
    void UpdateParticles(float deltaTime); // Emit queued grains and consume accumulated fixed steps.

    // Advance gravity, plate vibration, particle collisions, and plate contact for one step.
    void SimulateSubstep(float deltaTime); // Integrate every active grain once at the fixed physics interval.

    // Rebuild the broad-phase grid used to find nearby grains efficiently.
    void BuildParticleGrid(); // Map active particle indices into neighboring spatial cells.

    // Resolve overlapping grain spheres using equal-mass collision impulses.
    void ResolveParticleCollisions(); // Separate overlaps and exchange normal velocity between grain pairs.

    // Clear the previous pile and begin one new central pour.
    void ResetSimulation(); // Return all slots and timing accumulators to their initial state.

    // Randomly select half of the grains on the plate and queue them for a selected area.
    void RefeedPlateSand(float targetX, float targetZ, int percentage); // Select the requested surface fraction without replacing falling grains.

    // Convert a screen position into a plate section using a camera ray.
    bool GetPlateSectionAtScreenPosition(Vector2 screenPosition, int &sectionX, int &sectionZ) const; // Return false for misses, panel clicks, or points outside the outline.

    // Resize all particle storage when the user chooses a different preset.
    void SetParticleCount(int particleCount); // Resize particles and collision links while stopped.

    // Activate one grain above the selected feed point.
    void EmitParticle(Particle &particle, float targetX = 0.0f, float targetZ = 0.0f); // Initialize a grain in a narrow plume above the requested point.

    // Draw the frequency slider, simulation button, and current mode information.
    void DrawControls() const; // Render and label every input control in the right sidebar.

    // Draw the selected plate outline, thickness, and vibrating surface grid.
    void DrawPlate() const; // Tessellate the selected outline and displace its surface by the cached mode field.

    // Return whether an x-z position lies inside the current plate outline.
    bool IsPointInsidePlate(float x, float z) const; // Share the same shape test between grains and mouse picking.

    // Return the number of vertices used to draw the current plate outline.
    int GetPlateVertexCount() const; // Select four, five, six, or many circle-segment vertices.

    // Return one outline vertex at the requested height.
    Vector3 GetPlateVertex(int vertexIndex, float height) const; // Convert an outline index into a 3D world-space point.

    // Calculate the selected vibration mode's shape at a point on the plate.
    float GetPlateModeShape(float x, float z) const; // Bilinearly sample the cached 49-by-49 modal field.

    // Rebuild the plate field from the driven mode and nearby resonances.
    void BuildPlateModeField(int modeX, int modeZ); // Combine the driven mode with nearby damped resonances.

    // Evaluate one rectangular, circular, or polygonal basis mode.
    float EvaluatePlateBasis(float x, float z, int modeX, int modeZ) const; // Return one dimensionless approximate mode amplitude.

    // Move a grain back inside the plate and bounce it off the nearest edge.
    void ConstrainParticleToPlate(Particle &particle) const; // Project an escaped grain inward and damp outward velocity.

    // Change the outline and clear particles placed on the previous shape.
    void SetPlateShape(int shapeIndex); // Reject invalid/running changes and reset the pile after a valid choice.

    // Change physical plate dimensions and reset particles for the new geometry.
    void SetPlateSize(int sizeIndex);           // Update the characteristic side/diameter and reframe the camera.
    void SetPlateThickness(int thicknessIndex); // Update physical thickness for stiffness and visible sidewalls.

    // Return the selected plate size in meters, world units, and thickness in meters.
    float GetPlateSizeMeters() const;      // Read the selected side length or polygon/circle diameter.
    float GetPlateRadius() const;          // Convert half the selected metric dimension into scene units.
    float GetPlateThicknessMeters() const; // Read the selected physical thickness in SI units.

    // Find the plate mode whose natural frequency is closest to the selected frequency.
    void GetMode(int &modeX, int &modeZ) const; // Search the supported integer mode-index range.

    // Estimate a plate mode's natural frequency using thin-plate theory.
    float GetNaturalFrequencyHz(int modeX, int modeZ) const; // Apply the selected material, size, thickness, and outline approximation.

    Camera3D camera_{};                    // Orbit camera used to render and pick points on the 3D plate.
    int particleCount_ = 1600;             // Capacity of the particle pool selected in the sidebar.
    int materialIndex_ = 0;                // Index into the steel/aluminum/brass material table.
    int shapeIndex_ = 0;                   // Index into the square/circle/pentagon/hexagon shape table.
    int plateSizeIndex_ = 1;               // Index into the 20–40 cm plate dimension presets.
    int plateThicknessIndex_ = 1;          // Index into the 2–6 mm thickness presets.
    std::vector<Particle> particles_;      // Fixed-capacity storage reused for falling, resting, and queued grains.
    std::vector<float> plateModeField_;    // Cached scalar vibration amplitude at a 49-by-49 sample lattice.
    std::vector<int> gridHeads_;           // Head particle index for each broad-phase spatial cell.
    std::vector<int> nextParticle_;        // Linked-list next index for particles sharing a spatial cell.
    std::vector<int> reemissionQueue_;     // Resting grain indices waiting to be emitted at the selected section.
    float cameraYaw_ = 0.72f;              // Horizontal orbit angle in radians.
    float cameraPitch_ = 0.72f;            // Vertical orbit angle in radians, clamped to avoid pole flips.
    float cameraDistance_ = 6.2f;          // Camera-to-target orbit distance in scene units.
    float frequencyHz_ = 240.0f;           // User-selected drive frequency used for mode matching.
    float emissionAccumulator_ = 0.0f;     // Fractional grain count accumulated between emission events.
    float simulationAccumulator_ = 0.0f;   // Elapsed time waiting to be consumed by fixed physics steps.
    float vibrationTime_ = 0.0f;           // Phase clock for the deliberately slowed visible plate oscillation.
    float plateVibrationPhase_ = 0.0f;     // Current cosine phase shared by surface and grain forces.
    float plateVibrationAmplitude_ = 0.0f; // Normalized response scale, set to zero while stopped.
    int redistributePercent_ = 50;         // Selected fraction of resting grains to send through the feeder again.
    int emittedParticleCount_ = 0;         // Number of initial particle slots already introduced into the scene.
    int hoveredSectionX_ = -1;             // Column under the pointer, or -1 when no valid section is hovered.
    int hoveredSectionZ_ = -1;             // Row under the pointer, or -1 when no valid section is hovered.
    bool isDragging_ = false;              // Whether the left mouse button currently orbits the camera.
    bool isSimulating_ = false;            // Enables emission and fixed-step physical updates.
    bool isSelectingRefillArea_ = false;   // Routes plate clicks to the redistribution-section picker.
    bool isParticleMenuOpen_ = false;      // Indicates that the particle-count dropdown is expanded.
    bool isMaterialMenuOpen_ = false;      // Indicates that the material dropdown is expanded.
    bool isShapeMenuOpen_ = false;         // Indicates that the shape dropdown is expanded.
    bool isSizeMenuOpen_ = false;          // Indicates that the plate-size dropdown is expanded.
    bool isThicknessMenuOpen_ = false;     // Indicates that the thickness dropdown is expanded.
};