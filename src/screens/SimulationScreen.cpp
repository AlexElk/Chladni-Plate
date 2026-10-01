#include "SimulationScreen.h"
#include "rlgl.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
    // One record contains the material parameters used by the thin-plate frequency estimate.
    struct PlateMaterial
    {
        const char *name;      // Human-readable label shown in the material dropdown.
        float densityKgPerM3;  // Volumetric density rho in kilograms per cubic meter.
        float youngsModulusPa; // Elastic modulus E in pascals.
        float poissonsRatio;   // Dimensionless lateral-strain ratio nu.
    };

    constexpr float kPi = 3.14159265358979323846f; // Float-precision pi for mode and orbit equations.
    constexpr float kMinFrequencyHz = 40.0f;       // Lower endpoint of the drive-frequency slider.
    constexpr float kMaxFrequencyHz = 1000.0f;     // Upper endpoint of the drive-frequency slider.
    // Approximate engineering values; each selection changes the plate's flexural rigidity and mass.
    constexpr PlateMaterial kPlateMaterials[] = {
        {"Steel", 7850.0f, 200.0e9f, 0.30f},
        {"Aluminum", 2700.0f, 69.0e9f, 0.33f},
        {"Brass", 8500.0f, 100.0e9f, 0.34f}};
    constexpr int kPlateMaterialCount = 3; // Number of entries in the material preset table.
    // Labels and numeric values must remain index-aligned for the shape dropdown.
    constexpr const char *kPlateShapeNames[] = {"Square", "Circular", "Pentagonal", "Hexagonal"};
    constexpr int kPlateShapeCount = 4; // Supported outline families.
    // Metric side length for the square and diameter for circular/polygonal outlines.
    constexpr float kPlateSizeOptionsMeters[] = {0.20f, 0.25f, 0.30f, 0.35f, 0.40f};
    constexpr const char *kPlateSizeOptionLabels[] = {"20 cm", "25 cm", "30 cm", "35 cm", "40 cm"};
    // Thickness presets are stored in SI units because the resonance equations use meters.
    constexpr float kPlateThicknessOptionsMeters[] = {0.002f, 0.003f, 0.004f, 0.005f, 0.006f};
    constexpr const char *kPlateThicknessOptionLabels[] = {"2 mm", "3 mm", "4 mm", "5 mm", "6 mm"};
    constexpr int kPlateDimensionOptionCount = 5; // Shared count for the size and thickness preset arrays.
    constexpr float kWorldUnitsPerMeter = 10.0f;  // Render-space scale; physics material values remain metric.
    constexpr int kDefaultParticleCount = 1600;   // Initial pool capacity before the user chooses another preset.
    // Dropdown options define pool sizes rather than dynamically allocating grains during a pour.
    constexpr int kParticleCountOptions[] = {400, 800, 1600, 3200};
    constexpr int kParticleCountOptionTotal = 4;                          // Number of particle-count choices above.
    constexpr float kMaxPlateRadius = 0.5f * 0.40f * kWorldUnitsPerMeter; // Broad-phase bounds cover the largest preset.
    constexpr float kPlateTop = 0.08f;                                    // World-space height of the undeformed plate surface.
    constexpr float kParticleRadius = 0.014f;                             // Render/physics radius shared by every sand grain.
    constexpr float kParticleDiameter = kParticleRadius * 2.0f;           // Minimum center distance for sphere contact.
    constexpr float kGravity = 9.81f;                                     // Downward acceleration in the simulation's world scale.
    constexpr float kEmissionRate = 900.0f;                               // Feed rate in grains per simulation second.
    constexpr float kFixedTimeStep = 1.0f / 60.0f;                        // Stable 60 Hz physics interval independent of 30 FPS drawing.
    constexpr int kMaxSubsteps = 8;                                       // Bound catch-up work after a long frame or window transition.
    constexpr float kGridCellSize = kParticleDiameter;                    // A neighboring-cell search then covers touching grains.
    // These grid dimensions are compile-time capacity bounds; active plates occupy only part of the grid.
    constexpr int kGridColumnsX = static_cast<int>((2.0f * kMaxPlateRadius) / kGridCellSize) + 1;
    constexpr int kGridColumnsZ = static_cast<int>((2.0f * kMaxPlateRadius) / kGridCellSize) + 1;
    constexpr int kPlateSections = 3;        // The interactive redistribution overlay is a 3-by-3 grid.
    constexpr float kSidebarWidth = 320.0f;  // Fixed control-rail width in screen pixels.
    constexpr float kSidebarPadding = 24.0f; // Shared horizontal inset for labels and controls.
    // Vertical positions are authored for a 720-pixel-high window and scaled at draw/input time.
    constexpr float kParticleSelectorY = 195.0f;
    constexpr float kParticleSelectorHeight = 36.0f;
    constexpr float kMaterialSelectorY = 260.0f;
    constexpr float kMaterialSelectorHeight = 36.0f;
    constexpr float kShapeSelectorY = 325.0f;
    constexpr float kShapeSelectorHeight = 36.0f;
    constexpr float kSizeSelectorY = 390.0f;
    constexpr float kThicknessSelectorY = 455.0f;
    constexpr int kCircleSegments = 48;      // Polygonal approximation used to draw the circular edge.
    constexpr int kModeFieldResolution = 49; // Cached field contains 49 samples on each plate axis.
    // Neutral interface colors are separated from the material-specific colors used on the plate.
    constexpr int kDimensionOptionRowHeight = 24;
    constexpr Color kUiBackdrop{24, 27, 30, 255};
    constexpr Color kUiPanel{19, 22, 25, 250};
    constexpr Color kUiPanelEdge{62, 67, 72, 255};
    constexpr Color kUiControl{34, 38, 42, 255};
    constexpr Color kUiControlEdge{75, 81, 87, 255};
    constexpr Color kUiText{222, 224, 226, 255};
    constexpr Color kUiMuted{145, 151, 157, 255};
    constexpr Color kUiAccent{132, 155, 174, 255};
    constexpr Color kUiAccentHover{151, 171, 188, 255};
    constexpr float kParticleOptionRowHeight = 24.0f;
}

SimulationScreen::SimulationScreen()
    : particleCount_(kDefaultParticleCount),
      particles_(particleCount_),
      plateModeField_(kModeFieldResolution * kModeFieldResolution, 0.0f), // Allocate the modal cache once.
      gridHeads_(kGridColumnsX * kGridColumnsZ, -1),
      nextParticle_(particleCount_, -1)
{
    // Aim the initial view at the plate before the first frame is drawn.
    camera_.target = Vector3{0.0f, 0.0f, 0.0f};
    // Define world up so yaw and pitch orbit around a stable vertical axis.
    camera_.up = Vector3{0.0f, 1.0f, 0.0f};
    // Use a perspective field of view rather than an orthographic projection.
    camera_.fovy = 40.0f;
    // Select perspective camera projection in Raylib.
    camera_.projection = CAMERA_PERSPECTIVE;
    // Derive the initial camera position from the default orbit parameters.
    UpdateCamera();
}

void SimulationScreen::Update(float deltaTime)
{
    // Sample current mouse state once so every control in this frame uses identical coordinates.
    const Vector2 mousePosition = GetMousePosition();
    // Read live dimensions so fullscreen and window resizing keep the sidebar attached to the right edge.
    const float screenWidth = static_cast<float>(GetScreenWidth());
    // Compress the authored layout vertically only when the window is shorter than its design height.
    const float layoutScale = std::min(1.0f, static_cast<float>(GetScreenHeight()) / 720.0f);
    // Reserve a fixed-width screen-space region for controls; the rest remains the 3D workspace.
    const float sidebarLeft = screenWidth - kSidebarWidth;
    // Align both sliders to the same usable horizontal track.
    const float sliderLeft = sidebarLeft + kSidebarPadding;
    const float sliderRight = screenWidth - kSidebarPadding;
    // Convert the frequency slider's design coordinate to the current window height.
    const float sliderY = 154.0f * layoutScale;
    // Convert the redistribution slider's design coordinate using the same scale as its rendering.
    const float redistributeSliderY = 525.0f * layoutScale;
    // Hit-test area for requesting a plate section before grains can be re-emitted.
    const Rectangle refillButtonBounds{
        sidebarLeft + kSidebarPadding, 555.0f * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, 40.0f * layoutScale};
    // Hit-test area for starting a fresh pour or stopping the current run.
    const Rectangle simulationButtonBounds{
        sidebarLeft + kSidebarPadding, 610.0f * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, 48.0f * layoutScale};
    // Build input bounds for each dropdown using the exact positions used by DrawControls.
    const Rectangle particleSelectorBounds{
        sidebarLeft + kSidebarPadding, kParticleSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kParticleSelectorHeight * layoutScale};
    const Rectangle materialSelectorBounds{
        sidebarLeft + kSidebarPadding, kMaterialSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kMaterialSelectorHeight * layoutScale};
    const Rectangle shapeSelectorBounds{
        sidebarLeft + kSidebarPadding, kShapeSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kShapeSelectorHeight * layoutScale};
    const Rectangle sizeSelectorBounds{
        sidebarLeft + kSidebarPadding, kSizeSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kShapeSelectorHeight * layoutScale};
    const Rectangle thicknessSelectorBounds{
        sidebarLeft + kSidebarPadding, kThicknessSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kShapeSelectorHeight * layoutScale};

    // Escape or the right mouse button leaves area-selection mode without changing the pile.
    if (isSelectingRefillArea_ &&
        (IsKeyPressed(KEY_ESCAPE) || IsMouseButtonPressed(MOUSE_BUTTON_RIGHT)))
    {
        isSelectingRefillArea_ = false;
        hoveredSectionX_ = -1;
        hoveredSectionZ_ = -1;
    }

    // Route mouse clicks to the active modal interaction before considering camera drag.
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT))
    {
        if (isSelectingRefillArea_)
        {
            int sectionX = -1;
            int sectionZ = -1;
            if (GetPlateSectionAtScreenPosition(mousePosition, sectionX, sectionZ))
            {
                const float plateRadius = GetPlateRadius();
                const float sectionWidth = 2.0f * plateRadius / kPlateSections;
                float targetX = -plateRadius + (sectionX + 0.5f) * sectionWidth;
                float targetZ = -plateRadius + (sectionZ + 0.5f) * sectionWidth;
                while (!IsPointInsidePlate(targetX, targetZ))
                {
                    targetX *= 0.9f;
                    targetZ *= 0.9f;
                }
                RefeedPlateSand(targetX, targetZ, redistributePercent_);
                isSelectingRefillArea_ = false;
                hoveredSectionX_ = -1;
                hoveredSectionZ_ = -1;
            }
            isDragging_ = false;
        }
        else
        {
            isDragging_ = mousePosition.x < sidebarLeft;
            bool particleMenuHandled = false;

            // A click in an open dropdown either chooses an option or dismisses that menu.
            if ((isParticleMenuOpen_ || isMaterialMenuOpen_ || isShapeMenuOpen_ ||
                 isSizeMenuOpen_ || isThicknessMenuOpen_) &&
                !isSimulating_)
            {
                // Identify which selector owns the currently open option list.
                const bool selectingParticleCount = isParticleMenuOpen_;
                const bool selectingMaterial = isMaterialMenuOpen_;
                const bool selectingShape = isShapeMenuOpen_;
                // Reuse the same row-layout logic for all five dropdowns.
                const Rectangle activeSelector = selectingParticleCount
                                                     ? particleSelectorBounds
                                                 : selectingMaterial ? materialSelectorBounds
                                                 : selectingShape    ? shapeSelectorBounds
                                                 : isSizeMenuOpen_   ? sizeSelectorBounds
                                                                     : thicknessSelectorBounds;
                // Determine the number of rows before checking the pointer against each row.
                const int optionCount = selectingParticleCount
                                            ? kParticleCountOptionTotal
                                        : selectingMaterial ? kPlateMaterialCount
                                        : selectingShape    ? kPlateShapeCount
                                                            : kPlateDimensionOptionCount;
                // Match option height to the compact window layout.
                const float optionRowHeight = kParticleOptionRowHeight * layoutScale;
                const float optionsTop = activeSelector.y + activeSelector.height;
                for (int optionIndex = 0; optionIndex < optionCount; ++optionIndex)
                {
                    const Rectangle optionBounds{
                        activeSelector.x,
                        optionsTop + optionIndex * optionRowHeight,
                        activeSelector.width,
                        optionRowHeight};
                    // Apply the selected preset and close every dropdown state.
                    if (CheckCollisionPointRec(mousePosition, optionBounds))
                    {
                        if (selectingParticleCount)
                        {
                            SetParticleCount(kParticleCountOptions[optionIndex]);
                        }
                        else
                        {
                            if (selectingMaterial)
                            {
                                materialIndex_ = optionIndex;
                            }
                            else if (selectingShape)
                            {
                                SetPlateShape(optionIndex);
                            }
                            else if (isSizeMenuOpen_)
                            {
                                SetPlateSize(optionIndex);
                            }
                            else
                            {
                                SetPlateThickness(optionIndex);
                            }
                        }
                        isParticleMenuOpen_ = false;
                        isMaterialMenuOpen_ = false;
                        isShapeMenuOpen_ = false;
                        isSizeMenuOpen_ = false;
                        isThicknessMenuOpen_ = false;
                        particleMenuHandled = true;
                        break;
                    }
                }

                if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, activeSelector))
                {
                    isParticleMenuOpen_ = false;
                    isMaterialMenuOpen_ = false;
                    isShapeMenuOpen_ = false;
                    isSizeMenuOpen_ = false;
                    isThicknessMenuOpen_ = false;
                    particleMenuHandled = true;
                }
                else if (!particleMenuHandled)
                {
                    isParticleMenuOpen_ = false;
                    isMaterialMenuOpen_ = false;
                    isShapeMenuOpen_ = false;
                    isSizeMenuOpen_ = false;
                    isThicknessMenuOpen_ = false;
                }
            }

            // Opening one selector always closes the other dropdowns.
            if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, particleSelectorBounds))
            {
                isDragging_ = false;
                isParticleMenuOpen_ = !isSimulating_;
                isMaterialMenuOpen_ = false;
                isShapeMenuOpen_ = false;
                isSizeMenuOpen_ = false;
                isThicknessMenuOpen_ = false;
                particleMenuHandled = true;
            }
            if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, sizeSelectorBounds))
            {
                isDragging_ = false;
                isSizeMenuOpen_ = !isSimulating_;
                isParticleMenuOpen_ = false;
                isMaterialMenuOpen_ = false;
                isShapeMenuOpen_ = false;
                isThicknessMenuOpen_ = false;
                particleMenuHandled = true;
            }
            if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, thicknessSelectorBounds))
            {
                isDragging_ = false;
                isThicknessMenuOpen_ = !isSimulating_;
                isParticleMenuOpen_ = false;
                isMaterialMenuOpen_ = false;
                isShapeMenuOpen_ = false;
                isSizeMenuOpen_ = false;
                particleMenuHandled = true;
            }
            if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, materialSelectorBounds))
            {
                isDragging_ = false;
                isMaterialMenuOpen_ = !isSimulating_;
                isParticleMenuOpen_ = false;
                isShapeMenuOpen_ = false;
                isSizeMenuOpen_ = false;
                isThicknessMenuOpen_ = false;
                particleMenuHandled = true;
            }
            if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, shapeSelectorBounds))
            {
                isDragging_ = false;
                isShapeMenuOpen_ = !isSimulating_;
                isParticleMenuOpen_ = false;
                isMaterialMenuOpen_ = false;
                isSizeMenuOpen_ = false;
                isThicknessMenuOpen_ = false;
                particleMenuHandled = true;
            }
            if (particleMenuHandled)
            {
                isDragging_ = false;
            }

            // Convert horizontal slider position to a clamped 0-to-1 frequency ratio.
            if (!particleMenuHandled && mousePosition.x >= sidebarLeft &&
                mousePosition.y >= sliderY - 10.0f &&
                mousePosition.y <= sliderY + 10.0f && mousePosition.x >= sliderLeft - 8.0f &&
                mousePosition.x <= sliderRight + 8.0f)
            {
                const float sliderRatio = std::clamp(
                    (mousePosition.x - sliderLeft) / (sliderRight - sliderLeft), 0.0f, 1.0f);
                frequencyHz_ = kMinFrequencyHz + sliderRatio * (kMaxFrequencyHz - kMinFrequencyHz);
            }

            // Snap redistribution to ten-percent increments rather than arbitrary fractions.
            if (!particleMenuHandled && mousePosition.x >= sidebarLeft &&
                std::abs(mousePosition.y - redistributeSliderY) <= 12.0f &&
                mousePosition.x >= sliderLeft - 8.0f && mousePosition.x <= sliderRight + 8.0f)
            {
                const float ratio = std::clamp(
                    (mousePosition.x - sliderLeft) / (sliderRight - sliderLeft), 0.0f, 1.0f);
                redistributePercent_ = 10 * (1 + static_cast<int>(std::round(ratio * 9.0f)));
            }

            // The redistribute command only starts a plate-section pick when surface grains exist.
            if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, refillButtonBounds))
            {
                const bool hasPlateSand = std::any_of(
                    particles_.begin(), particles_.end(),
                    [](const Particle &particle)
                    { return particle.active && particle.touchingPlate; });
                if (isSimulating_ && hasPlateSand)
                {
                    isSelectingRefillArea_ = true;
                }
            }
            // Starting resets the old pile; stopping preserves it for inspection.
            else if (!particleMenuHandled && CheckCollisionPointRec(mousePosition, simulationButtonBounds))
            {
                isSimulating_ = !isSimulating_;
                isSelectingRefillArea_ = false;
                isParticleMenuOpen_ = false;
                isMaterialMenuOpen_ = false;
                isShapeMenuOpen_ = false;
                isSizeMenuOpen_ = false;
                isThicknessMenuOpen_ = false;
                if (isSimulating_)
                {
                    ResetSimulation();
                }
            }
        }
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT))
    {
        isDragging_ = false;
    }

    if (!isSelectingRefillArea_ && !isParticleMenuOpen_ && IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
        mousePosition.x >= sidebarLeft && mousePosition.y >= sliderY - 10.0f &&
        mousePosition.y <= sliderY + 10.0f &&
        mousePosition.x >= sliderLeft - 8.0f && mousePosition.x <= sliderRight + 8.0f)
    {
        const float sliderRatio = std::clamp(
            (mousePosition.x - sliderLeft) / (sliderRight - sliderLeft), 0.0f, 1.0f);
        frequencyHz_ = kMinFrequencyHz + sliderRatio * (kMaxFrequencyHz - kMinFrequencyHz);
    }

    if (!isSelectingRefillArea_ && !isParticleMenuOpen_ && IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
        mousePosition.x >= sidebarLeft && std::abs(mousePosition.y - redistributeSliderY) <= 12.0f &&
        mousePosition.x >= sliderLeft - 8.0f && mousePosition.x <= sliderRight + 8.0f)
    {
        const float ratio = std::clamp(
            (mousePosition.x - sliderLeft) / (sliderRight - sliderLeft), 0.0f, 1.0f);
        redistributePercent_ = 10 * (1 + static_cast<int>(std::round(ratio * 9.0f)));
    }

    if (isSelectingRefillArea_)
    {
        if (!GetPlateSectionAtScreenPosition(mousePosition, hoveredSectionX_, hoveredSectionZ_))
        {
            hoveredSectionX_ = -1;
            hoveredSectionZ_ = -1;
        }
    }

    // Apply orbit/zoom input once before the scene is rendered.
    UpdateCamera();
    if (isSimulating_)
    {
        // Advance emission and physics only while the user has started the simulation.
        UpdateParticles(deltaTime);
    }
    else
    {
        // Remove any frozen mesh displacement when the simulation is paused or stopped.
        plateVibrationAmplitude_ = 0.0f;
    }
}

void SimulationScreen::Draw() const
{
    // Clear the full backbuffer with the neutral workspace color before drawing world geometry.
    ClearBackground(kUiBackdrop);

    // Enter Raylib's 3D camera transform for the plate, grain spheres, and section overlay.
    BeginMode3D(camera_);

    // Render the selected shape, thickness, material color, and deformed modal surface.
    DrawPlate();

    // The green/steel selection tiles appear only during the redistribution picking step.
    if (isSelectingRefillArea_)
    {
        // Divide the current plate bounding square into equal cells for mouse targeting.
        const float plateRadius = GetPlateRadius();
        const float sectionWidth = 2.0f * plateRadius / kPlateSections;
        const float sectionDepth = sectionWidth;
        for (int sectionZ = 0; sectionZ < kPlateSections; ++sectionZ)
        {
            for (int sectionX = 0; sectionX < kPlateSections; ++sectionX)
            {
                const float centerX = -plateRadius + (sectionX + 0.5f) * sectionWidth;
                const float centerZ = -plateRadius + (sectionZ + 0.5f) * sectionDepth;
                // Omit cells whose center is outside a circle or polygon.
                if (!IsPointInsidePlate(centerX, centerZ))
                {
                    continue;
                }
                // Compare grid indices with the raycast result to highlight the hovered valid cell.
                const bool isHovered = sectionX == hoveredSectionX_ && sectionZ == hoveredSectionZ_;
                const Vector3 sectionCenter{centerX, kPlateTop + 0.004f, centerZ};
                const Color fillColor = isHovered ? Color{112, 139, 160, 105} : Color{92, 110, 126, 42};
                const Color outlineColor = isHovered ? kUiAccentHover : Color{115, 130, 143, 190};
                DrawCube(sectionCenter, sectionWidth - 0.018f, 0.008f,
                         sectionDepth - 0.018f, fillColor);
                DrawCubeWires(sectionCenter, sectionWidth - 0.018f, 0.01f,
                              sectionDepth - 0.018f, outlineColor);
            }
        }
    }

    // Draw only allocated grains; unused pool slots have no visible representation.
    for (const Particle &particle : particles_)
    {
        if (particle.active)
        {
            const Color grainColor = particle.touchingPlate ? Color{244, 190, 105, 255} : Color{237, 226, 196, 255};
            DrawSphere(particle.position, kParticleRadius, grainColor);
        }
    }

    // Restore screen-space drawing before painting the title, prompt, and sidebar controls.
    EndMode3D();

    DrawText("CHLADNI PLATE", 28, 24, 20, Color{218, 232, 229, 255});
    DrawText("Drag to rotate  |  Mouse wheel to zoom", 28, 51, 16, Color{143, 164, 167, 255});
    if (isSelectingRefillArea_)
    {
        DrawRectangle(24, 78, 348, 30, kUiPanel);
        DrawText("SELECT A SECTION  |  ESC OR RIGHT CLICK TO CANCEL", 34, 86, 12,
                 kUiText);
    }
    DrawControls();
}

void SimulationScreen::UpdateCamera()
{
    // Convert pointer displacement into orbit angles while a scene drag is active.
    if (isDragging_)
    {
        const Vector2 mouseDelta = GetMouseDelta();
        cameraYaw_ -= mouseDelta.x * 0.006f;
        cameraPitch_ = std::clamp(cameraPitch_ + mouseDelta.y * 0.006f, 0.12f, 1.42f);
    }

    // Zoom with the wheel and clamp distance so the camera cannot enter or lose the plate.
    cameraDistance_ = std::clamp(cameraDistance_ - GetMouseWheelMove() * 0.45f, 3.4f, 10.0f);

    // Offset the target so the plate stays centered in the workspace beside the sidebar.
    camera_.target = Vector3{
        std::cos(cameraYaw_) * 1.0f,
        0.0f,
        -std::sin(cameraYaw_) * 1.0f};

    // Convert orbit angles into a camera position around the shifted plate target.
    camera_.position = Vector3{
        camera_.target.x + std::sin(cameraYaw_) * std::cos(cameraPitch_) * cameraDistance_,
        camera_.target.y + std::sin(cameraPitch_) * cameraDistance_,
        camera_.target.z + std::cos(cameraYaw_) * std::cos(cameraPitch_) * cameraDistance_};
}

void SimulationScreen::UpdateParticles(float deltaTime)
{
    // Clamp large frame gaps so resuming after a pause does not create unbounded catch-up work.
    const float frameTime = std::clamp(deltaTime, 0.0f, 0.05f);

    // Feed grains steadily from one point instead of recycling them after every impact.
    // Add fractional grains proportional to elapsed time and the configured grains-per-second rate.
    emissionAccumulator_ += frameTime * kEmissionRate;
    while (emissionAccumulator_ >= 1.0f &&
           (!reemissionQueue_.empty() || emittedParticleCount_ < particleCount_))
    {
        // Reuse a queued resting grain before consuming a never-used particle slot.
        if (!reemissionQueue_.empty())
        {
            const int particleIndex = reemissionQueue_.back();
            reemissionQueue_.pop_back();
            Particle &particle = particles_[particleIndex];
            const float targetX = particle.position.x;
            const float targetZ = particle.position.z;
            EmitParticle(particle, targetX, targetZ);
        }
        else
        {
            EmitParticle(particles_[emittedParticleCount_]);
            ++emittedParticleCount_;
        }
        // One whole accumulated grain has now been emitted.
        emissionAccumulator_ -= 1.0f;
    }

    // Fixed-size steps keep collisions stable when frame time varies.
    // Carry fractional elapsed time until a complete physics interval is available.
    simulationAccumulator_ += frameTime;
    int substepCount = 0;
    while (simulationAccumulator_ >= kFixedTimeStep && substepCount < kMaxSubsteps)
    {
        // Run exactly one deterministic integration interval per loop iteration.
        SimulateSubstep(kFixedTimeStep);
        simulationAccumulator_ -= kFixedTimeStep;
        ++substepCount;
    }

    // Drop excessive backlog rather than allowing a slow frame to trigger a long catch-up pause.
    if (substepCount == kMaxSubsteps)
    {
        // Retain at most one interval after hitting the work cap to avoid a spiral of death.
        simulationAccumulator_ = std::min(simulationAccumulator_, kFixedTimeStep);
    }
}

void SimulationScreen::SimulateSubstep(float deltaTime)
{
    // Select the discrete mode whose estimated natural frequency is nearest the drive frequency.
    int modeX = 1;
    int modeZ = 1;
    GetMode(modeX, modeZ);
    // Rebuild one shared spatial field so grains and the rendered surface use identical nodes.
    BuildPlateModeField(modeX, modeZ);

    // Compare drive frequency with the selected mode's natural frequency.
    const float naturalFrequencyHz = GetNaturalFrequencyHz(modeX, modeZ);
    // r = f / f_n is the dimensionless frequency ratio used by a forced oscillator.
    const float frequencyRatio = frequencyHz_ / naturalFrequencyHz;
    // Detuning is zero at resonance and grows as the drive moves away from the natural frequency.
    const float detuning = 1.0f - frequencyRatio * frequencyRatio;
    // This damping ratio is a visual/model parameter, not a measurement of a real plate fixture.
    constexpr float kDampingRatio = 0.035f;
    // The denominator combines detuning and damping in the standard second-order response magnitude.
    const float responseDenominator = std::sqrt(
        detuning * detuning +
        4.0f * kDampingRatio * kDampingRatio * frequencyRatio * frequencyRatio);
    // Clamp amplification to keep the explicit particle integration numerically manageable.
    const float resonanceResponse = std::clamp(
        frequencyRatio * frequencyRatio / responseDenominator, 0.02f, 1.0f);

    // Slow the drive for visible motion while preserving each material's resonant response.
    // Normalize the UI range before mapping it to a slow, visible animation clock.
    const float normalizedFrequency = (frequencyHz_ - kMinFrequencyHz) /
                                      (kMaxFrequencyHz - kMinFrequencyHz);
    // Use 2–8 visual cycles per second instead of attempting to animate hundreds of hertz directly.
    const float simulatedCyclesPerSecond = 2.0f + normalizedFrequency * 6.0f;
    // Advance phase in radians using omega = 2*pi*f*dt.
    vibrationTime_ += 2.0f * kPi * simulatedCyclesPerSecond * deltaTime;
    // Cosine phase is shared by vertical grain forcing and plate-surface displacement.
    plateVibrationPhase_ = std::cos(vibrationTime_);
    // Scale visible movement with the resonant response while retaining a small off-resonance motion.
    plateVibrationAmplitude_ = 0.25f + resonanceResponse * 1.75f;
    // Stronger resonance increases the lateral drift that gathers grains toward nodes.
    const float nodalDriftStrength = 0.4f + resonanceResponse * 1.8f;

    // Integrate each allocated grain independently before resolving pair contacts.
    for (Particle &particle : particles_)
    {
        if (!particle.active)
        {
            continue;
        }

        // Read local signed mode amplitude; its zeros define the approximate nodal lines.
        const float modeShape = GetPlateModeShape(particle.position.x, particle.position.z);
        // A grain touches the top face when its center reaches the surface plus its radius.
        const float contactHeight = kPlateTop + kParticleRadius;
        // Apply vibration forces only near the plate, not during most of the free-fall path.
        const bool nearPlate = particle.position.y <= contactHeight + kParticleRadius * 2.0f;

        // Plate motion drives grains near the surface away from antinodes toward nodal lines.
        if (nearPlate)
        {
            // Central differences estimate d(mode)/dx and d(mode)/dz from the cached field.
            constexpr float kGradientStep = 0.001f;
            const float amplitudeGradientX =
                (GetPlateModeShape(particle.position.x + kGradientStep,
                                   particle.position.z) -
                 GetPlateModeShape(particle.position.x - kGradientStep,
                                   particle.position.z)) /
                (2.0f * kGradientStep);
            const float amplitudeGradientZ =
                (GetPlateModeShape(particle.position.x,
                                   particle.position.z + kGradientStep) -
                 GetPlateModeShape(particle.position.x,
                                   particle.position.z - kGradientStep)) /
                (2.0f * kGradientStep);
            // The gradient of mode^2 is 2*mode*grad(mode); moving downhill steers grains toward nodes.
            particle.velocity.x -= 2.0f * modeShape * amplitudeGradientX * nodalDriftStrength * deltaTime;
            particle.velocity.z -= 2.0f * modeShape * amplitudeGradientZ * nodalDriftStrength * deltaTime;
            particle.velocity.y += 14.0f * modeShape * plateVibrationPhase_ *
                                   plateVibrationAmplitude_ * deltaTime;
        }

        // Semi-implicit Euler: update velocity under gravity before advancing position.
        particle.velocity.y -= kGravity * deltaTime;
        particle.position.x += particle.velocity.x * deltaTime;
        particle.position.y += particle.velocity.y * deltaTime;
        particle.position.z += particle.velocity.z * deltaTime;

        // Clear contact state first, then set it only if this integration step hits the top face.
        particle.touchingPlate = false;
        if (particle.position.y <= contactHeight)
        {
            // Project the sphere onto the plate so it cannot tunnel below the top surface.
            particle.position.y = contactHeight;
            particle.touchingPlate = true;
            if (particle.velocity.y < 0.0f)
            {
                // Stop tiny rebounds; otherwise reflect downward speed with low restitution.
                particle.velocity.y = std::abs(particle.velocity.y) < 0.35f
                                          ? 0.0f
                                          : -particle.velocity.y * 0.08f;
            }
            // Exponential damping remains stable when the fixed physics step changes.
            const float surfaceFriction = std::exp(-2.5f * deltaTime);
            particle.velocity.x *= surfaceFriction;
            particle.velocity.z *= surfaceFriction;
        }

        // Keep grains inside the selected outline and reflect them from its nearest edge.
        if (!IsPointInsidePlate(particle.position.x, particle.position.z))
        {
            ConstrainParticleToPlate(particle);
        }
    }

    // Build the broad phase from new positions before checking grain-grain contacts.
    BuildParticleGrid();
    // Correct overlaps and exchange velocity along each contact normal.
    ResolveParticleCollisions();
}

void SimulationScreen::BuildParticleGrid()
{
    // Mark every spatial cell empty before inserting this step's active grains.
    std::fill(gridHeads_.begin(), gridHeads_.end(), -1);

    for (int particleIndex = 0; particleIndex < particleCount_; ++particleIndex)
    {
        const Particle &particle = particles_[particleIndex];
        if (!particle.active)
        {
            continue;
        }

        // Shift world coordinates into nonnegative cell indices and clamp to maximum plate bounds.
        const int cellX = std::clamp(
            static_cast<int>((particle.position.x + kMaxPlateRadius) / kGridCellSize),
            0, kGridColumnsX - 1);
        const int cellZ = std::clamp(
            static_cast<int>((particle.position.z + kMaxPlateRadius) / kGridCellSize),
            0, kGridColumnsZ - 1);
        // Flatten the 2D grid coordinate into one vector index.
        const int cellIndex = cellZ * kGridColumnsX + cellX;
        nextParticle_[particleIndex] = gridHeads_[cellIndex];
        gridHeads_[cellIndex] = particleIndex;
    }
}

void SimulationScreen::ResolveParticleCollisions()
{
    // Squared distance avoids square roots for pairs that are not touching.
    const float minimumDistanceSquared = kParticleDiameter * kParticleDiameter;

    for (int firstIndex = 0; firstIndex < particleCount_; ++firstIndex)
    {
        Particle &first = particles_[firstIndex];
        if (!first.active)
        {
            continue;
        }

        const int cellX = std::clamp(
            static_cast<int>((first.position.x + kMaxPlateRadius) / kGridCellSize),
            0, kGridColumnsX - 1);
        const int cellZ = std::clamp(
            static_cast<int>((first.position.z + kMaxPlateRadius) / kGridCellSize),
            0, kGridColumnsZ - 1);

        // A grain can only touch particles in its own cell or one of the eight adjacent cells.
        for (int offsetZ = -1; offsetZ <= 1; ++offsetZ)
        {
            for (int offsetX = -1; offsetX <= 1; ++offsetX)
            {
                const int neighborX = cellX + offsetX;
                const int neighborZ = cellZ + offsetZ;
                if (neighborX < 0 || neighborX >= kGridColumnsX ||
                    neighborZ < 0 || neighborZ >= kGridColumnsZ)
                {
                    continue;
                }

                const int cellIndex = neighborZ * kGridColumnsX + neighborX;
                for (int secondIndex = gridHeads_[cellIndex]; secondIndex != -1;
                     secondIndex = nextParticle_[secondIndex])
                {
                    // Handle each unordered pair once and skip self/earlier pairs.
                    if (secondIndex <= firstIndex)
                    {
                        continue;
                    }

                    Particle &second = particles_[secondIndex];
                    float differenceX = second.position.x - first.position.x;
                    float differenceY = second.position.y - first.position.y;
                    float differenceZ = second.position.z - first.position.z;
                    float distanceSquared = differenceX * differenceX +
                                            differenceY * differenceY +
                                            differenceZ * differenceZ;
                    // Ignore pairs whose sphere centers are already far enough apart.
                    if (distanceSquared >= minimumDistanceSquared)
                    {
                        continue;
                    }

                    // Separate overlapping equal-mass spheres along their contact normal.
                    float distance = 0.0f;
                    if (distanceSquared < 1.0e-8f)
                    {
                        differenceX = 1.0f;
                        differenceY = 0.0f;
                        differenceZ = 0.0f;
                        distance = 1.0f;
                    }
                    else
                    {
                        distance = std::sqrt(distanceSquared);
                    }

                    // Normalize the center-to-center vector to get the collision direction.
                    const float normalX = differenceX / distance;
                    const float normalY = differenceY / distance;
                    const float normalZ = differenceZ / distance;
                    // Split penetration correction equally because both grains have equal mass.
                    const float correction = (kParticleDiameter - distance) * 0.5f;
                    first.position.x -= normalX * correction;
                    first.position.y -= normalY * correction;
                    first.position.z -= normalZ * correction;
                    second.position.x += normalX * correction;
                    second.position.y += normalY * correction;
                    second.position.z += normalZ * correction;

                    const float relativeVelocityX = second.velocity.x - first.velocity.x;
                    const float relativeVelocityY = second.velocity.y - first.velocity.y;
                    const float relativeVelocityZ = second.velocity.z - first.velocity.z;
                    // A negative normal relative speed means the grains are moving toward each other.
                    const float closingSpeed = relativeVelocityX * normalX +
                                               relativeVelocityY * normalY +
                                               relativeVelocityZ * normalZ;
                    if (closingSpeed < 0.0f)
                    {
                        // Apply a low-restitution equal-mass impulse along the normal only.
                        const float impulse = -(1.0f + 0.08f) * closingSpeed * 0.5f;
                        first.velocity.x -= normalX * impulse;
                        first.velocity.y -= normalY * impulse;
                        first.velocity.z -= normalZ * impulse;
                        second.velocity.x += normalX * impulse;
                        second.velocity.y += normalY * impulse;
                        second.velocity.z += normalZ * impulse;
                    }
                }
            }
        }
    }
}

void SimulationScreen::ResetSimulation()
{
    // Reset every slot, including inactive ones, to remove all prior positions and velocities.
    for (Particle &particle : particles_)
    {
        particle = Particle{};
    }
    // Restart the initial feeder and discard fractional emission/physics time.
    emittedParticleCount_ = 0;
    emissionAccumulator_ = 0.0f;
    simulationAccumulator_ = 0.0f;
    vibrationTime_ = 0.0f;
    plateVibrationPhase_ = 0.0f;
    plateVibrationAmplitude_ = 0.0f;
    reemissionQueue_.clear();
}

int SimulationScreen::GetPlateVertexCount() const
{
    // A circle uses a polygon approximation; regular polygons use their literal side count.
    if (shapeIndex_ == 1)
    {
        return kCircleSegments;
    }
    if (shapeIndex_ == 2)
    {
        return 5;
    }
    if (shapeIndex_ == 3)
    {
        return 6;
    }
    return 4;
}

Vector3 SimulationScreen::GetPlateVertex(int vertexIndex, float height) const
{
    // Convert the selected metric size to its scene-space bounding radius.
    const float radius = GetPlateRadius();
    if (shapeIndex_ == 0)
    {
        // The square is axis-aligned and uses radius as its half-side length.
        const Vector2 squareVertices[] = {
            {-radius, -radius},
            {radius, -radius},
            {radius, radius},
            {-radius, radius}};
        const Vector2 &vertex = squareVertices[vertexIndex % 4];
        return Vector3{vertex.x, height, vertex.y};
    }

    // Regular polygon/circle vertices start at the bottom and proceed evenly around 2*pi.
    const int vertexCount = GetPlateVertexCount();
    const float angle = -0.5f * kPi + 2.0f * kPi * vertexIndex / vertexCount;
    return Vector3{
        std::cos(angle) * radius,
        height,
        std::sin(angle) * radius};
}

bool SimulationScreen::IsPointInsidePlate(float x, float z) const
{
    // Reuse the same selected size for point tests in physics and mouse picking.
    const float radius = GetPlateRadius();
    if (shapeIndex_ == 0)
    {
        // Axis-aligned square containment is a pair of interval comparisons.
        return std::abs(x) <= radius && std::abs(z) <= radius;
    }
    if (shapeIndex_ == 1)
    {
        // Circle containment compares squared distance with squared radius.
        return x * x + z * z <= radius * radius;
    }

    // Ray-crossing parity test handles the regular pentagon and hexagon outlines.
    bool isInside = false;
    const int vertexCount = GetPlateVertexCount();
    for (int vertexIndex = 0, previousIndex = vertexCount - 1;
         vertexIndex < vertexCount; previousIndex = vertexIndex++)
    {
        const Vector3 current = GetPlateVertex(vertexIndex, 0.0f);
        const Vector3 previous = GetPlateVertex(previousIndex, 0.0f);
        // A polygon edge can cross the horizontal ray only when its endpoints straddle z.
        const bool crossesHeight = (current.z > z) != (previous.z > z);
        const float crossingX = (previous.x - current.x) * (z - current.z) /
                                    (previous.z - current.z) +
                                current.x;
        if (crossesHeight && x < crossingX)
        {
            isInside = !isInside;
        }
    }
    return isInside;
}

float SimulationScreen::GetPlateModeShape(float x, float z) const
{
    // Map world coordinates into the fixed cached lattice and clamp edge samples safely.
    const float radius = GetPlateRadius();
    const float normalizedX = std::clamp(
        (x + radius) / (2.0f * radius), 0.0f, 1.0f);
    const float normalizedZ = std::clamp(
        (z + radius) / (2.0f * radius), 0.0f, 1.0f);
    const float gridX = normalizedX * (kModeFieldResolution - 1);
    const float gridZ = normalizedZ * (kModeFieldResolution - 1);
    const int x0 = static_cast<int>(gridX);
    const int z0 = static_cast<int>(gridZ);
    const int x1 = std::min(x0 + 1, kModeFieldResolution - 1);
    const int z1 = std::min(z0 + 1, kModeFieldResolution - 1);
    const float blendX = gridX - x0;
    const float blendZ = gridZ - z0;
    // Convert a 2D sample coordinate into the row-major field vector index.
    const auto fieldValue = [&](int fieldX, int fieldZ)
    {
        return plateModeField_[fieldZ * kModeFieldResolution + fieldX];
    };
    // Interpolate in x on the lower and upper rows, then interpolate those results in z.
    const float lowRow = fieldValue(x0, z0) * (1.0f - blendX) +
                         fieldValue(x1, z0) * blendX;
    const float highRow = fieldValue(x0, z1) * (1.0f - blendX) +
                          fieldValue(x1, z1) * blendX;
    return lowRow * (1.0f - blendZ) + highRow * blendZ;
}

void SimulationScreen::BuildPlateModeField(int modeX, int modeZ)
{
    // The candidate list includes the nearest mode, its transpose, and nearby index pairs.
    constexpr int kCandidateModeCount = 8;
    const int candidateModes[kCandidateModeCount][2] = {
        {modeX, modeZ},
        {modeZ, modeX},
        {modeX + 1, modeZ},
        {modeX - 1, modeZ},
        {modeX, modeZ + 1},
        {modeX, modeZ - 1},
        {modeX + 1, modeZ - 1},
        {modeX - 1, modeZ + 1}};
    // Store each candidate's signed excitation/response coefficient before sampling the field.
    float candidateWeights[kCandidateModeCount]{};
    int validCandidateCount = 0;
    constexpr float kDampingRatio = 0.055f;
    const float radius = GetPlateRadius();
    const float sourceX = 0.17f * radius;
    const float sourceZ = -0.13f * radius;

    // Weight modes by their damped resonant response and by how strongly the off-center driver excites them.
    for (int candidateIndex = 0; candidateIndex < kCandidateModeCount; ++candidateIndex)
    {
        const int candidateX = candidateModes[candidateIndex][0];
        const int candidateZ = candidateModes[candidateIndex][1];
        if (candidateX < 1 || candidateX > 8 || candidateZ < 1 || candidateZ > 8)
        {
            continue;
        }

        bool isDuplicate = false;
        for (int previousIndex = 0; previousIndex < candidateIndex; ++previousIndex)
        {
            if (candidateModes[previousIndex][0] == candidateX &&
                candidateModes[previousIndex][1] == candidateZ)
            {
                isDuplicate = true;
                break;
            }
        }
        if (isDuplicate)
        {
            continue;
        }

        // Estimate this candidate's resonant frequency for the current material and dimensions.
        const float naturalFrequencyHz = GetNaturalFrequencyHz(candidateX, candidateZ);
        const float frequencyRatio = frequencyHz_ / naturalFrequencyHz;
        // Split the damped oscillator susceptibility into in-phase and quadrature components.
        const float detuning = 1.0f - frequencyRatio * frequencyRatio;
        const float quadrature = 2.0f * kDampingRatio * frequencyRatio;
        const float responseDenominator = detuning * detuning + quadrature * quadrature;
        const float inPhaseResponse = detuning / responseDenominator;
        const float quadratureResponse = quadrature / responseDenominator;
        const float excitation = EvaluatePlateBasis(sourceX, sourceZ, candidateX, candidateZ);
        candidateWeights[candidateIndex] = excitation *
                                           (0.75f * inPhaseResponse + 0.25f * quadratureResponse);
        validCandidateCount = candidateIndex + 1;
    }

    // Use total absolute modal weight to keep field amplitude in a predictable range.
    float normalization = 0.0f;
    for (int candidateIndex = 0; candidateIndex < validCandidateCount; ++candidateIndex)
    {
        normalization += std::abs(candidateWeights[candidateIndex]);
    }

    // Sample one smooth modal field so grains and the rendered mesh use identical nodal contours.
    for (int gridZ = 0; gridZ < kModeFieldResolution; ++gridZ)
    {
        const float z = -radius +
                        2.0f * radius * gridZ / (kModeFieldResolution - 1);
        for (int gridX = 0; gridX < kModeFieldResolution; ++gridX)
        {
            const float x = -radius +
                            2.0f * radius * gridX / (kModeFieldResolution - 1);
            // Sum every candidate basis function at this grid point.
            float displacement = 0.0f;
            for (int candidateIndex = 0; candidateIndex < validCandidateCount; ++candidateIndex)
            {
                displacement += candidateWeights[candidateIndex] *
                                EvaluatePlateBasis(x, z,
                                                   candidateModes[candidateIndex][0],
                                                   candidateModes[candidateIndex][1]);
            }
            plateModeField_[gridZ * kModeFieldResolution + gridX] =
                normalization > 1.0e-6f ? std::clamp(displacement / normalization, -1.0f, 1.0f) : 0.0f;
        }
    }
}

float SimulationScreen::EvaluatePlateBasis(float x, float z, int modeX, int modeZ) const
{
    // The square uses separable sine modes that vanish at simply supported edges.
    const float plateRadius = GetPlateRadius();
    if (shapeIndex_ == 0)
    {
        const float normalizedX = (x + plateRadius) / (2.0f * plateRadius);
        const float normalizedZ = (z + plateRadius) / (2.0f * plateRadius);
        return std::sin(modeX * kPi * normalizedX) *
               std::sin(modeZ * kPi * normalizedZ);
    }

    // Convert the point to polar coordinates for circular and polygonal radial/angular modes.
    const float radius = std::sqrt(x * x + z * z);
    const float angle = std::atan2(z, x);
    const int angularOrder = std::max(0, modeX - 1);
    float normalizedRadius = 0.0f;
    if (shapeIndex_ == 1)
    {
        normalizedRadius = std::min(radius / plateRadius, 1.0f);
        // Approximate a clamped radial root and subtract an edge term to reduce boundary displacement.
        const float approximateBesselRoot =
            (modeZ + 0.5f * angularOrder - 0.25f) * kPi;
        const float radialMode = std::cyl_bessel_j(
            static_cast<float>(angularOrder), approximateBesselRoot * normalizedRadius);
        const float edgeValue = std::cyl_bessel_j(
            static_cast<float>(angularOrder), approximateBesselRoot);
        const float edgeCorrection = std::pow(normalizedRadius, angularOrder + 2) * edgeValue;
        const float angularModeShape = angularOrder == 0
                                           ? 1.0f
                                           : std::cos(angularOrder * angle) +
                                                 0.35f * std::sin(angularOrder * angle);
        return (radialMode - edgeCorrection) * angularModeShape;
    }

    // For polygons, compute the radial distance to the active side in the current angular sector.
    const int sideCount = shapeIndex_ == 2 ? 5 : 6;
    const float sectorAngle = 2.0f * kPi / sideCount;
    const float edgeNormalAngle = std::remainder(angle + 0.5f * kPi - 0.5f * sectorAngle,
                                                 sectorAngle);
    const float boundaryRadius = plateRadius * std::cos(0.5f * sectorAngle) /
                                 std::cos(edgeNormalAngle);
    normalizedRadius = std::clamp(radius / boundaryRadius, 0.0f, 1.0f);
    const float radialMode = std::sin(modeZ * kPi * normalizedRadius);
    const float angularModeShape = angularOrder == 0
                                       ? 1.0f
                                       : std::cos(angularOrder * angle) +
                                             0.35f * std::sin(angularOrder * angle);
    return radialMode * angularModeShape;
}

void SimulationScreen::ConstrainParticleToPlate(Particle &particle) const
{
    if (shapeIndex_ == 1)
    {
        // Circular boundary normal points radially outward from the center.
        const float distance = std::sqrt(particle.position.x * particle.position.x +
                                         particle.position.z * particle.position.z);
        if (distance <= 1.0e-6f)
        {
            return;
        }
        const float normalX = particle.position.x / distance;
        const float normalZ = particle.position.z / distance;
        const float edgeDistance = GetPlateRadius() - kParticleRadius;
        particle.position.x = normalX * edgeDistance;
        particle.position.z = normalZ * edgeDistance;
        const float outwardSpeed = particle.velocity.x * normalX + particle.velocity.z * normalZ;
        if (outwardSpeed > 0.0f)
        {
            particle.velocity.x -= 1.2f * outwardSpeed * normalX;
            particle.velocity.z -= 1.2f * outwardSpeed * normalZ;
        }
        return;
    }

    // For square/polygon outlines, search every edge segment for the nearest point.
    float closestX = 0.0f;
    float closestZ = 0.0f;
    float closestDistanceSquared = std::numeric_limits<float>::max();
    const int vertexCount = GetPlateVertexCount();
    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
    {
        const Vector3 first = GetPlateVertex(vertexIndex, 0.0f);
        const Vector3 second = GetPlateVertex((vertexIndex + 1) % vertexCount, 0.0f);
        const float edgeX = second.x - first.x;
        const float edgeZ = second.z - first.z;
        const float edgeLengthSquared = edgeX * edgeX + edgeZ * edgeZ;
        // Project onto the finite edge segment; clamp prevents selecting beyond either vertex.
        const float projection = std::clamp(
            ((particle.position.x - first.x) * edgeX +
             (particle.position.z - first.z) * edgeZ) /
                edgeLengthSquared,
            0.0f, 1.0f);
        const float candidateX = first.x + projection * edgeX;
        const float candidateZ = first.z + projection * edgeZ;
        const float differenceX = particle.position.x - candidateX;
        const float differenceZ = particle.position.z - candidateZ;
        const float distanceSquared = differenceX * differenceX + differenceZ * differenceZ;
        if (distanceSquared < closestDistanceSquared)
        {
            closestDistanceSquared = distanceSquared;
            closestX = candidateX;
            closestZ = candidateZ;
        }
    }

    // With these centered convex outlines, the closest boundary point defines the outward normal.
    const float normalLength = std::sqrt(closestX * closestX + closestZ * closestZ);
    const float normalX = closestX / normalLength;
    const float normalZ = closestZ / normalLength;
    particle.position.x = closestX - normalX * kParticleRadius;
    particle.position.z = closestZ - normalZ * kParticleRadius;
    const float outwardSpeed = particle.velocity.x * normalX + particle.velocity.z * normalZ;
    if (outwardSpeed > 0.0f)
    {
        particle.velocity.x -= 1.2f * outwardSpeed * normalX;
        particle.velocity.z -= 1.2f * outwardSpeed * normalZ;
    }
}

void SimulationScreen::SetPlateShape(int shapeIndex)
{
    // Shape presets can only change while stopped and only when the requested index is valid.
    if (isSimulating_ || shapeIndex < 0 || shapeIndex >= kPlateShapeCount || shapeIndex == shapeIndex_)
    {
        return;
    }

    shapeIndex_ = shapeIndex;
    ResetSimulation();
}

float SimulationScreen::GetPlateSizeMeters() const
{
    // Return the metric preset selected for the current outline.
    return kPlateSizeOptionsMeters[plateSizeIndex_];
}

float SimulationScreen::GetPlateRadius() const
{
    // Convert half the selected side/diameter into Raylib world units.
    return GetPlateSizeMeters() * 0.5f * kWorldUnitsPerMeter;
}

float SimulationScreen::GetPlateThicknessMeters() const
{
    // Return thickness in meters for use by SI-based resonance calculations.
    return kPlateThicknessOptionsMeters[plateThicknessIndex_];
}

void SimulationScreen::SetPlateSize(int sizeIndex)
{
    // Preserve the current plate/pile if a size change is invalid, redundant, or requested mid-run.
    if (isSimulating_ || sizeIndex < 0 || sizeIndex >= kPlateDimensionOptionCount ||
        sizeIndex == plateSizeIndex_)
    {
        return;
    }

    // Store the new preset index before querying its converted world-space radius.
    plateSizeIndex_ = sizeIndex;
    // Pull the camera back when the new plate is larger, but never force it closer for smaller sizes.
    cameraDistance_ = std::max(cameraDistance_, GetPlateRadius() * 3.2f);
    ResetSimulation();
}

void SimulationScreen::SetPlateThickness(int thicknessIndex)
{
    // Thickness changes share the same stopped-only safety rule as size and shape changes.
    if (isSimulating_ || thicknessIndex < 0 ||
        thicknessIndex >= kPlateDimensionOptionCount || thicknessIndex == plateThicknessIndex_)
    {
        return;
    }

    // Store the new thickness preset, then clear geometry-dependent particle state.
    plateThicknessIndex_ = thicknessIndex;
    ResetSimulation();
}

void SimulationScreen::SetParticleCount(int particleCount)
{
    // Preserve all current particles if resizing is attempted during a run or to the same capacity.
    if (isSimulating_ || particleCount == particleCount_)
    {
        return;
    }

    // Resize the particle pool and its collision links together while the simulation is stopped.
    particleCount_ = particleCount;
    particles_.assign(particleCount_, Particle{});
    nextParticle_.assign(particleCount_, -1);
    emittedParticleCount_ = 0;
    emissionAccumulator_ = 0.0f;
    simulationAccumulator_ = 0.0f;
    reemissionQueue_.clear();
}

void SimulationScreen::RefeedPlateSand(float targetX, float targetZ, int percentage)
{
    // Collect only active grains resting on the plate; airborne grains remain untouched.
    std::vector<int> plateParticleIndices;
    // Reserve enough room for the maximum possible resting population to avoid repeated reallocations.
    plateParticleIndices.reserve(emittedParticleCount_);

    for (int particleIndex = 0; particleIndex < particleCount_; ++particleIndex)
    {
        const Particle &particle = particles_[particleIndex];
        if (particle.active && particle.touchingPlate)
        {
            plateParticleIndices.push_back(particleIndex);
        }
    }

    // Convert the selected percentage into the nearest whole number of surface grains.
    const int refeedCount = static_cast<int>(std::round(
        plateParticleIndices.size() * percentage / 100.0f));
    // Partial Fisher-Yates selects an unbiased subset in linear time proportional to the requested count.
    for (int selectedIndex = 0; selectedIndex < refeedCount; ++selectedIndex)
    {
        // Partial Fisher-Yates sampling chooses each requested grain without shuffling the full list.
        const int randomIndex = GetRandomValue(
            selectedIndex, static_cast<int>(plateParticleIndices.size()) - 1);
        std::swap(plateParticleIndices[selectedIndex], plateParticleIndices[randomIndex]);

        // Deactivate the chosen grain now; the emission accumulator will reactivate it above the target.
        Particle &particle = particles_[plateParticleIndices[selectedIndex]];
        particle.active = false;
        particle.touchingPlate = false;
        particle.position.x = targetX;
        particle.position.z = targetZ;
        reemissionQueue_.push_back(plateParticleIndices[selectedIndex]);
    }
}

bool SimulationScreen::GetPlateSectionAtScreenPosition(
    Vector2 screenPosition, int &sectionX, int &sectionZ) const
{
    // The sidebar overlays the full-window 3D viewport, so its clicks cannot select plate cells.
    if (screenPosition.x >= GetScreenWidth() - kSidebarWidth)
    {
        return false;
    }

    // Cast the cursor through the camera onto the plate's horizontal surface.
    // Convert the 2D cursor into a normalized world-space ray from the active camera.
    const Ray mouseRay = GetScreenToWorldRay(screenPosition, camera_);
    // A nearly horizontal ray cannot intersect the plate plane reliably.
    if (std::abs(mouseRay.direction.y) < 1.0e-6f)
    {
        return false;
    }

    // Solve rayY(t) = plateHeight for the ray parameter t.
    const float distanceToPlate = (kPlateTop - mouseRay.position.y) / mouseRay.direction.y;
    // Negative ray distance means the plane intersection lies behind the camera.
    if (distanceToPlate < 0.0f)
    {
        return false;
    }

    // Evaluate the ray at the intersection parameter to recover the world-space hit point.
    const Vector3 hitPosition{
        mouseRay.position.x + mouseRay.direction.x * distanceToPlate,
        kPlateTop,
        mouseRay.position.z + mouseRay.direction.z * distanceToPlate};
    // Reject the rectangular cursor hit if it lies outside the selected circular/polygonal outline.
    if (!IsPointInsidePlate(hitPosition.x, hitPosition.z))
    {
        return false;
    }

    const float plateRadius = GetPlateRadius();
    const float normalizedX = (hitPosition.x + plateRadius) / (2.0f * plateRadius);
    const float normalizedZ = (hitPosition.z + plateRadius) / (2.0f * plateRadius);
    // Convert normalized plate coordinates to clamped 3-by-3 section indices.
    sectionX = std::clamp(static_cast<int>(normalizedX * kPlateSections), 0, kPlateSections - 1);
    sectionZ = std::clamp(static_cast<int>(normalizedZ * kPlateSections), 0, kPlateSections - 1);
    return true;
}

void SimulationScreen::EmitParticle(Particle &particle, float targetX, float targetZ)
{
    // Scatter grains within the chosen section so the re-pour does not form one exact column.
    // Randomize a small horizontal offset and vertical start height to avoid coincident particles.
    particle.position = Vector3{
        targetX + GetRandomValue(-100, 100) / 100.0f * 0.12f,
        2.8f + GetRandomValue(-100, 100) / 100.0f * 0.025f,
        targetZ + GetRandomValue(-100, 100) / 100.0f * 0.12f};
    // Add slight lateral launch velocity; gravity supplies the dominant downward motion.
    particle.velocity = Vector3{
        GetRandomValue(-100, 100) / 100.0f * 0.04f,
        0.0f,
        GetRandomValue(-100, 100) / 100.0f * 0.04f};
    // Mark the recycled pool entry as live only after its transform and velocity are initialized.
    particle.active = true;
    particle.touchingPlate = false;
}

void SimulationScreen::DrawPlate() const
{
    // The displayed mode is derived from the current material, geometry, and frequency selection.
    int modeX = 1;
    int modeZ = 1;
    GetMode(modeX, modeZ);

    // Material colors distinguish presets while retaining a restrained metallic appearance.
    const Color plateColor = materialIndex_ == 2
                                 ? Color{139, 123, 91, 255}
                             : materialIndex_ == 1
                                 ? Color{133, 153, 160, 255}
                                 : Color{93, 111, 119, 255};
    const Color edgeColor{183, 204, 205, 255};
    const Color gridColor = materialIndex_ == 2
                                ? Color{239, 198, 119, 220}
                            : materialIndex_ == 1
                                ? Color{183, 218, 224, 220}
                                : Color{168, 199, 204, 220};
    // This mapping lifts each mesh point by the cached mode amplitude and current phase.
    const auto surfacePoint = [&](float x, float z)
    {
        const float displacement = 0.02f * plateVibrationAmplitude_ *
                                   plateVibrationPhase_ * GetPlateModeShape(x, z);
        return Vector3{x, kPlateTop + displacement, z};
    };

    // Draw sidewalls from the top boundary down by the selected physical thickness.
    const int vertexCount = GetPlateVertexCount();
    const float plateRadius = GetPlateRadius();
    const Vector3 center = surfacePoint(0.0f, 0.0f);
    const float plateBottom = kPlateTop - GetPlateThicknessMeters() * kWorldUnitsPerMeter;
    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
    {
        const Vector3 topA = GetPlateVertex(vertexIndex, 0.0f);
        const Vector3 topB = GetPlateVertex((vertexIndex + 1) % vertexCount, 0.0f);
        const Vector3 displacedTopA = surfacePoint(topA.x, topA.z);
        const Vector3 displacedTopB = surfacePoint(topB.x, topB.z);
        const Vector3 bottomA{topA.x, plateBottom, topA.z};
        const Vector3 bottomB{topB.x, plateBottom, topB.z};

        DrawTriangle3D(displacedTopA, bottomA, bottomB, plateColor);
        DrawTriangle3D(displacedTopA, bottomB, displacedTopB, plateColor);
        DrawLine3D(displacedTopA, displacedTopB, edgeColor);
        DrawLine3D(bottomA, bottomB, edgeColor);
        DrawLine3D(displacedTopA, bottomA, edgeColor);
    }

    // Subdivide each top-face sector so the simulated deflection is represented by actual triangles.
    constexpr int kSurfaceSubdivisions = 8;
    // Submit all top-surface triangles as one Raylib rlgl batch rather than separate draw calls.
    rlBegin(RL_TRIANGLES);
    // Assign one neutral metallic material color and an upward normal to every submitted vertex.
    rlColor4ub(plateColor.r, plateColor.g, plateColor.b, plateColor.a);
    rlNormal3f(0.0f, 1.0f, 0.0f);
    for (int vertexIndex = 0; vertexIndex < vertexCount; ++vertexIndex)
    {
        const Vector3 edgeA = GetPlateVertex(vertexIndex, 0.0f);
        const Vector3 edgeB = GetPlateVertex((vertexIndex + 1) % vertexCount, 0.0f);
        // Barycentric weights interpolate from the center to the two neighboring outline vertices.
        const auto sectorPoint = [&](int stepA, int stepB)
        {
            const float weightA = static_cast<float>(stepA) / kSurfaceSubdivisions;
            const float weightB = static_cast<float>(stepB) / kSurfaceSubdivisions;
            return surfacePoint(
                edgeA.x * weightA + edgeB.x * weightB,
                edgeA.z * weightA + edgeB.z * weightB);
        };
        for (int stepA = 0; stepA < kSurfaceSubdivisions; ++stepA)
        {
            for (int stepB = 0; stepB < kSurfaceSubdivisions - stepA; ++stepB)
            {
                const Vector3 point00 = sectorPoint(stepA, stepB);
                const Vector3 point10 = sectorPoint(stepA + 1, stepB);
                const Vector3 point01 = sectorPoint(stepA, stepB + 1);
                // Emit the first triangle of this barycentric grid cell in counter-clockwise order.
                rlVertex3f(point00.x, point00.y, point00.z);
                rlVertex3f(point01.x, point01.y, point01.z);
                rlVertex3f(point10.x, point10.y, point10.z);

                if (stepB < kSurfaceSubdivisions - stepA - 1)
                {
                    // The remaining half-cell is present except along the triangular sector boundary.
                    const Vector3 point11 = sectorPoint(stepA + 1, stepB + 1);
                    rlVertex3f(point10.x, point10.y, point10.z);
                    rlVertex3f(point01.x, point01.y, point01.z);
                    rlVertex3f(point11.x, point11.y, point11.z);
                }
            }
        }
    }
    rlEnd();

    // The denser wire grid makes the deformed mesh legible without changing the physical outline.
    constexpr int kGridSegments = 48;
    // Batch the dense modal wire grid as line pairs to limit driver/API overhead.
    rlBegin(RL_LINES);
    rlColor4ub(gridColor.r, gridColor.g, gridColor.b, gridColor.a);
    for (int lineIndex = 0; lineIndex <= kGridSegments; ++lineIndex)
    {
        const float fixedCoordinate = -plateRadius +
                                      2.0f * plateRadius * lineIndex / kGridSegments;
        for (int segmentIndex = 0; segmentIndex < kGridSegments; ++segmentIndex)
        {
            const float startCoordinate = -plateRadius +
                                          2.0f * plateRadius * segmentIndex / kGridSegments;
            const float endCoordinate = -plateRadius +
                                        2.0f * plateRadius * (segmentIndex + 1) / kGridSegments;
            if (IsPointInsidePlate(startCoordinate, fixedCoordinate) &&
                IsPointInsidePlate(endCoordinate, fixedCoordinate))
            {
                const Vector3 start = surfacePoint(startCoordinate, fixedCoordinate);
                const Vector3 end = surfacePoint(endCoordinate, fixedCoordinate);
                rlVertex3f(start.x, start.y, start.z);
                rlVertex3f(end.x, end.y, end.z);
            }
            if (IsPointInsidePlate(fixedCoordinate, startCoordinate) &&
                IsPointInsidePlate(fixedCoordinate, endCoordinate))
            {
                const Vector3 start = surfacePoint(fixedCoordinate, startCoordinate);
                const Vector3 end = surfacePoint(fixedCoordinate, endCoordinate);
                rlVertex3f(start.x, start.y, start.z);
                rlVertex3f(end.x, end.y, end.z);
            }
        }
    }
    rlEnd();
}

void SimulationScreen::DrawControls() const
{
    // Recompute screen-space layout from current dimensions so fullscreen and resizing stay aligned.
    const float screenWidth = static_cast<float>(GetScreenWidth());                           // Current window width in pixels.
    const float layoutScale = std::min(1.0f, static_cast<float>(GetScreenHeight()) / 720.0f); // Vertical scale for shorter windows.
    const float sidebarLeft = screenWidth - kSidebarWidth;                                    // Left edge of the fixed-width control rail.
    const float sliderLeft = sidebarLeft + kSidebarPadding;                                   // Start of the shared slider track.
    const float sliderRight = screenWidth - kSidebarPadding;                                  // End of the shared slider track.
    const float sliderY = 154.0f * layoutScale;                                               // Frequency slider centerline.
    const float redistributeSliderY = 525.0f * layoutScale;                                   // Redistribution-percentage slider centerline.
    // Button rectangles are shared conceptually with Update's hit-testing coordinates.
    const Rectangle refillButtonBounds{
        sidebarLeft + kSidebarPadding, 555.0f * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, 40.0f * layoutScale};
    const Rectangle simulationButtonBounds{
        sidebarLeft + kSidebarPadding, 610.0f * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, 48.0f * layoutScale};
    // Dropdown rectangles match the selector hitboxes built by Update.
    const Rectangle particleSelectorBounds{
        sidebarLeft + kSidebarPadding, kParticleSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kParticleSelectorHeight * layoutScale};
    const Rectangle materialSelectorBounds{
        sidebarLeft + kSidebarPadding, kMaterialSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kMaterialSelectorHeight * layoutScale};
    const Rectangle shapeSelectorBounds{
        sidebarLeft + kSidebarPadding, kShapeSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kShapeSelectorHeight * layoutScale};
    const Rectangle sizeSelectorBounds{
        sidebarLeft + kSidebarPadding, kSizeSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kShapeSelectorHeight * layoutScale};
    const Rectangle thicknessSelectorBounds{
        sidebarLeft + kSidebarPadding, kThicknessSelectorY * layoutScale,
        kSidebarWidth - 2.0f * kSidebarPadding, kShapeSelectorHeight * layoutScale};
    const Vector2 mousePosition = GetMousePosition(); // Used only to draw hover feedback.

    // Draw a full-height control rail, separated from the 3D workspace.
    DrawRectangle(static_cast<int>(sidebarLeft), 0, static_cast<int>(kSidebarWidth),
                  GetScreenHeight(), kUiPanel);
    DrawLine(static_cast<int>(sidebarLeft), 0, static_cast<int>(sidebarLeft), GetScreenHeight(),
             kUiPanelEdge);
    // Establish the panel title and its smaller section heading.
    DrawText("SIMULATION", static_cast<int>(sidebarLeft + kSidebarPadding), static_cast<int>(30.0f * layoutScale), 21,
             kUiText);
    DrawText("PLATE CONTROLS", static_cast<int>(sidebarLeft + kSidebarPadding), static_cast<int>(59.0f * layoutScale), 12,
             kUiMuted);
    // Keep the currently estimated mode visible even before the simulation starts.
    int activeModeX = 1;
    int activeModeZ = 1;
    GetMode(activeModeX, activeModeZ);
    // Display the closest estimated natural mode and its frequency near the panel heading.
    DrawText(TextFormat("MODE %d x %d  |  %.0f Hz", activeModeX, activeModeZ,
                        GetNaturalFrequencyHz(activeModeX, activeModeZ)),
             static_cast<int>(sidebarLeft + kSidebarPadding), static_cast<int>(82.0f * layoutScale),
             12, kUiMuted);

    // Show the selected drive frequency as a precise numeric readout.
    DrawText(TextFormat("FREQUENCY  %.0f Hz", frequencyHz_),
             static_cast<int>(sidebarLeft + kSidebarPadding), static_cast<int>(112.0f * layoutScale), 16,
             kUiText);

    // Use a dimmed selector fill during simulation because preset changes are disabled while running.
    const Color selectorColor = isSimulating_ ? Color{29, 32, 35, 255} : kUiControl;
    // Particle-count selector: its value changes pool capacity while stopped.
    DrawText("PARTICLE COUNT", static_cast<int>(sidebarLeft + kSidebarPadding), static_cast<int>(180.0f * layoutScale), 13,
             kUiMuted);
    DrawRectangleRec(particleSelectorBounds, selectorColor);
    DrawRectangleLinesEx(particleSelectorBounds, 1.0f, kUiControlEdge);
    DrawText(TextFormat("%d", particleCount_), static_cast<int>(particleSelectorBounds.x) + 12,
             static_cast<int>(particleSelectorBounds.y + 6.0f * layoutScale), 14,
             isSimulating_ ? kUiMuted : kUiText);
    DrawTriangle(
        Vector2{particleSelectorBounds.x + particleSelectorBounds.width - 21.0f,
                particleSelectorBounds.y + 11.0f * layoutScale},
        Vector2{particleSelectorBounds.x + particleSelectorBounds.width - 9.0f,
                particleSelectorBounds.y + 11.0f * layoutScale},
        Vector2{particleSelectorBounds.x + particleSelectorBounds.width - 15.0f,
                particleSelectorBounds.y + 18.0f * layoutScale},
        kUiMuted);

    // Material selector: labels index the density, modulus, and Poisson-ratio table.
    DrawText("PLATE MATERIAL", static_cast<int>(sidebarLeft + kSidebarPadding), static_cast<int>(245.0f * layoutScale), 13,
             kUiMuted);
    DrawRectangleRec(materialSelectorBounds, selectorColor);
    DrawRectangleLinesEx(materialSelectorBounds, 1.0f, kUiControlEdge);
    DrawText(kPlateMaterials[materialIndex_].name,
             static_cast<int>(materialSelectorBounds.x) + 12,
             static_cast<int>(materialSelectorBounds.y + 10.0f * layoutScale), 14,
             isSimulating_ ? kUiMuted : kUiText);
    DrawTriangle(
        Vector2{materialSelectorBounds.x + materialSelectorBounds.width - 21.0f,
                materialSelectorBounds.y + 11.0f * layoutScale},
        Vector2{materialSelectorBounds.x + materialSelectorBounds.width - 9.0f,
                materialSelectorBounds.y + 11.0f * layoutScale},
        Vector2{materialSelectorBounds.x + materialSelectorBounds.width - 15.0f,
                materialSelectorBounds.y + 18.0f * layoutScale},
        kUiMuted);

    // Shape selector: changing this also changes containment and modal basis evaluation.
    DrawText("PLATE SHAPE", static_cast<int>(sidebarLeft + kSidebarPadding),
             static_cast<int>(310.0f * layoutScale), 13, kUiMuted);
    DrawRectangleRec(shapeSelectorBounds, selectorColor);
    DrawRectangleLinesEx(shapeSelectorBounds, 1.0f, kUiControlEdge);
    DrawText(kPlateShapeNames[shapeIndex_], static_cast<int>(shapeSelectorBounds.x) + 12,
             static_cast<int>(shapeSelectorBounds.y + 10.0f * layoutScale), 14,
             isSimulating_ ? kUiMuted : kUiText);
    DrawTriangle(
        Vector2{shapeSelectorBounds.x + shapeSelectorBounds.width - 21.0f,
                shapeSelectorBounds.y + 11.0f * layoutScale},
        Vector2{shapeSelectorBounds.x + shapeSelectorBounds.width - 9.0f,
                shapeSelectorBounds.y + 11.0f * layoutScale},
        Vector2{shapeSelectorBounds.x + shapeSelectorBounds.width - 15.0f,
                shapeSelectorBounds.y + 18.0f * layoutScale},
        kUiMuted);

    // Size selector labels side length for squares and diameter for all other outlines.
    DrawText("PLATE SIZE", static_cast<int>(sidebarLeft + kSidebarPadding),
             static_cast<int>(375.0f * layoutScale), 13, kUiMuted);
    DrawRectangleRec(sizeSelectorBounds, selectorColor);
    DrawRectangleLinesEx(sizeSelectorBounds, 1.0f, kUiControlEdge);
    DrawText(kPlateSizeOptionLabels[plateSizeIndex_],
             static_cast<int>(sizeSelectorBounds.x) + 12,
             static_cast<int>(sizeSelectorBounds.y + 10.0f * layoutScale), 14,
             isSimulating_ ? kUiMuted : kUiText);
    DrawTriangle(
        Vector2{sizeSelectorBounds.x + sizeSelectorBounds.width - 21.0f,
                sizeSelectorBounds.y + 11.0f * layoutScale},
        Vector2{sizeSelectorBounds.x + sizeSelectorBounds.width - 9.0f,
                sizeSelectorBounds.y + 11.0f * layoutScale},
        Vector2{sizeSelectorBounds.x + sizeSelectorBounds.width - 15.0f,
                sizeSelectorBounds.y + 18.0f * layoutScale},
        kUiMuted);

    // Thickness selector updates physical stiffness/mass and the drawn sidewall depth.
    DrawText("PLATE THICKNESS", static_cast<int>(sidebarLeft + kSidebarPadding),
             static_cast<int>(440.0f * layoutScale), 13, kUiMuted);
    DrawRectangleRec(thicknessSelectorBounds, selectorColor);
    DrawRectangleLinesEx(thicknessSelectorBounds, 1.0f, kUiControlEdge);
    DrawText(kPlateThicknessOptionLabels[plateThicknessIndex_],
             static_cast<int>(thicknessSelectorBounds.x) + 12,
             static_cast<int>(thicknessSelectorBounds.y + 10.0f * layoutScale), 14,
             isSimulating_ ? kUiMuted : kUiText);
    DrawTriangle(
        Vector2{thicknessSelectorBounds.x + thicknessSelectorBounds.width - 21.0f,
                thicknessSelectorBounds.y + 11.0f * layoutScale},
        Vector2{thicknessSelectorBounds.x + thicknessSelectorBounds.width - 9.0f,
                thicknessSelectorBounds.y + 11.0f * layoutScale},
        Vector2{thicknessSelectorBounds.x + thicknessSelectorBounds.width - 15.0f,
                thicknessSelectorBounds.y + 18.0f * layoutScale},
        kUiMuted);

    // Draw the frequency track, its active segment, and the current-value handle.
    DrawLineEx(Vector2{sliderLeft, sliderY}, Vector2{sliderRight, sliderY}, 4.0f,
               Color{75, 92, 96, 255});
    // Convert frequency into a normalized thumb position between the configured endpoints.
    const float sliderRatio = (frequencyHz_ - kMinFrequencyHz) / (kMaxFrequencyHz - kMinFrequencyHz);
    const float sliderX = sliderLeft + sliderRatio * (sliderRight - sliderLeft);
    DrawLineEx(Vector2{sliderLeft, sliderY}, Vector2{sliderX, sliderY}, 4.0f,
               kUiAccent);
    DrawCircleV(Vector2{sliderX, sliderY}, 7.0f, kUiAccentHover);

    // Report the selected discrete percentage above the redistribution slider.
    DrawText(TextFormat("REDISTRIBUTE AMOUNT  %d%%", redistributePercent_),
             static_cast<int>(sidebarLeft + kSidebarPadding),
             static_cast<int>(493.0f * layoutScale), 13, kUiText);
    DrawLineEx(Vector2{sliderLeft, redistributeSliderY},
               Vector2{sliderRight, redistributeSliderY}, 3.0f, kUiControlEdge);
    // Map 10–100 percent onto the same normalized track used by the pointer handler.
    const float redistributeRatio = (redistributePercent_ - 10) / 90.0f;
    const float redistributeX = sliderLeft + redistributeRatio * (sliderRight - sliderLeft);
    DrawLineEx(Vector2{sliderLeft, redistributeSliderY},
               Vector2{redistributeX, redistributeSliderY}, 3.0f, kUiAccent);
    // Add one tick for each legal redistribution value from 10% through 100%.
    for (int percent = 10; percent <= 100; percent += 10)
    {
        const float tickRatio = (percent - 10) / 90.0f;
        const float tickX = sliderLeft + tickRatio * (sliderRight - sliderLeft);
        DrawLineEx(Vector2{tickX, redistributeSliderY - 5.0f * layoutScale},
                   Vector2{tickX, redistributeSliderY + 5.0f * layoutScale}, 1.0f,
                   kUiMuted);
    }
    DrawCircleV(Vector2{redistributeX, redistributeSliderY}, 7.0f, kUiAccentHover);
    DrawText("10%", static_cast<int>(sliderLeft),
             static_cast<int>(redistributeSliderY + 9.0f * layoutScale), 10,
             kUiMuted);
    DrawText("100%", static_cast<int>(sliderRight - MeasureText("100%", 10)),
             static_cast<int>(redistributeSliderY + 9.0f * layoutScale), 10,
             kUiMuted);

    // Disable redistribution until at least one active grain has contacted the plate.
    const bool hasPlateSand = std::any_of(particles_.begin(), particles_.end(),
                                          [](const Particle &particle)
                                          { return particle.active && particle.touchingPlate; });
    // The command requires a running simulation and at least one grain resting on the plate.
    const bool refillEnabled = isSimulating_ && hasPlateSand;
    const bool refillHovered = refillEnabled && CheckCollisionPointRec(mousePosition, refillButtonBounds);
    const Color refillColor = !refillEnabled  ? Color{38, 41, 45, 255}
                              : refillHovered ? Color{63, 69, 75, 255}
                                              : Color{49, 54, 60, 255};
    DrawRectangleRec(refillButtonBounds, refillColor);
    const char *refillText = "Redistribute";
    const int refillTextWidth = MeasureText(refillText, 13);
    DrawText(refillText,
             static_cast<int>(refillButtonBounds.x + (refillButtonBounds.width - refillTextWidth) * 0.5f),
             static_cast<int>(refillButtonBounds.y + 15.0f), 13,
             refillEnabled ? kUiText : kUiMuted);

    // Start/stop is always available and uses the steel accent rather than the old saturated orange.
    const bool buttonHovered = CheckCollisionPointRec(mousePosition, simulationButtonBounds);
    const Color buttonColor = buttonHovered ? kUiAccentHover : kUiAccent;
    DrawRectangleRec(simulationButtonBounds, buttonColor);
    const char *buttonText = isSimulating_ ? "Stop" : "Start";
    const int buttonTextWidth = MeasureText(buttonText, 14);
    DrawText(buttonText,
             static_cast<int>(simulationButtonBounds.x + (simulationButtonBounds.width - buttonTextWidth) * 0.5f),
             static_cast<int>(simulationButtonBounds.y + 19.0f * layoutScale), 14, Color{20, 23, 26, 255});

    DrawText("F11  FULL SCREEN", static_cast<int>(sidebarLeft + kSidebarPadding),
             static_cast<int>(690.0f * layoutScale), 10, kUiMuted);

    // Render the one active dropdown below its selector and highlight its selected option.
    if ((isParticleMenuOpen_ || isMaterialMenuOpen_ || isShapeMenuOpen_ ||
         isSizeMenuOpen_ || isThicknessMenuOpen_) &&
        !isSimulating_)
    {
        // Resolve the selector state once so all dropdown options share one render loop.
        const bool showingParticleOptions = isParticleMenuOpen_;
        const bool showingMaterialOptions = isMaterialMenuOpen_;
        const bool showingShapeOptions = isShapeMenuOpen_;
        const Rectangle activeSelector = showingParticleOptions
                                             ? particleSelectorBounds
                                         : showingMaterialOptions ? materialSelectorBounds
                                         : showingShapeOptions    ? shapeSelectorBounds
                                         : isSizeMenuOpen_        ? sizeSelectorBounds
                                                                  : thicknessSelectorBounds;
        const int optionCount = showingParticleOptions
                                    ? kParticleCountOptionTotal
                                : showingMaterialOptions ? kPlateMaterialCount
                                : showingShapeOptions    ? kPlateShapeCount
                                                         : kPlateDimensionOptionCount;
        const float optionRowHeight = kDimensionOptionRowHeight * layoutScale;
        // Expand the option list directly below the selector that opened it.
        const float optionsTop = activeSelector.y + activeSelector.height;
        DrawRectangle(static_cast<int>(activeSelector.x), static_cast<int>(optionsTop),
                      static_cast<int>(activeSelector.width),
                      static_cast<int>(optionCount * optionRowHeight),
                      kUiPanel);
        for (int optionIndex = 0; optionIndex < optionCount; ++optionIndex)
        {
            const Rectangle optionBounds{
                activeSelector.x,
                optionsTop + optionIndex * optionRowHeight,
                activeSelector.width,
                optionRowHeight};
            // Apply hover and selected styling without changing application state during drawing.
            const bool isHovered = CheckCollisionPointRec(mousePosition, optionBounds);
            const bool isSelected = showingParticleOptions
                                        ? kParticleCountOptions[optionIndex] == particleCount_
                                    : showingMaterialOptions ? optionIndex == materialIndex_
                                    : showingShapeOptions    ? optionIndex == shapeIndex_
                                    : isSizeMenuOpen_        ? optionIndex == plateSizeIndex_
                                                             : optionIndex == plateThicknessIndex_;
            if (isHovered || isSelected)
            {
                DrawRectangleRec(optionBounds, isHovered ? Color{53, 59, 65, 255}
                                                         : Color{38, 43, 48, 255});
            }
            const char *optionLabel = showingParticleOptions
                                          ? TextFormat("%d granos", kParticleCountOptions[optionIndex])
                                      : showingMaterialOptions ? kPlateMaterials[optionIndex].name
                                      : showingShapeOptions    ? kPlateShapeNames[optionIndex]
                                      : isSizeMenuOpen_        ? kPlateSizeOptionLabels[optionIndex]
                                                               : kPlateThicknessOptionLabels[optionIndex];
            DrawText(optionLabel,
                     static_cast<int>(optionBounds.x) + 10,
                     static_cast<int>(optionBounds.y + 5.0f * layoutScale), 14,
                     isSelected ? kUiAccentHover : kUiText);
        }
        DrawRectangleLinesEx(
            Rectangle{activeSelector.x, optionsTop, activeSelector.width,
                      optionCount * optionRowHeight},
            1.0f, kUiControlEdge);
    }
}

void SimulationScreen::GetMode(int &modeX, int &modeZ) const
{
    float smallestFrequencyDifference = std::numeric_limits<float>::max();
    for (int candidateX = 1; candidateX <= 8; ++candidateX)
    {
        for (int candidateZ = 1; candidateZ <= 8; ++candidateZ)
        {
            const float candidateFrequency = GetNaturalFrequencyHz(candidateX, candidateZ);
            const float frequencyDifference = std::abs(candidateFrequency - frequencyHz_);
            if (frequencyDifference < smallestFrequencyDifference)
            {
                smallestFrequencyDifference = frequencyDifference;
                modeX = candidateX;
                modeZ = candidateZ;
            }
        }
    }
}

float SimulationScreen::GetNaturalFrequencyHz(int modeX, int modeZ) const
{
    // Estimate a simply supported thin plate's natural frequency from its material constants.
    const PlateMaterial &material = kPlateMaterials[materialIndex_];
    const float thicknessMeters = GetPlateThicknessMeters();
    const float thicknessCubed = thicknessMeters * thicknessMeters * thicknessMeters;
    const float flexuralRigidity = material.youngsModulusPa * thicknessCubed /
                                   (12.0f * (1.0f - material.poissonsRatio * material.poissonsRatio));
    const float massPerArea = material.densityKgPerM3 * thicknessMeters;
    const float waveSpeed = std::sqrt(flexuralRigidity / massPerArea);
    if (shapeIndex_ == 0)
    {
        // The square uses the simply supported rectangular-plate approximation.
        const float squareLengthMeters = GetPlateSizeMeters();
        const float modeWavenumberSquared =
            std::pow(modeX / squareLengthMeters, 2.0f) +
            std::pow(modeZ / squareLengthMeters, 2.0f);
        return 0.5f * kPi * waveSpeed * modeWavenumberSquared;
    }

    // Circular and polygonal plates use a radial/angular estimate rather than rectangular modes.
    const float radiusMeters = GetPlateSizeMeters() * 0.5f;
    const float angularOrder = static_cast<float>(modeX - 1);
    const float radialWavenumber = (modeZ + 0.5f * angularOrder) * kPi / radiusMeters;
    const float angularWavenumber = angularOrder / radiusMeters;
    const float modeWavenumberSquared = radialWavenumber * radialWavenumber +
                                        angularWavenumber * angularWavenumber;
    const float polygonCorrection = shapeIndex_ == 2   ? 1.06f
                                    : shapeIndex_ == 3 ? 1.03f
                                                       : 1.0f;
    return waveSpeed * modeWavenumberSquared * polygonCorrection / (2.0f * kPi);
}