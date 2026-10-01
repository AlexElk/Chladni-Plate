# Require a CMake version that supports the target and preset features used by this project.
cmake_minimum_required(VERSION 3.20)

# Give the generated executable and IDE project a stable name and version.
project(HelloRaylib
    # Record the current prototype version in CMake metadata.
    VERSION 1.0
    # Build this project as a C++ application.
    LANGUAGES CXX
)

# Compile all project source files with the C++17 language standard.
set(CMAKE_CXX_STANDARD 17)
# Fail configuration if the selected compiler cannot provide C++17.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# Disable compiler-specific language extensions for more portable source code.
set(CMAKE_CXX_EXTENSIONS OFF)

# Locate Raylib's installed CMake package and imported library target.
find_package(raylib REQUIRED)

# Compile the application entry point and each screen-system implementation into one executable.
add_executable(HelloRaylib
    # Initialize the window and run the application loop.
    src/main.cpp
    # Own and dispatch calls to the currently active screen.
    src/core/ScreenManager.cpp
    # Implement the Chladni plate view and simulation.
    src/screens/SimulationScreen.cpp
)

# Link Raylib privately because it is an implementation dependency of this executable.
target_link_libraries(HelloRaylib
    PRIVATE
    # Provides windowing, input, 3D drawing, and rlgl functionality.
    raylib
)