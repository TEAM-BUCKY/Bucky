cmake_minimum_required(VERSION 3.28)
project(bucky_simulation)

set(CMAKE_CXX_STANDARD 17)

# Robot source root — parent directory (Bucky/)
set(BUCKY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

add_subdirectory(libs/raylib)

# --- Portable robot algorithm sources (compiled on desktop) ---
set(ROBOT_SOURCES
    ${BUCKY_ROOT}/src/control/ekf/EKFCore.cpp
    ${BUCKY_ROOT}/src/control/pos/SelfLocalizationEKF.cpp
    ${BUCKY_ROOT}/src/control/pos/EnemyTracker.cpp
    ${BUCKY_ROOT}/src/control/ball/IMMBallTracker.cpp
    ${BUCKY_ROOT}/src/control/ball/IRBallProcessor.cpp
    ${BUCKY_ROOT}/src/strategy/StrategyFSM.cpp
)

# --- Simulation sources ---
set(SIM_SOURCES
    src/simulation/SensorSim.cpp
    src/simulation/SimLoop.cpp
)

# --- Rendering sources ---
set(RENDER_SOURCES
    src/rendering/FieldRenderer.cpp
    src/rendering/EntityRenderer.cpp
    src/rendering/SplitScreen.cpp
    src/rendering/OverlayRenderer.cpp
)

# --- UI sources ---
set(UI_SOURCES
    src/ui/Controls.cpp
    src/ui/HUD.cpp
)

add_executable(bucky_simulation
    src/main.cpp
    ${ROBOT_SOURCES}
    ${SIM_SOURCES}
    ${RENDER_SOURCES}
    ${UI_SOURCES}
)

target_include_directories(bucky_simulation PRIVATE
    ${BUCKY_ROOT}/src                       # Robot headers (DigitalField.h, Math.h, etc.)
    ${CMAKE_CURRENT_SOURCE_DIR}/src         # Visualization headers
    libs/raylib/src                         # Raylib
)

target_link_libraries(bucky_simulation raylib)

# Force-include desktop overrides BEFORE any robot header is processed.
# This blocks hardware-specific headers via their include guards and
# provides desktop-compatible replacements.
target_compile_options(bucky_simulation PRIVATE
    "-include" "${CMAKE_CURRENT_SOURCE_DIR}/platform/desktop_overrides.h"
)
