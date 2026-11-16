cmake_minimum_required(VERSION 3.20)
project(HD VERSION 0.7.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(HD_BUILD_APP   "Build the HD desktop application (needs GLFW + Dear ImGui, fetched automatically)" ON)
option(HD_BUILD_TESTS "Build the headless core tests" ON)
option(HD_BUILD_OFFSCREEN_RENDER_TEST "Linux only: render the viewport offscreen via EGL (developer check)" OFF)
option(HD_BUILD_UI_SCREENSHOT_TEST "Linux only: run the UI headlessly via EGL and save screenshots (developer check)" OFF)

# ------------------------------------------------------------------ core engine
add_library(hd_core STATIC
  src/core/Compositing.cpp
  src/core/Demo.cpp
  src/core/Denoise.cpp
  src/core/Destruction.cpp
  src/core/Expr.cpp
  src/core/Fluids.cpp
  src/core/Fracture.cpp
  src/core/GeoIO.cpp
  src/core/Geometry.cpp
  src/core/Image.cpp
  src/core/Json.cpp
  src/core/Lights.cpp
  src/core/Node.cpp
  src/core/Nodes.cpp
  src/core/Noise.cpp
  src/core/Ops.cpp
  src/core/PathTracer.cpp
  src/core/Pyro.cpp
  src/core/Rigging.cpp
  src/core/Shading.cpp
  src/core/Shader.cpp
  src/core/Boolean.cpp
  src/core/ImageSequence.cpp
  src/core/Video.cpp
  src/core/Usd.cpp
  src/core/Python.cpp
  src/core/Version.cpp
  src/core/Sim.cpp
  src/core/Vellum.cpp
  src/core/Volume.cpp
  src/core/Whitewater.cpp
)
target_include_directories(hd_core PUBLIC src)

# Version identification, compiled into Version.cpp only so a new commit doesn't rebuild everything.
set(HD_GIT_REVISION "unknown")
find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
  execute_process(COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} OUTPUT_VARIABLE HD_GIT_REVISION_OUT
    OUTPUT_STRIP_TRAILING_WHITESPACE ERROR_QUIET RESULT_VARIABLE HD_GIT_RESULT)
  if(HD_GIT_RESULT EQUAL 0 AND HD_GIT_REVISION_OUT)
    set(HD_GIT_REVISION "${HD_GIT_REVISION_OUT}")
  endif()
endif()
set_source_files_properties(src/core/Version.cpp PROPERTIES COMPILE_DEFINITIONS
  "HD_VERSION_STRING=\"${PROJECT_VERSION}\";HD_GIT_REVISION=\"${HD_GIT_REVISION}\"")
find_package(Threads REQUIRED)
target_link_libraries(hd_core PUBLIC Threads::Threads ${CMAKE_DL_LIBS}) # dl: Python is loaded at runtime
if(MSVC)
  target_compile_options(hd_core PRIVATE /W3 /utf-8)
  target_compile_definitions(hd_core PUBLIC _USE_MATH_DEFINES NOMINMAX _CRT_SECURE_NO_WARNINGS)
else()
  target_compile_options(hd_core PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()

# headless command-line tool (cook / export / demos)
# Windows: icon + version resource for both executables
if(WIN32)
  set(HD_ICON_PATH "${CMAKE_SOURCE_DIR}/packaging/icons/hd.ico")
  file(TO_CMAKE_PATH "${HD_ICON_PATH}" HD_ICON_PATH)
  configure_file(packaging/windows/hd.rc.in ${CMAKE_BINARY_DIR}/hd.rc @ONLY)
  set(HD_WIN_RESOURCES ${CMAKE_BINARY_DIR}/hd.rc)
endif()

add_executable(hdc src/tools/hdc.cpp ${HD_WIN_RESOURCES})
target_link_libraries(hdc PRIVATE hd_core)

if(HD_BUILD_TESTS)
  enable_testing()
  add_executable(hd_tests tests/test_core.cpp)
  target_link_libraries(hd_tests PRIVATE hd_core)
  target_compile_definitions(hd_tests PRIVATE HD_EXAMPLES_DIR="${CMAKE_SOURCE_DIR}/examples")
  add_test(NAME hd_tests COMMAND hd_tests)
  # mutation fuzzing of every parser (more iterations in the sanitizer CI job)
  add_executable(hd_fuzz tests/fuzz.cpp)
  target_link_libraries(hd_fuzz PRIVATE hd_core)
  if(UNIX AND NOT APPLE)
    target_link_options(hd_fuzz PRIVATE -rdynamic) # symbol names in the fuzzer's crash stack trace
  endif()
  add_test(NAME hd_fuzz COMMAND hd_fuzz 300)
endif()

# ------------------------------------------------------------------ renderer (OpenGL 3.3 core)
add_library(hd_render STATIC
  src/render/GL.cpp
  src/render/Renderer.cpp
)
target_include_directories(hd_render PUBLIC src)
target_link_libraries(hd_render PUBLIC hd_core)
if(APPLE)
  target_compile_definitions(hd_render PUBLIC GL_SILENCE_DEPRECATION)
endif()
if(NOT WIN32)
  target_link_libraries(hd_render PUBLIC ${CMAKE_DL_LIBS})
endif()

if(HD_BUILD_OFFSCREEN_RENDER_TEST AND UNIX AND NOT APPLE)
  add_executable(hd_render_test tests/render_offscreen.cpp)
  target_link_libraries(hd_render_test PRIVATE hd_render)
endif()

# ------------------------------------------------------------------ application
if(HD_BUILD_APP)
  include(FetchContent)
  set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
    GIT_SHALLOW TRUE)
  FetchContent_Declare(imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9b-docking
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(glfw imgui)

  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${imgui_SOURCE_DIR}/misc/cpp/imgui_stdlib.cpp
  )
  target_include_directories(imgui PUBLIC ${imgui_SOURCE_DIR} ${imgui_SOURCE_DIR}/backends)
  target_link_libraries(imgui PUBLIC glfw)
  target_compile_definitions(imgui PUBLIC GLFW_INCLUDE_NONE)

  add_library(hd_app STATIC
    src/app/Animation.cpp
    src/app/App.cpp
    src/app/Assets.cpp
    src/app/NetworkEditor.cpp
    src/app/ParmPane.cpp
    src/app/Menus.cpp
    src/app/Panels.cpp
    src/app/RenderView.cpp
    src/app/SequencePlayer.cpp
    src/app/PythonShell.cpp
    src/app/Shelf.cpp
    src/app/Theme.cpp
  )
  if(APPLE)
    # Compiled as C++ with -x objective-c++ so enabling OBJCXX doesn't change how GLFW's .m files build.
    set_source_files_properties(src/app/MacOpenFiles.mm PROPERTIES LANGUAGE CXX COMPILE_OPTIONS "-x;objective-c++")
    target_sources(hd_app PRIVATE src/app/MacOpenFiles.mm)
  endif()
  target_link_libraries(hd_app PUBLIC hd_render imgui glfw)

  set(HD_APP_SOURCES src/app/main.cpp)
  if(APPLE)
    set(HD_ICNS ${CMAKE_SOURCE_DIR}/packaging/icons/hd.icns)
    set_source_files_properties(${HD_ICNS} PROPERTIES MACOSX_PACKAGE_LOCATION Resources)
    add_executable(HD MACOSX_BUNDLE ${HD_APP_SOURCES} ${HD_ICNS})
    set_target_properties(HD PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST ${CMAKE_SOURCE_DIR}/packaging/macos/Info.plist.in
      MACOSX_BUNDLE_BUNDLE_NAME "HD"
      MACOSX_BUNDLE_EXECUTABLE_NAME "HD"
      MACOSX_BUNDLE_ICON_FILE "hd.icns"
      MACOSX_BUNDLE_GUI_IDENTIFIER "com.vfxchris581.hd"
      MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION}
      MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
      MACOSX_BUNDLE_COPYRIGHT "vfxchris581")
  elseif(WIN32)
    add_executable(HD WIN32 ${HD_APP_SOURCES} ${HD_WIN_RESOURCES})
  else()
    add_executable(HD ${HD_APP_SOURCES})
  endif()
  target_link_libraries(HD PRIVATE hd_app)
  if(WIN32)
    target_link_libraries(HD PRIVATE opengl32)
    if(MSVC)
      target_link_options(HD PRIVATE /ENTRY:mainCRTStartup)
    endif()
  elseif(APPLE)
    target_link_libraries(HD PRIVATE "-framework OpenGL" "-framework Cocoa" "-framework IOKit")
  endif()
  # Linux developer check: run the whole UI headlessly and write screenshots
  if(HD_BUILD_UI_SCREENSHOT_TEST AND UNIX AND NOT APPLE)
    add_executable(hd_ui_screenshot tests/ui_screenshot.cpp)
    target_link_libraries(hd_ui_screenshot PRIVATE hd_app ${CMAKE_DL_LIBS})
    add_executable(hd_ui_smoke tests/ui_smoke.cpp)
    target_link_libraries(hd_ui_smoke PRIVATE hd_app ${CMAKE_DL_LIBS})
  endif()

  # Ship example scenes next to the executable
  add_custom_command(TARGET HD POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_SOURCE_DIR}/examples $<TARGET_FILE_DIR:HD>/examples)
endif()
