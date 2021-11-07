
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

include_directories(
    ${OPENGL_INCLUDE_DIR}
    ${GLEW_INCLUDE_DIRS}

    include/gui/model
    include/gui/axis
    include/gui/camera
    include/gui/renderee
    include/gui/openGL

    include/gui/imgui

    include/gui/glad
    include/gui/KHR
)

add_library(${PROJECT_NAME}_gui
    model/model.cpp
    axis/axis.cpp
    camera/camera.cpp
    renderer/renderer.cpp
    glad/glad.c
)

target_link_libraries(${PROJECT_NAME}_gui
    ${PROJECT_NAME}_core

    ${GLFW_LIBRARIES}
    ${GLEW_LIBRARIES}
    ${OPENGL_LIBRARIES}

    -I/usr/include 
    -L/usr/lib 
    -lglut 
    -lGLU 
    -lGL 
    -lX11 
    -lcudart
)
