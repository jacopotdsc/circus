set(CMAKE_AUTOMOC ON)

# Including headers globally here is a workaround.
# Ideally, headers should be added per-target via include_directories(). But it's not working for some reason...
file(GLOB_RECURSE SRC_FILES
    ${CMAKE_SOURCE_DIR}/Src/*.cpp
    ${CMAKE_SOURCE_DIR}/Include/*.h
)

add_executable(main ${SRC_FILES})

# Link libraries
target_link_libraries(main
    mujoco
    pugixml
    yaml-cpp
    Qt6::Widgets
    Qt6::OpenGLWidgets
    Eigen3::Eigen
    GLU
)
