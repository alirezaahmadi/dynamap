
# add_executable(testRaytracer      testRaytracer.cpp)
# target_link_libraries(testRaytracer
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )

# add_executable(testProjection testProjection.cpp)
# target_link_libraries(testProjection
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )

# add_executable(testMeshBlender    testMeshBlender.cpp)
# target_link_libraries(testMeshBlender
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )

# add_executable(testDualQuaternion testDualQuaternion.cpp)
# target_link_libraries(testDualQuaternion
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )

# add_executable(testKDTree testKDTree.cpp)
# target_link_libraries(testKDTree
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )

# add_executable(testDefGraph testDefGraph.cpp)
# target_link_libraries(testDefGraph
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )

add_executable(testDynaMap testDynaMap.cpp)
target_link_libraries(testDynaMap
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_cuda
    ${PROJECT_NAME}_gui
)

add_executable(testDynaMapGUI testDynaMapGUI.cpp)
target_link_libraries(testDynaMapGUI
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_cuda
    ${PROJECT_NAME}_gui
)

# add_executable(glfwWindow glfwWindow.cpp)
# target_link_libraries(glfwWindow
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )

