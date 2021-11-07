add_executable(DynaMap DynaMap.cpp)

target_link_libraries(DynaMap
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_cuda
    ${PROJECT_NAME}_gui
)

## Declare cpp executables
# add_executable(dynaMap_node
#   src/dynaMap_node.cpp
#   include/nodehandler/dynaMap_nodehandler.cpp
# )

# target_link_libraries(dynaMap_node
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )


# add_executable(testgui testgui.cpp)
# target_link_libraries(testgui
#     ${PROJECT_NAME}_core
#     ${PROJECT_NAME}_cuda
#     ${PROJECT_NAME}_gui
# )