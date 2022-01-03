add_executable(dynaMap dynaMap.cpp)
target_link_libraries(dynaMap
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_cuda
    ${PROJECT_NAME}_gui
)

add_executable(dynaMap_node dynaMap_node.cpp 
    ../include/nodeHandler/nodeHandler.cpp)
target_link_libraries(dynaMap_node
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_cuda
    ${PROJECT_NAME}_gui
)


