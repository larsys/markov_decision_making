cmake_minimum_required(VERSION 2.8.3)
project(mdm_test)

find_package(catkin REQUIRED COMPONENTS mdm_library
                                        predicate_manager
                                        topological_tools)

find_package( SDL )
find_package( SDL_image )

if ( NOT_SDLIMAGE_FOUND )
  message(FATAL_ERROR, "SDL_image must be installed in order to run the demo (and topological_tools must be compiled accordingly).")
endif ( NOT_SDLIMAGE_FOUND )

catkin_package(
  CATKIN_DEPENDS mdm_library
                 predicate_manager
                 topological_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(predicates src/predicates.cpp)

target_link_libraries(predicates ${catkin_LIBRARIES})

add_executable(state_layer src/state_layer.cpp)

target_link_libraries(state_layer ${catkin_LIBRARIES})

add_executable(action_layer src/action_layer.cpp)

target_link_libraries(action_layer ${catkin_LIBRARIES})

add_executable(control_layer src/control_layer.cpp)

target_link_libraries(control_layer ${catkin_LIBRARIES})

set_target_properties(control_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")