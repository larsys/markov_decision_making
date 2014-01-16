cmake_minimum_required(VERSION 2.8.3)
project(mdm_example)

find_package(catkin REQUIRED COMPONENTS markov_decision_making
                                        predicate_manager
                                        topological_tools)

find_package( SDL )
find_package( SDL_image )

if ( NOT_SDLIMAGE_FOUND )
  message(FATAL_ERROR, "SDL_image must be installed in order to run the demo (and topological_tools must be compiled accordingly).")
endif ( NOT_SDLIMAGE_FOUND )

catkin_package(
  CATKIN_DEPENDS markov_decision_making
                 predicate_manager
                 topological_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(demo_predicates src/demo_predicates.cpp)

target_link_libraries(demo_predicates ${catkin_LIBRARIES})

add_executable(demo_state_layer src/demo_state_layer.cpp)

target_link_libraries(demo_state_layer ${catkin_LIBRARIES})

add_executable(demo_action_layer src/demo_action_layer.cpp)

target_link_libraries(demo_action_layer ${catkin_LIBRARIES})

add_executable(demo_control_layer src/demo_control_layer.cpp)

target_link_libraries(demo_control_layer ${catkin_LIBRARIES})

set_target_properties(demo_control_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")