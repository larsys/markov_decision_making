cmake_minimum_required(VERSION 2.8.3)
project(learning_test)

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

add_executable(learning_predicates src/learning_predicates.cpp)

target_link_libraries(learning_predicates ${catkin_LIBRARIES})

add_executable(learning_state_layer src/learning_state_layer.cpp)

target_link_libraries(learning_state_layer ${catkin_LIBRARIES})

add_executable(learning_action_layer src/learning_action_layer.cpp)

target_link_libraries(learning_action_layer ${catkin_LIBRARIES})

add_executable(learning_control_q_layer src/learning_control_q_layer.cpp)

target_link_libraries(learning_control_q_layer ${catkin_LIBRARIES})

set_target_properties(learning_control_q_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")

add_executable(learning_control_sarsa_layer src/learning_control_sarsa_layer.cpp)

target_link_libraries(learning_control_sarsa_layer ${catkin_LIBRARIES})

set_target_properties(learning_control_sarsa_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")