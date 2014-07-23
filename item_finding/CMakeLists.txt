cmake_minimum_required(VERSION 2.8.3)
project(item_finding)

find_package(catkin REQUIRED COMPONENTS mdm_library
                                        predicate_manager
                                        topological_tools
					rospy
					genmsg
					actionlib_msgs
					actionlib)

find_package( SDL )
find_package( SDL_image )

if ( NOT_SDLIMAGE_FOUND )
  message(FATAL_ERROR, "SDL_image must be installed in order to run the demo (and topological_tools must be compiled accordingly).")
endif ( NOT_SDLIMAGE_FOUND )

# Actionlib msg
#find_package(catkin REQUIRED genmsg actionlib_msgs actionlib)
add_action_files(DIRECTORY action FILES HandleObject.action)
generate_messages(DEPENDENCIES actionlib_msgs)

catkin_package(
  CATKIN_DEPENDS mdm_library
                 predicate_manager
                 topological_tools
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(if_predicates src/if_predicates.cpp)

target_link_libraries(if_predicates ${catkin_LIBRARIES})

add_executable(if_state_layer src/if_state_layer.cpp)

target_link_libraries(if_state_layer ${catkin_LIBRARIES})

add_executable(if_action_layer src/if_action_layer.cpp)

target_link_libraries(if_action_layer ${catkin_LIBRARIES})

add_executable(if_control_q_layer src/if_control_q_layer.cpp)

target_link_libraries(if_control_q_layer ${catkin_LIBRARIES})

set_target_properties(if_control_q_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")

add_executable(if_control_sarsa_layer src/if_control_sarsa_layer.cpp)

target_link_libraries(if_control_sarsa_layer ${catkin_LIBRARIES})

set_target_properties(if_control_sarsa_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")
