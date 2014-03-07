cmake_minimum_required(VERSION 2.8.3)

project(mdm_library)

SET ( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMakeModules )

find_package(catkin REQUIRED COMPONENTS
                             roscpp
                             message_generation
                             std_msgs
                             predicate_manager)

find_package( MADP )

add_message_files(FILES
                  ActionSymbol.msg 
                  WorldSymbol.msg
                  SymbolMetadata.msg
                  FactoredSymbolMetadata.msg 
                  BeliefStateInfo.msg
                  FactoredDistribution.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES mdm_control_layer mdm_learning_layer mdm_action_layer mdm_state_layer
   CATKIN_DEPENDS roscpp predicate_manager message_generation
  )

if ( MADP_FOUND )
    add_library(mdm_control_layer
                src/control_layer_base.cpp
                src/controller_mdp.cpp
                src/controller_timed_mdp.cpp
                src/controller_event_mdp.cpp
                src/controller_pomdp.cpp
                src/controller_timed_pomdp.cpp
                src/controller_event_pomdp.cpp
                src/decpomdp_loader.cpp)

    add_library(mdm_learning_layer
                src/control_layer_base.cpp
                src/controller_mdp.cpp
                src/controller_timed_mdp.cpp
                src/controller_event_mdp.cpp
                src/controller_pomdp.cpp
                src/controller_timed_pomdp.cpp
                src/controller_event_pomdp.cpp
                src/decpomdp_loader.cpp)
		#src/online_learning_mdp.cpp
                #src/q_learning_mdp.cpp
		#src/sarsa_learning_mdp.cpp)
            
    set_target_properties(mdm_control_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")
    set_target_properties(mdm_learning_layer PROPERTIES COMPILE_FLAGS "-std=c++0x")
    add_definitions(-DHAVE_MADP)
else ( MADP_FOUND )
    message(STATUS "WARNING: MADP was not found on your system. Compiling without MADP support. Some functionalities may be absent.")
    add_library(mdm_control_layer
                src/control_layer_base.cpp
                src/controller_mdp.cpp
                src/controller_timed_mdp.cpp
                src/controller_event_mdp.cpp
                src/decpomdp_loader.cpp)

    add_library(mdm_learning_layer
                src/control_layer_base.cpp
                src/controller_mdp.cpp
                src/controller_timed_mdp.cpp
                src/controller_event_mdp.cpp
                src/decpomdp_loader.cpp)
		#src/online_learning_mdp.cpp
                #src/q_learning_mdp.cpp
		#src/sarsa_learning_mdp.cpp)
            
endif ( MADP_FOUND )

add_dependencies(mdm_control_layer ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(mdm_control_layer ${MADP_LIBRARIES})

add_dependencies(mdm_learning_layer ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(mdm_learning_layer ${MADP_LIBRARIES})

add_library(mdm_action_layer
            src/action_layer.cpp)

add_dependencies(mdm_action_layer ${PROJECT_NAME}_generate_messages_cpp)
            
include_directories(include ${catkin_INCLUDE_DIRS})


add_library(mdm_state_layer
            src/state_layer.cpp)
             
add_dependencies(mdm_state_layer ${PROJECT_NAME}_generate_messages_cpp)

add_library(mdm_observation_layer
            src/observation_layer.cpp)

add_dependencies(mdm_observation_layer ${PROJECT_NAME}_generate_messages_cpp)