## FindMADP
## Mantainer: Joao Messias <jmessias@isr.ist.utl.pt>

IF (MADP_INCLUDE_DIRS AND MADP_LIBRARIES)
   # in cache already - wont print a message
   SET(MADP_FIND_QUIETLY TRUE)
ENDIF ()

########################################################################
##  general find

FIND_PATH( MADP_INCLUDE_DIRS madp/Globals.h
           /usr/include
           /usr/local/include
           $ENV{MADP_ROOT}
           $ENV{MADP_ROOT}/include
           DOC "directory containing madp/*.h for the MADP library" )

IF(EXISTS ${MADP_INCLUDE_DIRS})
  SET(MADP_LIBRARIES "-lMADPBase -lMADPSupport -lMADPPlanning -lMADPParser -lPOMDPSolve -lLPSolveOld -lmdp")
ENDIF()

########################################################################
## finished - now just set up flags

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(MADP DEFAULT_MSG MADP_LIBRARIES MADP_INCLUDE_DIRS)

MARK_AS_ADVANCED(MADP_INCLUDE_DIRS MADP_LIBRARIES)
