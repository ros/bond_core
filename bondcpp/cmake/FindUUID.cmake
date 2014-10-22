if(WIN32)
  find_library(UUID_LIBRARIES NAMES Rpcrt4 PATH)

  if(UUID_LIBRARIES)
    set(UUID_FOUND true)
  endif(UUID_LIBRARIES)

else()
  find_path(UUID_INCLUDE_DIRS uuid/uuid.h)
  find_library(UUID_LIBRARIES NAMES uuid PATH)

  if(UUID_INCLUDE_DIRS)
    set(UUID_FOUND true)
  endif(UUID_INCLUDE_DIRS)

  if(NOT UUID_LIBRARIES)
    set(UUID_LIBRARIES "")
  endif(NOT UUID_LIBRARIES)

endif(WIN32)

if(NOT UUID_FOUND)
  if(UUID_FIND_REQUIRED)
    if(WIN32)
      message(FATAL_ERROR "Could not find Rpcrt4")
    else()
      message(FATAL_ERROR "Could not find UUID")
    endif(WIN32)
  endif(UUID_FIND_REQUIRED)
endif(NOT UUID_FOUND)
