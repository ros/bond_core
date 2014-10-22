if(WIN32)
  find_library(UUID_LIBRARY NAMES Rpcrt4 PATH)

  if(UUID_LIBRARY)
    set(UUID_FOUND true)
  endif(UUID_LIBRARY)

else()
  find_path(UUID_INCLUDE_DIR uuid/uuid.h)
  find_library(UUID_LIBRARY NAMES uuid PATH)

  if(UUID_INCLUDE_DIR)
    set(UUID_FOUND true)
  endif(UUID_INCLUDE_DIR)

  if(NOT UUID_LIBRARY)
    set(UUID_LIBRARY "")
  endif(NOT UUID_LIBRARY)

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
