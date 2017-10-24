INCLUDE(FindPackageHandleStandardArgs)

FIND_PATH(Aravis_INCLUDE_PATH 
NAMES arv.h
HINTS
  "$ENV{Aravis_INCLUDE_PATH}"
  /usr/local/include
  /usr/include
PATH_SUFFIXES
  aravis-0.4
  aravis-0.6
)

FIND_LIBRARY(Aravis_LIBRARIES
NAMES
  libaravis-0.4
  libaravis-0.6
  aravis
  aravis-0.4
  aravis-0.6
  libaravis

HINTS
  "$ENV{ARAVIS_LIBRARY}"
  /usr/local/lib
  /usr/lib
)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(Aravis DEFAULT_MSG
  Aravis_INCLUDE_PATH
  Aravis_LIBRARIES)

