project(OpenTX)

set(VERSION_MAJOR "2")
set(VERSION_MINOR "3")
set(VERSION_REVISION "15")
if(TEST_BUILD_WARNING OR ALLOW_NIGHTLY_BUILDS)
  math(EXPR VERSION_REVISION "${VERSION_REVISION} + 1")
endif()
set(VERSION_SUFFIX $ENV{OPENTX_VERSION_SUFFIX})
set(VERSION_FAMILY ${VERSION_MAJOR}.${VERSION_MINOR})
set(VERSION ${VERSION_MAJOR}.${VERSION_MINOR}.${VERSION_REVISION}${VERSION_SUFFIX})
set(SDCARD_REVISION "0039")
set(SDCARD_VERSION  ${VERSION_MAJOR}.${VERSION_MINOR}V${SDCARD_REVISION})

cmake_minimum_required(VERSION 2.8)
cmake_policy(SET CMP0020 NEW)
cmake_policy(SET CMP0023 OLD)
if(POLICY CMP0042)
  cmake_policy(SET CMP0042 NEW)
endif()
if(POLICY CMP0043)
  cmake_policy(SET CMP0043 NEW)
endif()
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
if(POLICY CMP0058)
  cmake_policy(SET CMP0058 NEW)  # for ninja
endif()

set(CMAKE_COLOR_MAKEFILE ON)

set(CMAKE_CXX_STANDARD 11)

set(RADIO_DIRECTORY ${PROJECT_SOURCE_DIR}/radio)
set(RADIO_SRC_DIRECTORY ${RADIO_DIRECTORY}/src)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)

include(Macros)
today(DATE)
now(TIME)
git_id(GIT_STR)

message(STATUS "OpenTX ${VERSION} @${GIT_STR}")

# options shared by all targets

option(APP_CUSTOM_DBG_HANDLER "Use custom message handler for debug output in all Qt apps." ON)
set(APP_CUSTOM_DBG_LEVEL "0" CACHE STRING
  "Debug output level: 0=debug+; 1=info+; 2=warn+; 3=critical+; 4=fatal only.")
set(APP_CUSTOM_DBG_SHOW_FILE "DEFAULT" CACHE STRING
  "Show source file names in debug output. 1/0/DEFAULT (default is ON with DEBUG builds).")

option(VERBOSE_CMAKELISTS "Show extra information while processing CMakeLists.txt files." OFF)
option(WARNINGS_AS_ERRORS "Treat any compiler warning as an error (adds -Werror flag)." OFF)


# Python check
find_package("PythonInterp")
if(PYTHONINTERP_FOUND)
  message(STATUS "Python found, version: ${PYTHON_VERSION_STRING}")
  get_filename_component(PYTHON_DIRECTORY ${PYTHON_EXECUTABLE} DIRECTORY)
else()
  message(WARNING "Python not found! Most firmware and simu flavors not buildable.")
  set(LUA NO)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")  # regular Clang or AppleClang
  set(CLANG TRUE)
endif()

option(DISABLE_COMPANION "Disable building companion and simulators" OFF)



# Check for a file that is typically left from a OpenTX 2.1 build and abort if found
if (EXISTS ${RADIO_SRC_DIRECTORY}/stamp.h OR EXISTS ${RADIO_SRC_DIRECTORY}/translations/en.h)
   message(FATAL_ERROR "Source directory contains files leftover from a OpenTX 2.1 build. Please run `git clean -f` in source directory (Careful: Will remove any extra files) or do a new clean git checkout")
endif()


add_subdirectory(${RADIO_SRC_DIRECTORY})


