
set(srcs
  main.cpp
  )

add_executable(${PROJECT_NAME} ${srcs})
add_definitions("-O2 -MMD -MP -Wall -Wextra -std=c++11")



#
# matplot-libcpp
#
find_package(PythonLibs REQUIRED) # 2.7
# find_package(PythonLibs 3.5) # 3.5
message (STATUS "${PYTHON_INCLUDE_DIRS}")
message (STATUS "${PYTHON_LIBRARIES}")
include_directories(${PYTHON_INCLUDE_DIRS})
# target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES})


target_link_libraries(${PROJECT_NAME} ${PYTHON_LIBRARIES} Simulator Controller Model)


add_subdirectory(model)
add_subdirectory(controller)
add_subdirectory(simulator)

