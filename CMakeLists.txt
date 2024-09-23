cmake_minimum_required(VERSION 3.15...3.30)

project(excavator_problem VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE Debug)

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 

find_package(Boost 1.71.0 COMPONENTS graph unit_test_framework program_options log log_setup REQUIRED) 

add_compile_options(-Wall -Wextra -Wpedantic -Werror)

add_library(planner SHARED src/planner_starter_code.cpp 
                           src/assignment.cpp
                           src/grid_search.cpp
                           src/conflict_based_search.cpp)
target_include_directories(planner PUBLIC ./include ${Boost_INCLUDE_DIRS})
target_link_libraries(planner ${Boost_LIBRARIES})

add_executable(run src/main.cpp)
target_include_directories(run PRIVATE ./include)
target_link_libraries(run planner)
 
# Tests

add_executable(test_starter_code test/test_starter_code.cpp)
target_include_directories(test_starter_code PRIVATE ./include)
target_link_libraries(test_starter_code planner ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_assignment test/test_assignment.cpp)
target_include_directories(test_assignment PRIVATE ./include)
target_link_libraries(test_assignment planner ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_grid_search test/test_grid_search.cpp)
target_include_directories(test_grid_search PRIVATE ./include)
target_link_libraries(test_grid_search planner ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

add_executable(test_conflict_based_search test/test_conflict_based_search.cpp)
target_include_directories(test_conflict_based_search PRIVATE ./include)
target_link_libraries(test_conflict_based_search planner ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})

