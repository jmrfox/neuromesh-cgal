cmake_minimum_required(VERSION 3.12)
project(neuromesh-cgal)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(CGAL REQUIRED)
find_package(Eigen3 3.2 REQUIRED NO_MODULE)

include_directories(${CGAL_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
add_definitions(-DCGAL_EIGEN3_ENABLED)

add_executable(mesh_simplify src/mesh_simplify.cpp)
target_link_libraries(mesh_simplify CGAL::CGAL)

add_executable(mesh_smooth src/mesh_smooth.cpp)
target_link_libraries(mesh_smooth CGAL::CGAL)
target_include_directories(mesh_smooth PRIVATE ${EIGEN3_INCLUDE_DIR})

add_executable(mesh_remesh src/mesh_remesh.cpp)
target_link_libraries(mesh_remesh CGAL::CGAL)

add_executable(mesh_repair src/mesh_repair.cpp)
target_link_libraries(mesh_repair CGAL::CGAL)
