cmake_minimum_required(VERSION 3.20)
project(Door2Door2)
include(FetchContent)

if(POLICY CMP0135)
	cmake_policy(SET CMP0135 NEW)
	set(CMAKE_POLICY_DEFAULT_CMP0135 NEW)
endif()

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.2/json.tar.xz)
message( STATUS "Fetching Thor..." )
FetchContent_MakeAvailable(json)

set(CMAKE_CXX_STANDARD 17)

# build tren windows thi bat len
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -static-libstdc++ -static")

add_executable(Door2Door2 src/Config.cpp src/Input.cpp src/Solution.cpp src/TabuSearch.cpp main.cpp src/Utils.cpp src/Utils.h src/Random.h src/LocalSearch.cpp src/LocalSearch.h)
target_link_libraries(Door2Door2 PRIVATE nlohmann_json::nlohmann_json)

