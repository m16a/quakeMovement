cmake_minimum_required(VERSION 2.8.9)
project (quakeMovement)

include_directories(include)

set (server_SRC smain.cpp message.cpp shared.cpp)
set (client_SRC cmain.cpp message.cpp shared.cpp)
add_executable(client ${client_SRC})
add_executable(server ${server_SRC})

find_library(RAKNET_LIBRARY NAMES libRakNetDLL.so PATHS ${CMAKE_SOURCE_DIR})

find_library(ODE_LIBRARY NAMES libode.a)
find_library(DRAWSTFUF_LIBRARY NAMES libdrawstuff.a PATHS ${CMAKE_SOURCE_DIR})
	
find_package (Threads)
find_package(OpenGL)
find_package(X11)

target_link_libraries(client PUBLIC ${RAKNET_LIBRARY} ${ODE_LIBRARY} ${DRAWSTFUF_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${OPENGL_LIBRARIES} ${X11_LIBRARIES})
target_link_libraries(server PUBLIC ${RAKNET_LIBRARY} ${ODE_LIBRARY} ${DRAWSTFUF_LIBRARY} ${CMAKE_THREAD_LIBS_INIT} ${OPENGL_LIBRARIES} ${X11_LIBRARIES})
