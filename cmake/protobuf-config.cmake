# Stub protobuf config to override anaconda's conflicting CMake config.
# Instead of setting _FOUND=FALSE (which causes cmake to keep searching and
# find the anaconda config), we directly forward to cmake's built-in
# FindProtobuf module which uses the system protobuf (v3.6.1).
include(${CMAKE_ROOT}/Modules/FindProtobuf.cmake)
