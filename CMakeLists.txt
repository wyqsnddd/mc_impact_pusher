cmake_minimum_required(VERSION 3.1.1)

include(cmake/base.cmake)

set(PROJECT_NAME mc_impact_pusher)
set(PROJECT_DESCRIPTION "Multi-contact impact controller that generates an energetic push.")
set(PROJECT_URL https://github.com/wyqsnddd/mc_impact_pusher)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CXX_DISABLE_WERROR 1)
set(CMAKE_CXX_STANDARD 11)
setup_project()

find_package(mc_rtc REQUIRED)
find_package(mc_impact_constraints REQUIRED)
find_package(mc_impact_predictor REQUIRED)



add_subdirectory(src)

find_description_package(mc_env_description)
find_description_package(mc_int_obj_description)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/etc/Pusher.conf.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/etc/Pusher.conf)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/Pusher.conf DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc)



setup_project_finalize()
