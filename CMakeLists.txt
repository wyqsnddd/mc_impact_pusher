add_subdirectory(src)

find_description_package(mc_env_description)
find_description_package(mc_int_obj_description)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/etc/RArmPushUnknownWall.conf.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/etc/RArmPushUnknownWall.conf)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/RArmPushUnknownWall.conf DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc)

