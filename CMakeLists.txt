set(LIB_NAME mc_r_arm_push_unknown_wall)

set(controller_SRC
  src/mc_r_arm_push_unknown_wall.cpp
)

set(controller_HDR
  src/mc_r_arm_push_unknown_wall.h
)
macro(add_state state_name)
    add_library(${state_name} SHARED src/states/${state_name}.cpp src/states/${state_name}.h)
    set_target_properties(${state_name} PROPERTIES PREFIX "")
    target_link_libraries(${state_name} mc_control_fsm)
    target_link_libraries(${state_name} ${LIB_NAME})
    install(TARGETS ${state_name}
      DESTINATION "${MC_RTC_LIBDIR}/mc_controller/r_arm_push_unknown_wall/states")
endmacro()

add_state(PrepareHand)
add_state(DynamicContact)

include_directories(mc_prediction)

add_library(${LIB_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${LIB_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
pkg_config_use_dependency(${LIB_NAME} mc_rtc)
target_link_libraries(${LIB_NAME} mc_impact_predictor)
target_link_libraries(${LIB_NAME} mc_control_fsm)
install(TARGETS ${LIB_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)


macro(find_description_package desc_pkg)
  string(TOUPPER "${desc_pkg}" "DESC_PKG")
  if(NOT DEFINED ${DESC_PKG}_PATH)
    execute_process(
      COMMAND "${PKG_CONFIG_EXECUTABLE}"
      "--variable=datadir" "${desc_pkg}"
      RESULT_VARIABLE "${DESC_PKG}_RESULT"
      OUTPUT_VARIABLE "${DESC_PKG}_PATH"
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(${${DESC_PKG}_RESULT})
      set(${DESC_PKG}_FOUND False)
      message(WARNING "${DESC_PKG}_PATH is not defined and pkg-config failed to find ${desc_pkg}, the related robots' modules will not work properly. Check your system configuration or define ${DESC_PKG} to fix this issue.")
    else()
      message(STATUS "Found ${desc_pkg}: ${${DESC_PKG}_PATH}")
      set(${DESC_PKG}_FOUND True)
    endif()
  else()
    set(${DESC_PKG}_FOUND True)
    message(STATUS "Found ${desc_pkg}: ${${DESC_PKG}_PATH}")
  endif()
endmacro()

find_description_package(mc_env_description)
find_description_package(mc_int_obj_description)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/etc/RArmPushUnknownWall.conf.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/etc/RArmPushUnknownWall.conf)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/etc/RArmPushUnknownWall.conf DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc)


