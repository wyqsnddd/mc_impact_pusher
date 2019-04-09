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

add_library(${LIB_NAME} SHARED ${controller_SRC} ${controller_HDR})
set_target_properties(${LIB_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")
pkg_config_use_dependency(${LIB_NAME} mc_rtc)
target_link_libraries(${LIB_NAME} mc_control_fsm)
install(TARGETS ${LIB_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/etc/RArmPushUnknownWall.conf DESTINATION ${MC_RTC_LIBDIR}/mc_controller/etc)


