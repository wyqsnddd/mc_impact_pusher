set(controller_SRC
  mc_r_arm_push_unknown_wall.cpp
)

set(controller_HDR
  mc_r_arm_push_unknown_wall.h
)

add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm)
target_link_libraries(${PROJECT_NAME} PUBLIC mc_impact_constraints::mc_impact_constraints)
install(TARGETS ${PROJECT_NAME} DESTINATION "${MC_RTC_LIBDIR}")

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})

add_subdirectory(states)

