cmake_minimum_required(VERSION 3.16)
project(casper2_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra -DUNIT_TEST -DARM_MATH_CM7")
# Coverage flags (optional, enabled by coverage.sh)
# set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")

# Suppress some warnings from firmware code
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-unused-parameter -Wno-unused-variable -Wno-missing-field-initializers")

# ---- Paths ----
set(FIRMWARE_ROOT ${CMAKE_SOURCE_DIR}/../Software)
set(APP_ROOT ${FIRMWARE_ROOT}/App)

# ---- CSV data path define ----
add_compile_definitions(CSV_DATA_DIR="${CMAKE_SOURCE_DIR}/CSVs")

# ---- Include paths (stubs override real HAL headers) ----
include_directories(
    ${CMAKE_SOURCE_DIR}/stubs        # HAL stubs FIRST (override real headers)
    ${CMAKE_SOURCE_DIR}/mocks
    ${CMAKE_SOURCE_DIR}/unity
    ${CMAKE_SOURCE_DIR}              # test_config.h
    ${APP_ROOT}/nav
    ${APP_ROOT}/command
    ${APP_ROOT}/pyro
    ${APP_ROOT}/pack
    ${APP_ROOT}/telemetry
    ${APP_ROOT}/fsm
    ${APP_ROOT}/flight
    ${APP_ROOT}/drivers
    ${APP_ROOT}/cal
    ${APP_ROOT}/diag
    ${APP_ROOT}/util
)

# ---- Unity library ----
add_library(unity STATIC unity/unity.c)

# ---- Stub/mock libraries ----
add_library(stubs STATIC
    stubs/stub_helpers.c
    stubs/arm_math_stub.c
    stubs/global_handles.c
)

add_library(mock_tick STATIC mocks/mock_tick.c)
add_library(mock_gpio STATIC mocks/mock_gpio.c)
add_library(mock_adc STATIC mocks/mock_adc.c)

# ---- Common firmware sources (math/nav — no HAL deps) ----
set(NAV_SOURCES
    ${APP_ROOT}/nav/casper_quat.c
    ${APP_ROOT}/nav/casper_ekf.c
)

set(TELEMETRY_SOURCES
    ${APP_ROOT}/telemetry/cobs.c
    ${APP_ROOT}/telemetry/crc32_hw.c
)

set(PACK_SOURCES
    ${APP_ROOT}/pack/status_pack.c
    ${APP_ROOT}/pack/quat_pack.c
)

# ======================================================================
#  TIER 1 — Pure Math Unit Tests
# ======================================================================

# test_casper_quat
add_executable(test_casper_quat
    tier1_unit/test_casper_quat.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_casper_quat unity stubs m)
add_test(NAME quat COMMAND test_casper_quat)

# test_casper_ekf
add_executable(test_casper_ekf
    tier1_unit/test_casper_ekf.c
    ${APP_ROOT}/nav/casper_ekf.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_casper_ekf unity stubs m)
add_test(NAME ekf COMMAND test_casper_ekf)

# test_casper_attitude
add_executable(test_casper_attitude
    tier1_unit/test_casper_attitude.c
    ${APP_ROOT}/nav/casper_attitude.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_casper_attitude unity stubs m)
add_test(NAME attitude COMMAND test_casper_attitude)

# test_casper_gyro_int (deprecated but still tested)
if(EXISTS ${APP_ROOT}/nav/casper_gyro_int.c)
    add_executable(test_casper_gyro_int
        tier1_unit/test_casper_gyro_int.c
        ${APP_ROOT}/nav/casper_gyro_int.c
        ${APP_ROOT}/nav/casper_quat.c
    )
    target_link_libraries(test_casper_gyro_int unity stubs m)
    add_test(NAME gyro_int COMMAND test_casper_gyro_int)
endif()

# test_cobs
add_executable(test_cobs
    tier1_unit/test_cobs.c
    ${APP_ROOT}/telemetry/cobs.c
)
target_link_libraries(test_cobs unity stubs m)
add_test(NAME cobs COMMAND test_cobs)

# test_crc32
add_executable(test_crc32
    tier1_unit/test_crc32.c
    ${APP_ROOT}/telemetry/crc32_hw.c
)
target_link_libraries(test_crc32 unity stubs m)
add_test(NAME crc32 COMMAND test_crc32)

# test_status_pack
add_executable(test_status_pack
    tier1_unit/test_status_pack.c
    ${APP_ROOT}/pack/status_pack.c
)
target_link_libraries(test_status_pack unity stubs m)
add_test(NAME status_pack COMMAND test_status_pack)

# test_ms5611_math
add_executable(test_ms5611_math
    tier1_unit/test_ms5611_math.c
    ${APP_ROOT}/drivers/ms5611.c
)
target_link_libraries(test_ms5611_math unity stubs mock_tick m)
add_test(NAME ms5611_math COMMAND test_ms5611_math)

# test_tlm_pack
add_executable(test_tlm_pack
    tier1_unit/test_tlm_pack.c
    ${APP_ROOT}/pack/quat_pack.c
)
target_link_libraries(test_tlm_pack unity stubs m)
add_test(NAME tlm_pack COMMAND test_tlm_pack)

# ======================================================================
#  TIER 2 — Protocol and FSM Tests
# ======================================================================

# test_flight_fsm
if(EXISTS ${APP_ROOT}/fsm/flight_fsm.c)
    add_executable(test_flight_fsm
        tier2_protocol/test_flight_fsm.c
        ${APP_ROOT}/fsm/flight_fsm.c
        ${APP_ROOT}/telemetry/cobs.c
        ${APP_ROOT}/telemetry/crc32_hw.c
    )
    target_link_libraries(test_flight_fsm unity stubs mock_tick m)
    add_test(NAME flight_fsm COMMAND test_flight_fsm)
endif()

# test_pyro_manager
if(EXISTS ${APP_ROOT}/pyro/pyro_manager.c)
    add_executable(test_pyro_manager
        tier2_protocol/test_pyro_manager.c
        ${APP_ROOT}/pyro/pyro_manager.c
        ${APP_ROOT}/pyro/casper_pyro.c
    )
    target_link_libraries(test_pyro_manager unity stubs mock_tick mock_gpio mock_adc m)
    add_test(NAME pyro_manager COMMAND test_pyro_manager)
endif()

# test_cac_handler
if(EXISTS ${APP_ROOT}/command/cac_handler.c)
    add_executable(test_cac_handler
        tier2_protocol/test_cac_handler.c
        ${APP_ROOT}/command/cac_handler.c
        ${APP_ROOT}/telemetry/cobs.c
        ${APP_ROOT}/telemetry/crc32_hw.c
        ${APP_ROOT}/pyro/pyro_manager.c
        ${APP_ROOT}/pyro/casper_pyro.c
    )
    target_link_libraries(test_cac_handler unity stubs mock_tick mock_gpio mock_adc m)
    add_test(NAME cac_handler COMMAND test_cac_handler)
endif()

# test_cmd_router
if(EXISTS ${APP_ROOT}/command/cmd_router.c)
    add_executable(test_cmd_router
        tier2_protocol/test_cmd_router.c
        ${APP_ROOT}/command/cmd_router.c
        ${APP_ROOT}/telemetry/cobs.c
        ${APP_ROOT}/telemetry/crc32_hw.c
    )
    target_link_libraries(test_cmd_router unity stubs mock_tick m)
    add_test(NAME cmd_router COMMAND test_cmd_router)
endif()

# test_telemetry_framing
add_executable(test_telemetry_framing
    tier2_protocol/test_telemetry_framing.c
    ${APP_ROOT}/telemetry/cobs.c
    ${APP_ROOT}/telemetry/crc32_hw.c
)
target_link_libraries(test_telemetry_framing unity stubs m)
add_test(NAME telemetry_framing COMMAND test_telemetry_framing)

# ======================================================================
#  TIER 3 — Integration Tests
# ======================================================================

# test_nav_pipeline
add_executable(test_nav_pipeline
    tier3_integration/test_nav_pipeline.c
    ${APP_ROOT}/nav/casper_ekf.c
    ${APP_ROOT}/nav/casper_attitude.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_nav_pipeline unity stubs m)
add_test(NAME nav_pipeline COMMAND test_nav_pipeline)

# test_flight_loop
add_executable(test_flight_loop
    tier3_integration/test_flight_loop.c
    ${APP_ROOT}/nav/casper_ekf.c
    ${APP_ROOT}/nav/casper_attitude.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_flight_loop unity stubs mock_tick m)
add_test(NAME flight_loop COMMAND test_flight_loop)

# ======================================================================
#  REGRESSION — CSV-Driven Tests
# ======================================================================

# CSV loader library
add_library(csv_loader STATIC
    regression/csv_loader.c
)

# Regression helpers library
if(EXISTS ${CMAKE_SOURCE_DIR}/regression/regression_helpers.c)
    add_library(regression_helpers STATIC
        regression/regression_helpers.c
    )
    target_link_libraries(regression_helpers csv_loader m)
endif()

# test_ekf_regression
add_executable(test_ekf_regression
    regression/test_ekf_regression.c
    ${APP_ROOT}/nav/casper_ekf.c
    ${APP_ROOT}/nav/casper_attitude.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_ekf_regression unity stubs csv_loader m)
if(TARGET regression_helpers)
    target_link_libraries(test_ekf_regression regression_helpers)
endif()
add_test(NAME ekf_regression COMMAND test_ekf_regression)
set_tests_properties(ekf_regression PROPERTIES TIMEOUT 30)

# test_attitude_regression
add_executable(test_attitude_regression
    regression/test_attitude_regression.c
    ${APP_ROOT}/nav/casper_attitude.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_attitude_regression unity stubs csv_loader m)
if(TARGET regression_helpers)
    target_link_libraries(test_attitude_regression regression_helpers)
endif()
add_test(NAME attitude_regression COMMAND test_attitude_regression)
set_tests_properties(attitude_regression PROPERTIES TIMEOUT 30)

# ======================================================================
#  COMPLIANCE — Spec Constant Validation
# ======================================================================

# test_ekf_spec
add_executable(test_ekf_spec
    compliance/test_ekf_spec.c
    ${APP_ROOT}/nav/casper_ekf.c
    ${APP_ROOT}/nav/casper_quat.c
)
target_link_libraries(test_ekf_spec unity stubs m)
add_test(NAME ekf_spec COMMAND test_ekf_spec)

# test_interface_spec
add_executable(test_interface_spec
    compliance/test_interface_spec.c
)
target_link_libraries(test_interface_spec unity stubs m)
add_test(NAME interface_spec COMMAND test_interface_spec)

# test_sensor_spec
add_executable(test_sensor_spec
    compliance/test_sensor_spec.c
)
target_link_libraries(test_sensor_spec unity stubs m)
add_test(NAME sensor_spec COMMAND test_sensor_spec)

# test_pyro_spec
add_executable(test_pyro_spec
    compliance/test_pyro_spec.c
)
target_link_libraries(test_pyro_spec unity stubs m)
add_test(NAME pyro_spec COMMAND test_pyro_spec)

# test_naming_convention
add_executable(test_naming_convention
    compliance/test_naming_convention.c
)
target_link_libraries(test_naming_convention unity stubs m)
add_test(NAME naming_convention COMMAND test_naming_convention)

# JSF compliance (shell script)
add_test(NAME jsf_compliance
    COMMAND bash ${CMAKE_SOURCE_DIR}/scripts/jsf_check.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/..
)

enable_testing()
