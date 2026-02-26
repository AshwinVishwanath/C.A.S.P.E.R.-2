# C.A.S.P.E.R.-2 Test Harness Makefile
# Host-side unit/integration tests using mingw64 GCC
# Usage: make -j8 && make test

# Ensure temp dir is writable (Windows workaround)
export TMP := $(if $(TMP),$(TMP),$(USERPROFILE)/AppData/Local/Temp)
export TEMP := $(if $(TEMP),$(TEMP),$(USERPROFILE)/AppData/Local/Temp)

CC = gcc
CFLAGS = -std=c11 -Wall -Wextra -DUNIT_TEST -DARM_MATH_CM7 -pipe
CFLAGS += -Wno-unused-parameter -Wno-unused-variable -Wno-missing-field-initializers
CFLAGS += -Wno-sign-compare
LDFLAGS = -lm

# Paths
TEST_DIR   = .
FW_ROOT    = ../Software
APP_ROOT   = $(FW_ROOT)/App
BUILD_DIR  = build

# CSV data path
CFLAGS += -DCSV_DATA_DIR=\"$(TEST_DIR)/CSVs\"

# Include paths (stubs FIRST to override real HAL headers)
INCLUDES  = -I$(TEST_DIR)/stubs
INCLUDES += -I$(TEST_DIR)/mocks
INCLUDES += -I$(TEST_DIR)/unity
INCLUDES += -I$(TEST_DIR)
INCLUDES += -I$(APP_ROOT)/nav
INCLUDES += -I$(APP_ROOT)/command
INCLUDES += -I$(APP_ROOT)/pyro
INCLUDES += -I$(APP_ROOT)/pack
INCLUDES += -I$(APP_ROOT)/telemetry
INCLUDES += -I$(APP_ROOT)/fsm
INCLUDES += -I$(APP_ROOT)/flight
INCLUDES += -I$(APP_ROOT)/drivers
INCLUDES += -I$(APP_ROOT)/cal
INCLUDES += -I$(APP_ROOT)/diag
INCLUDES += -I$(APP_ROOT)/util
INCLUDES += -I$(TEST_DIR)/regression

# ---- Source files ----
UNITY_SRC     = unity/unity.c
STUB_SRCS     = stubs/stub_helpers.c stubs/arm_math_stub.c stubs/global_handles.c
MOCK_TICK_SRC = mocks/mock_tick.c
MOCK_GPIO_SRC = mocks/mock_gpio.c
MOCK_ADC_SRC  = mocks/mock_adc.c
CSV_LOADER    = regression/csv_loader.c
REG_HELPERS   = regression/regression_helpers.c

# Firmware sources
QUAT_SRC      = $(APP_ROOT)/nav/casper_quat.c
EKF_SRC       = $(APP_ROOT)/nav/casper_ekf.c
ATT_SRC       = $(APP_ROOT)/nav/casper_attitude.c
GYRO_INT_SRC  = $(APP_ROOT)/nav/casper_gyro_int.c
COBS_SRC      = $(APP_ROOT)/telemetry/cobs.c
CRC_SRC       = $(APP_ROOT)/telemetry/crc32_hw.c
STATUS_SRC    = $(APP_ROOT)/pack/status_pack.c
QPACK_SRC     = $(APP_ROOT)/pack/quat_pack.c
MS5611_SRC    = $(APP_ROOT)/drivers/ms5611.c
FSM_SRC       = $(APP_ROOT)/fsm/flight_fsm.c
FSM_UTIL_SRC  = $(APP_ROOT)/fsm/fsm_util.c
PYRO_MGR_SRC  = $(APP_ROOT)/pyro/pyro_manager.c
PYRO_DRV_SRC  = $(APP_ROOT)/pyro/casper_pyro.c
CAC_SRC       = $(APP_ROOT)/command/cac_handler.c
CMD_SRC       = $(APP_ROOT)/command/cmd_router.c

# Common compile recipe
COMMON = $(UNITY_SRC) $(STUB_SRCS)

# ======================================================================
# Test targets
# ======================================================================

TIER1_TESTS = \
	$(BUILD_DIR)/test_casper_quat \
	$(BUILD_DIR)/test_casper_ekf \
	$(BUILD_DIR)/test_casper_attitude \
	$(BUILD_DIR)/test_cobs \
	$(BUILD_DIR)/test_crc32 \
	$(BUILD_DIR)/test_status_pack \
	$(BUILD_DIR)/test_ms5611_math \
	$(BUILD_DIR)/test_tlm_pack \
	$(BUILD_DIR)/test_fsm_util

TIER1_NAV_TESTS = \
	$(BUILD_DIR)/test_casper_gyro_int

TIER2_TESTS = \
	$(BUILD_DIR)/test_telemetry_framing

TIER3_TESTS = \
	$(BUILD_DIR)/test_nav_pipeline \
	$(BUILD_DIR)/test_flight_loop

REGRESSION_TESTS = \
	$(BUILD_DIR)/test_ekf_regression \
	$(BUILD_DIR)/test_attitude_regression

COMPLIANCE_TESTS = \
	$(BUILD_DIR)/test_ekf_spec \
	$(BUILD_DIR)/test_interface_spec \
	$(BUILD_DIR)/test_sensor_spec \
	$(BUILD_DIR)/test_pyro_spec \
	$(BUILD_DIR)/test_naming_convention

# Conditional tier2 tests (depend on firmware source existence)
TIER2_CONDITIONAL =

ifneq ($(wildcard $(FSM_SRC)),)
TIER2_CONDITIONAL += $(BUILD_DIR)/test_flight_fsm
TIER2_CONDITIONAL += $(BUILD_DIR)/test_fsm_transitions
TIER2_CONDITIONAL += $(BUILD_DIR)/test_hil_handler
endif
ifneq ($(wildcard $(PYRO_MGR_SRC)),)
TIER2_CONDITIONAL += $(BUILD_DIR)/test_pyro_manager
TIER2_CONDITIONAL += $(BUILD_DIR)/test_pyro_safety
endif
ifneq ($(wildcard $(CAC_SRC)),)
TIER2_CONDITIONAL += $(BUILD_DIR)/test_cac_handler
endif
ifneq ($(wildcard $(CMD_SRC)),)
TIER2_CONDITIONAL += $(BUILD_DIR)/test_cmd_router
endif
ifneq ($(wildcard $(GYRO_INT_SRC)),)
TIER1_TESTS += $(BUILD_DIR)/test_casper_gyro_int
endif

ALL_TESTS = $(TIER1_TESTS) $(TIER2_TESTS) $(TIER2_CONDITIONAL) $(TIER3_TESTS) $(REGRESSION_TESTS) $(COMPLIANCE_TESTS)

.PHONY: all clean test tier1 tier2 tier3 regression compliance

all: $(BUILD_DIR) $(ALL_TESTS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# ---- TIER 1: Pure Math ----

$(BUILD_DIR)/test_casper_quat: tier1_unit/test_casper_quat.c $(QUAT_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_casper_ekf: tier1_unit/test_casper_ekf.c $(EKF_SRC) $(QUAT_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_casper_attitude: tier1_unit/test_casper_attitude.c $(ATT_SRC) $(QUAT_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_casper_gyro_int: tier1_unit/test_casper_gyro_int.c $(GYRO_INT_SRC) $(QUAT_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_cobs: tier1_unit/test_cobs.c $(COBS_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_crc32: tier1_unit/test_crc32.c $(CRC_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_status_pack: tier1_unit/test_status_pack.c $(STATUS_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_ms5611_math: tier1_unit/test_ms5611_math.c $(MS5611_SRC) $(COMMON) $(MOCK_TICK_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_tlm_pack: tier1_unit/test_tlm_pack.c $(QPACK_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_fsm_util: tier1_unit/test_fsm_util.c $(FSM_UTIL_SRC) $(QUAT_SRC) $(COMMON) $(MOCK_TICK_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -DHIL_MODE -o $@ $^ $(LDFLAGS)

# ---- TIER 2: Protocol/FSM ----

$(BUILD_DIR)/test_flight_fsm: tier2_protocol/test_flight_fsm.c $(FSM_SRC) $(FSM_UTIL_SRC) $(QUAT_SRC) $(COBS_SRC) $(CRC_SRC) $(COMMON) $(MOCK_TICK_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -DHIL_MODE -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_fsm_transitions: tier2_protocol/test_fsm_transitions.c $(FSM_SRC) $(FSM_UTIL_SRC) $(QUAT_SRC) $(COMMON) $(MOCK_TICK_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -DHIL_MODE -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_hil_handler: tier2_protocol/test_hil_handler.c $(FSM_SRC) $(FSM_UTIL_SRC) $(QUAT_SRC) $(COMMON) $(MOCK_TICK_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -DHIL_MODE -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_pyro_manager: tier2_protocol/test_pyro_manager.c $(PYRO_MGR_SRC) $(PYRO_DRV_SRC) $(COMMON) $(MOCK_TICK_SRC) $(MOCK_GPIO_SRC) $(MOCK_ADC_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_pyro_safety: tier2_protocol/test_pyro_safety.c $(PYRO_MGR_SRC) $(PYRO_DRV_SRC) $(FSM_SRC) $(FSM_UTIL_SRC) $(QUAT_SRC) $(COMMON) $(MOCK_TICK_SRC) $(MOCK_GPIO_SRC) $(MOCK_ADC_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -DHIL_MODE -DPYRO_EXCLUDE_MASK=0x04 -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_cac_handler: tier2_protocol/test_cac_handler.c $(CAC_SRC) $(COBS_SRC) $(CRC_SRC) $(PYRO_MGR_SRC) $(PYRO_DRV_SRC) $(COMMON) $(MOCK_TICK_SRC) $(MOCK_GPIO_SRC) $(MOCK_ADC_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_cmd_router: tier2_protocol/test_cmd_router.c $(CMD_SRC) $(COBS_SRC) $(CRC_SRC) $(COMMON) $(MOCK_TICK_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_telemetry_framing: tier2_protocol/test_telemetry_framing.c $(COBS_SRC) $(CRC_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

# ---- TIER 3: Integration ----

$(BUILD_DIR)/test_nav_pipeline: tier3_integration/test_nav_pipeline.c $(EKF_SRC) $(ATT_SRC) $(QUAT_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_flight_loop: tier3_integration/test_flight_loop.c $(EKF_SRC) $(ATT_SRC) $(QUAT_SRC) $(COMMON) $(MOCK_TICK_SRC) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

# ---- REGRESSION ----

$(BUILD_DIR)/test_ekf_regression: regression/test_ekf_regression.c $(EKF_SRC) $(ATT_SRC) $(QUAT_SRC) $(CSV_LOADER) $(REG_HELPERS) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_attitude_regression: regression/test_attitude_regression.c $(ATT_SRC) $(QUAT_SRC) $(CSV_LOADER) $(REG_HELPERS) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

# ---- COMPLIANCE ----

$(BUILD_DIR)/test_ekf_spec: compliance/test_ekf_spec.c $(EKF_SRC) $(QUAT_SRC) $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_interface_spec: compliance/test_interface_spec.c $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_sensor_spec: compliance/test_sensor_spec.c $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_pyro_spec: compliance/test_pyro_spec.c $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

$(BUILD_DIR)/test_naming_convention: compliance/test_naming_convention.c $(COMMON) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $^ $(LDFLAGS)

# ---- Run all tests ----

test: all
	@echo "=== Running C.A.S.P.E.R.-2 Test Suite ==="
	@PASS=0; FAIL=0; TOTAL=0; \
	for test in $(ALL_TESTS); do \
		TOTAL=$$((TOTAL+1)); \
		name=$$(basename $$test); \
		if $$test > /dev/null 2>&1; then \
			PASS=$$((PASS+1)); \
			echo "  PASS: $$name"; \
		else \
			FAIL=$$((FAIL+1)); \
			echo "  FAIL: $$name"; \
			$$test 2>&1 | grep -i "fail" | head -5; \
		fi; \
	done; \
	echo ""; \
	echo "=== Results: $$PASS/$$TOTAL passed, $$FAIL failed ===";\
	exit $$FAIL

tier1: $(TIER1_TESTS)
	@for test in $(TIER1_TESTS); do $$(basename $$test) && echo "PASS: $$(basename $$test)" || echo "FAIL: $$(basename $$test)"; done

clean:
	rm -rf $(BUILD_DIR)
