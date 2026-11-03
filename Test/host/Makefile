# Host unit tests (gcc, AddressSanitizer + UndefinedBehaviorSanitizer).
#   make -C Test/host        build and run all tests

CC      ?= gcc
CFLAGS  := -std=c99 -Wall -Wextra -g -fsanitize=address,undefined \
           -Imock -I../../Core/Inc -I../../App/Inc
BUILD   := build
APP     := ../../App/Src
CORE    := ../../Core/Src

TESTS := $(BUILD)/test_phy $(BUILD)/test_gear $(BUILD)/test_monitor

all: $(TESTS)
	@for t in $(TESTS); do ./$$t || exit 1; done

$(BUILD)/test_phy: test_phy.c $(CORE)/dali_phy.c mock/mock_hw.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@

$(BUILD)/test_gear: test_gear.c $(APP)/dali_gear.c $(APP)/light_ctrl.c $(APP)/dali_membank.c \
                    $(APP)/nvm.c $(APP)/variant.c $(APP)/crc16.c mock/mock_hw.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@

$(BUILD)/test_monitor: test_monitor.c $(APP)/led_monitor.c $(APP)/light_ctrl.c $(APP)/opdata.c \
                       $(APP)/nvm.c $(APP)/crc16.c mock/mock_hw.c
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean
