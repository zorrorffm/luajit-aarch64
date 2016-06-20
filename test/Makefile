# Default values for running luajit.
LUAJIT?=luajit
LUA_FLAGS?=-jdump=+rsXa
# Define tests which should be skipped for some reasons.
SKIPPED_LUA_TEST=mul div

# Find all tests inside the folder.
LUA_TEST_SRC=$(wildcard *.lua)
ALL_LUA_TEST=$(LUA_TEST_SRC:%.lua=test_%)

# Define the tests.
DISABLED_LUA_TEST:=$(SKIPPED_LUA_TEST:%=test_%)
ENABLED_LUA_TEST=$(filter-out $(DISABLED_LUA_TEST),$(ALL_LUA_TEST))


all test: $(ENABLED_LUA_TEST)

$(ALL_LUA_TEST) : test_% : %.lua
	@echo "Running test : $@"
	$(LUAJIT) $(LUA_FLAGS) $<

.PHONY: $(ALL_LUA_TEST)
