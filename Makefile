SRC_DIR = ./src
IGNORE_DIRS = ./src/third_party
BUILD_DIR = ./build
TEST_DIR = ./tests

IGNORE_FILES = $(foreach d, $(IGNORE_DIRS), $(wildcard $d/*.js))
TEST_FILES = $(foreach d, $(TEST_DIR), $(wildcard $d/*_tests.js))

all: legs

clean:
	rm ./build/*

legs:
	browserify -r $(SRC_DIR)/main.js:$@ $(addprefix -u ,$(IGNORE_FILES)) > $(BUILD_DIR)/$@.js

test: $(TEST_FILES)
	nodeunit $?