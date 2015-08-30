SRC_DIR = ./src
IGNORE_DIRS = ./src/third_party
BUILD_DIR = ./build

IGNORE_FILES = $(foreach d, $(IGNORE_DIRS), $(wildcard $d/*.js))

all: legs

clean:
	rm ./build/*

legs:
	browserify -r $(SRC_DIR)/main.js:$@ $(addprefix -u ,$(IGNORE_FILES)) > $(BUILD_DIR)/$@.js