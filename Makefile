CXXFLAGS = -std=c++20 -Wall -Wextra -O3 -Os -Wno-char-subscripts $(CXXFLAGS_EXTRA) -I src -I dep/include
#SANITIZE = true

ifdef SANITIZE
CXXFLAGS += -g -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined
endif

OUT = server.exe

OUT_SRC = $(wildcard src/*.cpp) $(wildcard dep/src/*.cpp)

OUT_OBJ = $(OUT_SRC:.cpp=.o)

all: $(OUT)

$(OUT): $(OUT_OBJ)
	$(CXX) $(CXXFLAGS) $(OUT_OBJ) -o $(OUT) -lws2_32 -Ldep/lib -lViGEmClient -lSetupapi -lgdi32

clean:
	rm -f $(OUT_OBJ) $(OUT)

clean_all: clean
	rm -f $(TEST_STL_OBJ)

.PHONY: all clean test_all_obj