CXXFLAGS = -std=c++20 -Wall -Wextra -O3 -Os -Wno-char-subscripts $(CXXFLAGS_EXTRA) -D_3DSSTREAM_HOST -I src -I ../common -I ../dep/ViGEmClient/include
#SANITIZE = true

ifdef SANITIZE
CXXFLAGS += -g -fsanitize=address -fno-omit-frame-pointer -fsanitize=undefined
endif

OUT = server.exe

../dep/ViGEmClient/src/ViGEmClient.o: CXXFLAGS_EXTRA = -DERROR_INVALID_DEVICE_OBJECT_PARAMETER=650L -w	#temp fix for out-of-date win32, do not show warnings on unrelated code
OUT_SRC = $(wildcard src/*.cpp) $(wildcard ../dep/ViGEmClient/src/*.cpp)

OUT_OBJ = $(OUT_SRC:.cpp=.o)

all: $(OUT)

$(OUT): $(OUT_OBJ)
	$(CXX) $(CXXFLAGS) $(OUT_OBJ) -o $(OUT) -lws2_32 -lSetupapi -lgdi32

clean:
	rm -f $(OUT_OBJ) $(OUT)

clean_all: clean
	rm -f $(TEST_STL_OBJ)

.PHONY: all clean test_all_obj
