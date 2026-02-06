# 1. 변수 설정
CC = gcc
CFLAGS = -Wall
TARGET_NAME = debug_app
BUILD_DIR = build

# 실행 파일 경로
TARGET = $(BUILD_DIR)/$(TARGET_NAME)

# 소스 및 오브젝트 파일
SRCS = debug.c vcp_debug.c
OBJS = $(BUILD_DIR)/debug.o $(BUILD_DIR)/vcp_debug.o

# 2. 기본 빌드 규칙 (make)
all: $(BUILD_DIR) $(TARGET)

# 3. 실행 파일 생성 (링킹)
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^

# 4. 오브젝트 파일 생성 (컴파일)
$(BUILD_DIR)/%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 5. build 디렉터리 생성
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 6. 정리 규칙 (make clean)
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean