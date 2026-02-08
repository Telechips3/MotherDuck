# 1. 변수 설정
CC = gcc
CFLAGS = -Wall -Wextra -O2
BUILD_DIR = build

# 소스 파일들
SRCS = debug.c vcp_debug.c

# 소스 및 오브젝트 파일
# SRCS = debug.c vcp_debug.c
# OBJS = $(BUILD_DIR)/debug.o $(BUILD_DIR)/vcp_debug.o

# 소스 파일의 이름에서 확장자를 제거하고 build/ 경로를 붙여 실행 파일 이름을 생성합니다.
# 예: debug.c -> build/debug, vcp_debug.c -> build/vcp_debug
TARGETS = $(patsubst %.c, $(BUILD_DIR)/%, $(SRCS))

# 2. 기본 빌드 규칙 (모든 타겟 생성)
all: $(BUILD_DIR) $(TARGETS)

# 3. 각 실행 파일 빌드 규칙
# % 패턴을 사용하여 각 .c 파일을 동일한 이름의 실행 파일로 컴파일합니다.
$(BUILD_DIR)/%: %.c | $(BUILD_DIR)
	$(CC) $(CFLAGS) $< -o $@

# 4. build 디렉터리 생성
$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# 5. 정리 규칙
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean