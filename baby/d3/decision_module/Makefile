# 1. 컴파일러랑 옵션 설정
CC = gcc
CFLAGS = -Wall -Wextra -g
# 헤더 파일(.h)이 있는 폴더 위치를 알려줌 (-I 옵션)
INCLUDES = -I./decide_mode -I./ArUco_RX

# 2. 실행 파일 이름 (니 맘대로 바꿔)
TARGET = run_test

# 3. 소스 파일 목록 (경로까지 정확히 적어야 함!)
SRCS = decision_module_V1.c \
		decide_mode/decide_mode.c \
		ArUco_RX/ArUco_RX.c

# 4. 소스 파일(.c)을 목적 파일(.o)로 변환한다고 정의
OBJS = $(SRCS:.c=.o)

# 5. 기본 타겟 (make 치면 실행되는 놈)
all: $(TARGET)

# 6. 실행 파일 만드는 규칙 (링킹)
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $(TARGET) $(OBJS)

# 7. 개별 파일 컴파일 규칙 (.c -> .o)
# $< : 현재 처리 중인 .c 파일
# $@ : 현재 만들고 있는 .o 파일
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# 8. 청소부 (make clean 치면 싹 지워줌)
clean:
	rm -f $(OBJS) $(TARGET)