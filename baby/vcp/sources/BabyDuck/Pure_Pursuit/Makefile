# 1. 컴파일러랑 옵션 설정
CC = gcc
CFLAGS = -Wall -Wextra -g
# 헤더 파일(.h)이 있는 폴더 위치를 알려줌 (-I 옵션)
INCLUDES = -I.
# ✅ [수정 1] 라이브러리 변수 정의 (여기서 -lm을 선언해야 함!)
LIBS = -lm

# 2. 실행 파일 이름
TARGET = PP_test

# 3. 소스 파일 목록
SRCS = PP_test.c Pure_Pursuit.c

# 4. 소스 파일(.c)을 목적 파일(.o)로 변환한다고 정의
OBJS = $(SRCS:.c=.o)

# 5. 기본 타겟
all: $(TARGET)

# 6. 실행 파일 만드는 규칙 (링킹)
$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) $(INCLUDES) -o $(TARGET) $(OBJS) $(LIBS)
# ✅ 위 줄 맨 끝에 $(LIBS)가 있으니까, 이제 -lm이 저기로 쏙 들어감!

# 7. 개별 파일 컴파일 규칙 (.c -> .o)
%.o: %.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@
# ✅ [수정 2] 여기 있던 -lm은 지웠어 (여기선 필요 없음)

# 8. 청소부
clean:
	rm -f $(OBJS) $(TARGET)