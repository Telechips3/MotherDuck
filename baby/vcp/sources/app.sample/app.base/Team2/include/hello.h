#ifndef _hello_H_  // 중복 포함 방지 시작
#define _hello_H_

#include "../team2_header.h"
#include "gpio_test.h"
#include "gic.h"

#define EIT (GIC_EXT4)
#define MY_GPIO (GPIO_GPB(2))

void hello_world(void);

#endif