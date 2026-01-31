#include "interrupt_example.h"

void My_ISR_Handler(void* args);

void hello_world(void)
{
    
    GPIO_Config(MY_GPIO, GPIO_FUNC(0) | GPIO_INPUT | GPIO_INPUTBUF_EN | GPIO_PULLUP);

    
    mcu_printf("i'm gpio_inTexSet %d\n",GPIO_IntExtSet(EIT, MY_GPIO));

    
    (void)GIC_IntVectSet(EIT, GIC_PRIORITY_NO_MEAN, GIC_INT_TYPE_EDGE_FALLING, (GICIsrFunc)&My_ISR_Handler, (void *)0);

    (void)GIC_IntSrcEn(EIT);

    mcu_printf("GIC_EXT0 Interrupt Setup Complete!\n");
    // while(1)
    // {
    //     if(GPIO_Get(MY_GPIO) == 1)
    //     {
    //         mcu_printf("one!\n");
    //     }   
    //     else{
    //         mcu_printf("zero!\n");
    //     }
    //     (void)SAL_TaskSleep(1000);
    // }
}

void My_ISR_Handler(void* args)
{
    GPIO_Set(MY_GPIO, 1);
    mcu_printf("hello i'm ISR %d\n", GPIO_Get(MY_GPIO));
}