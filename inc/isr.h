/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名       ：isr.h
 * 描述         ：重新宏定义中断号，重映射中断向量表里的中断函数地址，
 *                使其指向我们所定义的中断服务函数。声明中断服务函数
 *                警告：只能在"vectors.c"包含，而且必须在"vectors.h"包含的后面！！！
 *
 * 实验平台     ：野火kinetis开发板
 * 库版本       ：
 * 嵌入系统     ：
 *
 * 作者         ：野火嵌入式开发工作室
 * 淘宝店       ：http://firestm32.taobao.com
 * 技术支持论坛 ：http://www.ourdev.cn/bbs/bbs_list.jsp?bbs_id=1008
**********************************************************************************/



#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"

/*                          重新定义中断向量表
 *  先取消默认的中断向量元素宏定义       #undef  VECTOR_xxx
 *  在重新定义到自己编写的中断函数       #define VECTOR_xxx    xxx_IRQHandler
 *  例如：
 *       #undef  VECTOR_003
 *       #define VECTOR_003    HardFault_Handler    重新定义硬件上访中断服务函数
 *
 *       extren void  HardFault_Handler(void);      声明函数，然后在isr.c里定义
 */


#undef  VECTOR_104                        //取消中断号的定义
#define VECTOR_104    PORTB_IRQHandler    //PORTA中断

#undef  VECTOR_107                        //取消中断号的定义
#define VECTOR_107    PORTE_IRQHandler    //PORTE中断

#undef  VECTOR_084
#define VECTOR_084    PIT0_IRQHandler     //重新定义84号中断为PIT0_IRQHandler中断

#undef  VECTOR_085
#define VECTOR_085    PIT1_IRQHandler//key_IRQHandler     //重新定义84号中断为key_IRQHandler中断，PIT1

#undef  VECTOR_016
#define VECTOR_016   DMA0_IRQHandler


extern void PORTB_IRQHandler();           //PORTA中断服务函数
extern void PORTE_IRQHandler();           //PORTE中断服务函数
extern void PIT0_IRQHandler();            //PIT0 定时中断服务函数
extern void PIT1_IRQHandler();            //PIT1 定时中断服务函数
extern void DMA0_IRQHandler();


#endif  //__ISR_H

/* End of "isr.h" */
