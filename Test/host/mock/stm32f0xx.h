/**
 * @file    stm32f0xx.h (host test mock)
 * @brief   Minimal fake of the CMSIS device header: peripheral registers are
 *          plain RAM structures so that the firmware modules can run on a PC.
 */
#ifndef MOCK_STM32F0XX_H
#define MOCK_STM32F0XX_H

#include <stdint.h>

typedef struct
{
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR;
  volatile uint32_t AFR[2];
  volatile uint32_t BRR;
} GPIO_TypeDef;

typedef struct
{
  volatile uint32_t CR1, CR2, SMCR, DIER, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, RCR;
  volatile uint32_t CCR1, CCR2, CCR3, CCR4;
} TIM_TypeDef;

typedef struct { volatile uint32_t APB1ENR, APB2ENR, AHBENR; } RCC_TypeDef;
typedef struct { volatile uint32_t CFGR1; volatile uint32_t EXTICR[4]; } SYSCFG_TypeDef;
typedef struct { volatile uint32_t IMR, EMR, RTSR, FTSR, SWIER, PR; } EXTI_TypeDef;

extern GPIO_TypeDef mockGPIOA, mockGPIOB, mockGPIOF;
extern TIM_TypeDef mockTIM3;
extern RCC_TypeDef mockRCC;
extern SYSCFG_TypeDef mockSYSCFG;
extern EXTI_TypeDef mockEXTI;

#define GPIOA   (&mockGPIOA)
#define GPIOB   (&mockGPIOB)
#define GPIOF   (&mockGPIOF)
#define TIM3    (&mockTIM3)
#define RCC     (&mockRCC)
#define SYSCFG  (&mockSYSCFG)
#define EXTI    (&mockEXTI)

typedef enum { EXTI0_1_IRQn = 5, TIM3_IRQn = 16, SysTick_IRQn = -1 } IRQn_Type;

#define TIM_CR1_CEN              (1U << 0)
#define TIM_EGR_UG               (1U << 0)
#define TIM_SR_CC1IF             (1U << 1)
#define TIM_SR_CC2IF             (1U << 2)
#define TIM_DIER_CC1IE           (1U << 1)
#define TIM_DIER_CC2IE           (1U << 2)
#define RCC_APB1ENR_TIM3EN       (1U << 1)
#define SYSCFG_EXTICR1_EXTI0     (0xFU << 0)
#define SYSCFG_EXTICR1_EXTI0_PA  (0x0U)
#define SYSCFG_EXTICR1_EXTI1     (0xFU << 4)
#define SYSCFG_EXTICR1_EXTI1_PF  (0x5U << 4)
#define EXTI_IMR_MR0             (1U << 0)
#define EXTI_IMR_MR1             (1U << 1)
#define EXTI_RTSR_TR1            (1U << 1)
#define EXTI_FTSR_TR0            (1U << 0)
#define EXTI_FTSR_TR1            (1U << 1)
#define EXTI_PR_PR0              (1U << 0)
#define EXTI_PR_PR1              (1U << 1)

static inline void NVIC_SetPriority(IRQn_Type irq, uint32_t prio) { (void)irq; (void)prio; }
static inline void NVIC_EnableIRQ(IRQn_Type irq) { (void)irq; }
static inline void __NOP(void) { }

#endif
