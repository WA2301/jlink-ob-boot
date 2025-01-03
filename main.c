/**
  ******************************************************************************
  * @file    App/main.c 
  * @author  Windy Albert
  * @version V1.0.0
  * @date    27-January-2023
  * @brief   Main program body
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
// #include "main.h"

/* Public variables ----------------------------------------------------------*/
// uint32_t SysTime = 0;
// extern uint32_t Image$$INIT_TBL$$Base;
// extern uint32_t Image$$INIT_TBL$$Limit;
// extern uint32_t Image$$PROC_TBL$$Base;
// extern uint32_t Image$$PROC_TBL$$Limit;

// /**
//   * @brief  Main program
//   * @param  None
//   * @retval None
//   */
// int main(void)
// {
//     RCC_ClocksTypeDef RCC_Clocks;
//     init_fnc_t *init_fnc;
//     process_fnc_t *proc_fnc;
    
// 	__disable_irq();
    
//     /* SysTick end of count event each 1ms */
//     RCC_GetClocksFreq(&RCC_Clocks);
//     SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
    
//     for( init_fnc = (init_fnc_t*)&Image$$INIT_TBL$$Base;                \
//          init_fnc < (init_fnc_t*)&Image$$INIT_TBL$$Limit;               \
//          init_fnc ++ )
//     {
//         if( (*init_fnc)() )
//         {
//             while(1);
//         }
//     }
//     __enable_irq();
    
//     for(;;)
//     {
//         for( proc_fnc = (process_fnc_t*)&Image$$PROC_TBL$$Base;         \
//              proc_fnc < (process_fnc_t*)&Image$$PROC_TBL$$Limit;        \
//              proc_fnc ++ )
//         {
//             (*proc_fnc)();
//         }        
//     }
// }

// /**
//   * @brief  This function handles SysTick Handler.
//   * @param  None
//   * @retval None
//   */
// void SysTick_Handler(void)
// {
//     SysTime++;
// }












/* filename: stm32boot.c, bootloader for JLink ARM-OB STM32 */
/* To Test: burn ob-stm32-after-update.bin to 0x08000000, 
 * and then burn stm32boot.bin to 0x08000000.
 * Power up, LED0 and LED1 should light up and then off,
 * and LED0 on for a moment(during copy_firmware()), and LED0 off, LED1 on to
 * indicate finished. LED0 on PE6, LED1 on PE1. */
//#define DEBUG
const char *pId = "$Id: stm32boot.c 21 2009-10-14 12:38:12Z minux $";
typedef volatile unsigned int vu32;
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
#define GPIOE_CRL    (*((vu32*)(0x40011800)))
#define GPIOE_ODR    (*((vu32*)(0x4001180C)))
#define GPIOE_BSRR   (*((vu32*)(0x40011810)))
#define GPIOE_BRR    (*((vu32*)(0x40011814)))
#define RCC_APB2ENR  (*((vu32*)(0x40021018)))

#define FLASH_KEY1 0x45670123
#define FLASH_KEY2 0xcdef89ab

#define FLASH_ACR     (*((vu32*)(0x40022000)))
#define FLASH_KEYR    (*((vu32*)(0x40022004)))
#define FLASH_OPTKEYR (*((vu32*)(0x40022008)))  /* patch this to FLASH_KEYR */
#define FLASH_SR      (*((vu32*)(0x4002200c)))
#define FLASH_CR      (*((vu32*)(0x40022010)))
#define FLASH_AR      (*((vu32*)(0x40022014)))

#define SCB_BASE (0xe000ed00)
#define SCB_VTOR (*((vu32*)(SCB_BASE + 0x08)))

#define LED0 (1 << 6)
#define LED1 (1 << 1)
#ifdef DEBUG
# define LED_ON(led) (GPIOE_BRR = led)
# define LED_OFF(led) (GPIOE_BSRR = led)
# define LED_TOGGLE(led) (GPIOE_ODR ^= led)
#else
# define LED_ON(led)
# define LED_OFF(led)
# define LED_TOGGLE(led)
#endif

#ifndef STACK_SIZE
#define STACK_SIZE                   1024
#endif

void ResetISR(void);
void NMIException(void);
void HardFaultException(void);
void delay(void);
typedef void (* pfnISR)(void); // Pointer to exception handle function
// mthomas: added section -> alignment thru linker-script
__attribute__ ((section(".stackarea")))
static unsigned long pulStack[STACK_SIZE];


__attribute__ ((section(".isr_vector")))
pfnISR VectorTable[] = 
{
        (pfnISR)((unsigned long)pulStack + sizeof(pulStack)), // The initial stack pointer
        ResetISR,                                // The reset handler
        NMIException,
        HardFaultException
};

void delay(void)
{
        unsigned int i;
        for( i = 0; i < 0x3ffff; ++i)
                asm("nop");
}


// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
extern unsigned long _etext;
extern unsigned long _data;
extern unsigned long _edata;
extern unsigned long _bss;
extern unsigned long _ebss;
int main(void);
void ResetISR(void) {
        unsigned long *pulSrc, *pulDest;
        // Copy the data segment initializers from flash to SRAM.
        pulSrc = &_etext;
        for(pulDest = &_data; pulDest < &_edata; ) *pulDest++ = *pulSrc++;
        // Zero fill the bss segment.
        for(pulDest = &_bss; pulDest < &_ebss; ) *pulDest++ = 0;
        main();
}
void NMIException(void) { return; }
void HardFaultException(void) { return; } 
void init(void)
{
#ifndef DEBUG
        return;
#endif
        RCC_APB2ENR |= (1<<6); // enable GPIOE
        GPIOE_CRL = 0x03000030; // PE6, PE1 output push-pull
        //GPIOE_CRL |= 0x33330000;

        LED_ON(LED0);
        LED_ON(LED1);
        delay();
        LED_OFF(LED0);
        LED_OFF(LED1);
/*        int i;
        for (i = 0; i < 1; i++) {
                LED_TOGGLE(LED0);
                delay();
        }
        LED_OFF(LED0);
*/
}

// stm32 bootloader for ob-stm32
#define FIRMWARE_SIZE 0x5C00
#define FIRMWARE_BASE 0x4000
#define FROM (0x08000000+FIRMWARE_BASE+FIRMWARE_SIZE)
#define TO   (0x08000000+FIRMWARE_BASE)
#define PAGE_SIZE 1024

unsigned short CalcCrc(unsigned char *R4, int count) // huge thanks to DASM!
{
        unsigned R0 = 0, R3, R5, i;
        R3 = 0x8408;
        for (;count != 0; count--)
        {
                R5 = R0;
                R0 = *R4;
                R4++;
                R0 ^= R5;
                for (i = 8; i !=0; i--)
                {
                        int last = R0;
                        R0 >>= 1;
                        if (last & 1)
                                R0 ^= R3;

                }
        }
        return R0;
}

void flash_unlock(void) { // unlock the flash program erase controller
        FLASH_KEYR = FLASH_KEY1;
        FLASH_KEYR = FLASH_KEY2;
}

void flash_wait(void) {
        while (FLASH_SR & 0x1/*FLASH_FLAG_BSY*/)
                ;
        FLASH_SR = 0x35;
}

void flash_erase_page(uint32_t addr) {
        //LED_TOGGLE(LED1);
        FLASH_CR |= 0x02;
        FLASH_AR = addr;
        FLASH_CR |= 0x40;
        flash_wait();
        FLASH_CR &= ~0x02;
}

static inline int flash_write_halfword(uint32_t addr, uint16_t word) {
        flash_wait();
        FLASH_CR |= 0x01;
        *((volatile uint16_t *)addr) = word;
        flash_wait();
        FLASH_CR &= ~0x01;
        return *((volatile uint16_t *)addr) == word;
}

static inline int flash_write_word(uint32_t addr, uint32_t word) {
        flash_wait();
        FLASH_CR |= 0x01;
        asm volatile ("nop");
        *((volatile uint16_t *)addr) = word & 0xffff;
        flash_wait();
        *((volatile uint16_t *) (addr + 2)) = word >> 16;
        flash_wait();
        FLASH_CR &= ~0x01;
        return *((volatile uint32_t *)addr) == word;
}

void copy_firmware(void) {
        uint32_t addr, waddr, data;

        for (addr = TO; addr < TO + FIRMWARE_SIZE; addr += PAGE_SIZE) {
                int i;
                data = ~0;
                for (i = 0; i < PAGE_SIZE; i += 4) {
                        data &= *((uint32_t *)(addr + i));
                        if (data != (uint32_t)~0) { // need erase
                                //LED_ON(LED1); delay();
                                flash_erase_page(addr);
                                break;
                        }
                }
        }
        for (addr = FROM, waddr = TO; addr < FROM + FIRMWARE_SIZE; ) {
                //if (addr & 0x1FF == 0x100) LED_TOGGLE(LED1);
                data = *((uint32_t *)addr);
                if (data == (uint32_t)&FLASH_OPTKEYR) 
                        data = (uint32_t)&FLASH_KEYR; // patch firmware
                flash_write_word(waddr, data);
                addr += 4;
                waddr += 4;
        }
}

void mark_firmware_invalid(void) {
        flash_erase_page(FROM);
}

unsigned char IsValideFirmware(void) {
        unsigned int *i = (unsigned int *)FROM;
        int cnt = 0;
        unsigned int result = 0xFFFFFFFF;
        for (cnt = 0; cnt < 32; cnt++)
                result &= (*i++);
        if (result == 0xFFFFFFFF)
                return 0;
        unsigned short crc = CalcCrc((unsigned char *) FROM, FIRMWARE_SIZE - 2);
        if (*(unsigned short*)(FROM + FIRMWARE_SIZE- 2) != crc)
                return 0;
        return 1;
}

int main() {
        init(); // flash LED to indicate we started!

        if (IsValideFirmware())
        {
                flash_unlock();
                LED_ON(LED0);
                copy_firmware();
                LED_OFF(LED0);
                mark_firmware_invalid();
                LED_ON(LED1);
        } 
        FLASH_CR |= (1 << 7); // lock the FPEC

        // relocate vector table
        SCB_VTOR = TO;
        asm volatile ("ISB");
        asm volatile ("movw r3, #:lower16:134234112"); // FROM == 134234112
        asm volatile ("movt r3, #:upper16:134234112");
        asm volatile ("ldr r6, [r3, #0]");
        asm volatile ("msr msp, r6"); // load new stack pointer
        asm volatile ("ldr r6, [r3, #4]");
        asm volatile ("bx r6"); // direct branch to new reset routine
        return 0;
}















/******************* (C) COPYRIGHT 2025 Windy Albert ********** END OF FILE ***/
