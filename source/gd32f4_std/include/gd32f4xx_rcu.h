/*!
    \file    gd32f4xx_rcu.h
    \brief   definitions for the RCU
    
    \version 2024-01-15, V3.2.0, firmware for GD32F4xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#ifndef GD32F4XX_RCU_H
#define GD32F4XX_RCU_H

#include "gd32f4xx.h"

/* RCU definitions */
#define RCU                             RCU_BASE

/* registers definitions */
#define RCU_CTL                         STD_REG32(RCU + 0x00U)        /*!< control register */
#define RCU_PLL                         STD_REG32(RCU + 0x04U)        /*!< PLL register */
#define RCU_CFG0                        STD_REG32(RCU + 0x08U)        /*!< clock configuration register 0 */
#define RCU_INT                         STD_REG32(RCU + 0x0CU)        /*!< clock interrupt register */
#define RCU_AHB1RST                     STD_REG32(RCU + 0x10U)        /*!< AHB1 reset register */
#define RCU_AHB2RST                     STD_REG32(RCU + 0x14U)        /*!< AHB2 reset register */
#define RCU_AHB3RST                     STD_REG32(RCU + 0x18U)        /*!< AHB3 reset register */
#define RCU_APB1RST                     STD_REG32(RCU + 0x20U)        /*!< APB1 reset register */
#define RCU_APB2RST                     STD_REG32(RCU + 0x24U)        /*!< APB2 reset register */
#define RCU_AHB1EN                      STD_REG32(RCU + 0x30U)        /*!< AHB1 enable register */
#define RCU_AHB2EN                      STD_REG32(RCU + 0x34U)        /*!< AHB2 enable register */
#define RCU_AHB3EN                      STD_REG32(RCU + 0x38U)        /*!< AHB3 enable register */
#define RCU_APB1EN                      STD_REG32(RCU + 0x40U)        /*!< APB1 enable register */
#define RCU_APB2EN                      STD_REG32(RCU + 0x44U)        /*!< APB2 enable register */
#define RCU_AHB1SPEN                    STD_REG32(RCU + 0x50U)        /*!< AHB1 sleep mode enable register */
#define RCU_AHB2SPEN                    STD_REG32(RCU + 0x54U)        /*!< AHB2 sleep mode enable register */
#define RCU_AHB3SPEN                    STD_REG32(RCU + 0x58U)        /*!< AHB3 sleep mode enable register */ 
#define RCU_APB1SPEN                    STD_REG32(RCU + 0x60U)        /*!< APB1 sleep mode enable register */
#define RCU_APB2SPEN                    STD_REG32(RCU + 0x64U)        /*!< APB2 sleep mode enable register */
#define RCU_BDCTL                       STD_REG32(RCU + 0x70U)        /*!< backup domain control register */
#define RCU_RSTSCK                      STD_REG32(RCU + 0x74U)        /*!< reset source / clock register */
#define RCU_PLLSSCTL                    STD_REG32(RCU + 0x80U)        /*!< PLL clock spread spectrum control register */
#define RCU_PLLI2S                      STD_REG32(RCU + 0x84U)        /*!< PLLI2S register */ 
#define RCU_PLLSAI                      STD_REG32(RCU + 0x88U)        /*!< PLLSAI register */ 
#define RCU_CFG1                        STD_REG32(RCU + 0x8CU)        /*!< clock configuration register 1 */
#define RCU_ADDCTL                      STD_REG32(RCU + 0xC0U)        /*!< Additional clock control register */
#define RCU_ADDINT                      STD_REG32(RCU + 0xCCU)        /*!< Additional clock interrupt register */
#define RCU_ADDAPB1RST                  STD_REG32(RCU + 0xE0U)        /*!< APB1 additional reset register */
#define RCU_ADDAPB1EN                   STD_REG32(RCU + 0xE4U)        /*!< APB1 additional enable register */
#define RCU_ADDAPB1SPEN                 STD_REG32(RCU + 0xE8U)        /*!< APB1 additional sleep mode enable register */
#define RCU_VKEY                        STD_REG32(RCU + 0x100U)       /*!< voltage key register */
#define RCU_DSV                         STD_REG32(RCU + 0x134U)       /*!< deep-sleep mode voltage register */

/* bits definitions */
/* RCU_CTL */
#define RCU_CTL_IRC16MEN                STD_BIT(0)                    /*!< internal high speed oscillator enable */
#define RCU_CTL_IRC16MSTB               STD_BIT(1)                    /*!< IRC16M high speed internal oscillator stabilization flag */
#define RCU_CTL_IRC16MADJ               STD_BITS(3,7)                 /*!< high speed internal oscillator clock trim adjust value */
#define RCU_CTL_IRC16MCALIB             STD_BITS(8,15)                /*!< high speed internal oscillator calibration value register */
#define RCU_CTL_HXTALEN                 STD_BIT(16)                   /*!< external high speed oscillator enable */
#define RCU_CTL_HXTALSTB                STD_BIT(17)                   /*!< external crystal oscillator clock stabilization flag */
#define RCU_CTL_HXTALBPS                STD_BIT(18)                   /*!< external crystal oscillator clock bypass mode enable */
#define RCU_CTL_CKMEN                   STD_BIT(19)                   /*!< HXTAL clock monitor enable */
#define RCU_CTL_PLLEN                   STD_BIT(24)                   /*!< PLL enable */
#define RCU_CTL_PLLSTB                  STD_BIT(25)                   /*!< PLL Clock Stabilization Flag */
#define RCU_CTL_PLLI2SEN                STD_BIT(26)                   /*!< PLLI2S enable */
#define RCU_CTL_PLLI2SSTB               STD_BIT(27)                   /*!< PLLI2S Clock Stabilization Flag */
#define RCU_CTL_PLLSAIEN                STD_BIT(28)                   /*!< PLLSAI enable */
#define RCU_CTL_PLLSAISTB               STD_BIT(29)                   /*!< PLLSAI Clock Stabilization Flag */

/* RCU_PLL */
#define RCU_PLL_PLLPSC                  STD_BITS(0,5)                 /*!< The PLL VCO source clock prescaler */
#define RCU_PLL_PLLN                    STD_BITS(6,14)                /*!< The PLL VCO clock multi factor */
#define RCU_PLL_PLLP                    STD_BITS(16,17)               /*!< The PLLP output frequency division factor from PLL VCO clock */
#define RCU_PLL_PLLSEL                  STD_BIT(22)                   /*!< PLL Clock Source Selection */
#define RCU_PLL_PLLQ                    STD_BITS(24,27)               /*!< The PLL Q output frequency division factor from PLL VCO clock */

/* RCU_CFG0 */
#define RCU_CFG0_SCS                    STD_BITS(0,1)                 /*!< system clock switch */
#define RCU_CFG0_SCSS                   STD_BITS(2,3)                 /*!< system clock switch status */
#define RCU_CFG0_AHBPSC                 STD_BITS(4,7)                 /*!< AHB prescaler selection */
#define RCU_CFG0_APB1PSC                STD_BITS(10,12)               /*!< APB1 prescaler selection */
#define RCU_CFG0_APB2PSC                STD_BITS(13,15)               /*!< APB2 prescaler selection */
#define RCU_CFG0_RTCDIV                 STD_BITS(16,20)               /*!< RTC clock divider factor */
#define RCU_CFG0_CKOUT0SEL              STD_BITS(21,22)               /*!< CKOUT0 Clock Source Selection */
#define RCU_CFG0_I2SSEL                 STD_BIT(23)                   /*!< I2S Clock Source Selection */
#define RCU_CFG0_CKOUT0DIV              STD_BITS(24,26)               /*!< The CK_OUT0 divider which the CK_OUT0 frequency can be reduced */
#define RCU_CFG0_CKOUT1DIV              STD_BITS(27,29)               /*!< The CK_OUT1 divider which the CK_OUT1 frequency can be reduced */
#define RCU_CFG0_CKOUT1SEL              STD_BITS(30,31)               /*!< CKOUT1 Clock Source Selection */

/* RCU_INT */
#define RCU_INT_IRC32KSTBIF             STD_BIT(0)                    /*!< IRC32K stabilization interrupt flag */
#define RCU_INT_LXTALSTBIF              STD_BIT(1)                    /*!< LXTAL stabilization interrupt flag */
#define RCU_INT_IRC16MSTBIF             STD_BIT(2)                    /*!< IRC16M stabilization interrupt flag */
#define RCU_INT_HXTALSTBIF              STD_BIT(3)                    /*!< HXTAL stabilization interrupt flag */
#define RCU_INT_PLLSTBIF                STD_BIT(4)                    /*!< PLL stabilization interrupt flag */
#define RCU_INT_PLLI2SSTBIF             STD_BIT(5)                    /*!< PLLI2S stabilization interrupt flag */
#define RCU_INT_PLLSAISTBIF             STD_BIT(6)                    /*!< PLLSAI stabilization interrupt flag */
#define RCU_INT_CKMIF                   STD_BIT(7)                    /*!< HXTAL clock stuck interrupt flag */
#define RCU_INT_IRC32KSTBIE             STD_BIT(8)                    /*!< IRC32K stabilization interrupt enable */
#define RCU_INT_LXTALSTBIE              STD_BIT(9)                    /*!< LXTAL stabilization interrupt enable */
#define RCU_INT_IRC16MSTBIE             STD_BIT(10)                   /*!< IRC16M stabilization interrupt enable */
#define RCU_INT_HXTALSTBIE              STD_BIT(11)                   /*!< HXTAL stabilization interrupt enable */
#define RCU_INT_PLLSTBIE                STD_BIT(12)                   /*!< PLL stabilization interrupt enable */
#define RCU_INT_PLLI2SSTBIE             STD_BIT(13)                   /*!< PLLI2S Stabilization Interrupt Enable */
#define RCU_INT_PLLSAISTBIE             STD_BIT(14)                   /*!< PLLSAI Stabilization Interrupt Enable */
#define RCU_INT_IRC32KSTBIC             STD_BIT(16)                   /*!< IRC32K Stabilization Interrupt Clear */
#define RCU_INT_LXTALSTBIC              STD_BIT(17)                   /*!< LXTAL Stabilization Interrupt Clear */
#define RCU_INT_IRC16MSTBIC             STD_BIT(18)                   /*!< IRC16M Stabilization Interrupt Clear */
#define RCU_INT_HXTALSTBIC              STD_BIT(19)                   /*!< HXTAL Stabilization Interrupt Clear */
#define RCU_INT_PLLSTBIC                STD_BIT(20)                   /*!< PLL stabilization Interrupt Clear */
#define RCU_INT_PLLI2SSTBIC             STD_BIT(21)                   /*!< PLLI2S stabilization Interrupt Clear */
#define RCU_INT_PLLSAISTBIC             STD_BIT(22)                   /*!< PLLSAI stabilization Interrupt Clear */
#define RCU_INT_CKMIC                   STD_BIT(23)                   /*!< HXTAL Clock Stuck Interrupt Clear */

/* RCU_AHB1RST */
#define RCU_AHB1RST_PARST               STD_BIT(0)                    /*!< GPIO port A reset */
#define RCU_AHB1RST_PBRST               STD_BIT(1)                    /*!< GPIO port B reset */
#define RCU_AHB1RST_PCRST               STD_BIT(2)                    /*!< GPIO port C reset */
#define RCU_AHB1RST_PDRST               STD_BIT(3)                    /*!< GPIO port D reset */
#define RCU_AHB1RST_PERST               STD_BIT(4)                    /*!< GPIO port E reset */
#define RCU_AHB1RST_PFRST               STD_BIT(5)                    /*!< GPIO port F reset */
#define RCU_AHB1RST_PGRST               STD_BIT(6)                    /*!< GPIO port G reset */
#define RCU_AHB1RST_PHRST               STD_BIT(7)                    /*!< GPIO port H reset */
#define RCU_AHB1RST_PIRST               STD_BIT(8)                    /*!< GPIO port I reset */
#define RCU_AHB1RST_CRCRST              STD_BIT(12)                   /*!< CRC reset */ 
#define RCU_AHB1RST_DMA0RST             STD_BIT(21)                   /*!< DMA0 reset */
#define RCU_AHB1RST_DMA1RST             STD_BIT(22)                   /*!< DMA1 reset */
#define RCU_AHB1RST_IPARST              STD_BIT(23)                   /*!< IPA reset */
#define RCU_AHB1RST_ENETRST             STD_BIT(25)                   /*!< ENET reset */
#define RCU_AHB1RST_USBHSRST            STD_BIT(29)                   /*!< USBHS reset */

/* RCU_AHB2RST */
#define RCU_AHB2RST_DCIRST              STD_BIT(0)                    /*!< DCI reset */
#define RCU_AHB2RST_TRNGRST             STD_BIT(6)                    /*!< TRNG reset */
#define RCU_AHB2RST_USBFSRST            STD_BIT(7)                    /*!< USBFS reset */
                                    
/* RCU_AHB3RST */
#define RCU_AHB3RST_EXMCRST             STD_BIT(0)                    /*!< EXMC reset */

/* RCU_APB1RST */
#define RCU_APB1RST_TIMER1RST           STD_BIT(0)                    /*!< TIMER1 reset */
#define RCU_APB1RST_TIMER2RST           STD_BIT(1)                    /*!< TIMER2 reset */
#define RCU_APB1RST_TIMER3RST           STD_BIT(2)                    /*!< TIMER3 reset */
#define RCU_APB1RST_TIMER4RST           STD_BIT(3)                    /*!< TIMER4 reset */
#define RCU_APB1RST_TIMER5RST           STD_BIT(4)                    /*!< TIMER5 reset */
#define RCU_APB1RST_TIMER6RST           STD_BIT(5)                    /*!< TIMER6 reset */
#define RCU_APB1RST_TIMER11RST          STD_BIT(6)                    /*!< TIMER11 reset */
#define RCU_APB1RST_TIMER12RST          STD_BIT(7)                    /*!< TIMER12 reset */
#define RCU_APB1RST_TIMER13RST          STD_BIT(8)                    /*!< TIMER13 reset */
#define RCU_APB1RST_WWDGTRST            STD_BIT(11)                   /*!< WWDGT reset */
#define RCU_APB1RST_SPI1RST             STD_BIT(14)                   /*!< SPI1 reset */
#define RCU_APB1RST_SPI2RST             STD_BIT(15)                   /*!< SPI2 reset */
#define RCU_APB1RST_USART1RST           STD_BIT(17)                   /*!< USART1 reset */
#define RCU_APB1RST_USART2RST           STD_BIT(18)                   /*!< USART2 reset */
#define RCU_APB1RST_UART3RST            STD_BIT(19)                   /*!< UART3 reset */
#define RCU_APB1RST_UART4RST            STD_BIT(20)                   /*!< UART4 reset */
#define RCU_APB1RST_I2C0RST             STD_BIT(21)                   /*!< I2C0 reset */
#define RCU_APB1RST_I2C1RST             STD_BIT(22)                   /*!< I2C1 reset */
#define RCU_APB1RST_I2C2RST             STD_BIT(23)                   /*!< I2C2 reset */
#define RCU_APB1RST_CAN0RST             STD_BIT(25)                   /*!< CAN0 reset */
#define RCU_APB1RST_CAN1RST             STD_BIT(26)                   /*!< CAN1 reset */
#define RCU_APB1RST_PMURST              STD_BIT(28)                   /*!< PMU reset */
#define RCU_APB1RST_DACRST              STD_BIT(29)                   /*!< DAC reset */
#define RCU_APB1RST_UART6RST            STD_BIT(30)                   /*!< UART6 reset */
#define RCU_APB1RST_UART7RST            STD_BIT(31)                   /*!< UART7 reset */

/* RCU_APB2RST */
#define RCU_APB2RST_TIMER0RST           STD_BIT(0)                    /*!< TIMER0 reset */
#define RCU_APB2RST_TIMER7RST           STD_BIT(1)                    /*!< TIMER7 reset */
#define RCU_APB2RST_USART0RST           STD_BIT(4)                    /*!< USART0 reset */
#define RCU_APB2RST_USART5RST           STD_BIT(5)                    /*!< USART5 reset */
#define RCU_APB2RST_ADCRST              STD_BIT(8)                    /*!< ADC reset */
#define RCU_APB2RST_SDIORST             STD_BIT(11)                   /*!< SDIO reset */
#define RCU_APB2RST_SPI0RST             STD_BIT(12)                   /*!< SPI0 reset */
#define RCU_APB2RST_SPI3RST             STD_BIT(13)                   /*!< SPI3 reset */
#define RCU_APB2RST_SYSCFGRST           STD_BIT(14)                   /*!< SYSCFG reset */
#define RCU_APB2RST_TIMER8RST           STD_BIT(16)                   /*!< TIMER8 reset */
#define RCU_APB2RST_TIMER9RST           STD_BIT(17)                   /*!< TIMER9 reset */
#define RCU_APB2RST_TIMER10RST          STD_BIT(18)                   /*!< TIMER10 reset */
#define RCU_APB2RST_SPI4RST             STD_BIT(20)                   /*!< SPI4 reset */
#define RCU_APB2RST_SPI5RST             STD_BIT(21)                   /*!< SPI5 reset */
#define RCU_APB2RST_TLIRST              STD_BIT(26)                   /*!< TLI reset */

/* RCU_AHB1EN */
#define RCU_AHB1EN_PAEN                 STD_BIT(0)                    /*!< GPIO port A clock enable */
#define RCU_AHB1EN_PBEN                 STD_BIT(1)                    /*!< GPIO port B clock enable */
#define RCU_AHB1EN_PCEN                 STD_BIT(2)                    /*!< GPIO port C clock enable */
#define RCU_AHB1EN_PDEN                 STD_BIT(3)                    /*!< GPIO port D clock enable */
#define RCU_AHB1EN_PEEN                 STD_BIT(4)                    /*!< GPIO port E clock enable */
#define RCU_AHB1EN_PFEN                 STD_BIT(5)                    /*!< GPIO port F clock enable */
#define RCU_AHB1EN_PGEN                 STD_BIT(6)                    /*!< GPIO port G clock enable */
#define RCU_AHB1EN_PHEN                 STD_BIT(7)                    /*!< GPIO port H clock enable */
#define RCU_AHB1EN_PIEN                 STD_BIT(8)                    /*!< GPIO port I clock enable */
#define RCU_AHB1EN_CRCEN                STD_BIT(12)                   /*!< CRC clock enable */
#define RCU_AHB1EN_BKPSRAMEN            STD_BIT(18)                   /*!< BKPSRAM clock enable */
#define RCU_AHB1EN_TCMSRAMEN            STD_BIT(20)                   /*!< TCMSRAM clock enable */
#define RCU_AHB1EN_DMA0EN               STD_BIT(21)                   /*!< DMA0 clock enable */
#define RCU_AHB1EN_DMA1EN               STD_BIT(22)                   /*!< DMA1 clock enable */
#define RCU_AHB1EN_IPAEN                STD_BIT(23)                   /*!< IPA clock enable */
#define RCU_AHB1EN_ENETEN               STD_BIT(25)                   /*!< ENET clock enable */
#define RCU_AHB1EN_ENETTXEN             STD_BIT(26)                   /*!< Ethernet TX clock enable */
#define RCU_AHB1EN_ENETRXEN             STD_BIT(27)                   /*!< Ethernet RX clock enable */
#define RCU_AHB1EN_ENETPTPEN            STD_BIT(28)                   /*!< Ethernet PTP clock enable */
#define RCU_AHB1EN_USBHSEN              STD_BIT(29)                   /*!< USBHS clock enable */
#define RCU_AHB1EN_USBHSULPIEN          STD_BIT(30)                   /*!< USBHS ULPI clock enable */

/* RCU_AHB2EN */
#define RCU_AHB2EN_DCIEN                STD_BIT(0)                    /*!< DCI clock enable */
#define RCU_AHB2EN_TRNGEN               STD_BIT(6)                    /*!< TRNG clock enable */
#define RCU_AHB2EN_USBFSEN              STD_BIT(7)                    /*!< USBFS clock enable */

/* RCU_AHB3EN */
#define RCU_AHB3EN_EXMCEN               STD_BIT(0)                    /*!< EXMC clock enable */

/* RCU_APB1EN */
#define RCU_APB1EN_TIMER1EN             STD_BIT(0)                    /*!< TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN             STD_BIT(1)                    /*!< TIMER2 clock enable */
#define RCU_APB1EN_TIMER3EN             STD_BIT(2)                    /*!< TIMER3 clock enable */
#define RCU_APB1EN_TIMER4EN             STD_BIT(3)                    /*!< TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN             STD_BIT(4)                    /*!< TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN             STD_BIT(5)                    /*!< TIMER6 clock enable */
#define RCU_APB1EN_TIMER11EN            STD_BIT(6)                    /*!< TIMER11 clock enable */
#define RCU_APB1EN_TIMER12EN            STD_BIT(7)                    /*!< TIMER12 clock enable */
#define RCU_APB1EN_TIMER13EN            STD_BIT(8)                    /*!< TIMER13 clock enable */
#define RCU_APB1EN_WWDGTEN              STD_BIT(11)                   /*!< WWDGT clock enable */
#define RCU_APB1EN_SPI1EN               STD_BIT(14)                   /*!< SPI1 clock enable */
#define RCU_APB1EN_SPI2EN               STD_BIT(15)                   /*!< SPI2 clock enable */
#define RCU_APB1EN_USART1EN             STD_BIT(17)                   /*!< USART1 clock enable */
#define RCU_APB1EN_USART2EN             STD_BIT(18)                   /*!< USART2 clock enable */
#define RCU_APB1EN_UART3EN              STD_BIT(19)                   /*!< UART3 clock enable */
#define RCU_APB1EN_UART4EN              STD_BIT(20)                   /*!< UART4 clock enable */
#define RCU_APB1EN_I2C0EN               STD_BIT(21)                   /*!< I2C0 clock enable */
#define RCU_APB1EN_I2C1EN               STD_BIT(22)                   /*!< I2C1 clock enable */
#define RCU_APB1EN_I2C2EN               STD_BIT(23)                   /*!< I2C2 clock enable */
#define RCU_APB1EN_CAN0EN               STD_BIT(25)                   /*!< CAN0 clock enable */
#define RCU_APB1EN_CAN1EN               STD_BIT(26)                   /*!< CAN1 clock enable */
#define RCU_APB1EN_PMUEN                STD_BIT(28)                   /*!< PMU clock enable */
#define RCU_APB1EN_DACEN                STD_BIT(29)                   /*!< DAC clock enable */
#define RCU_APB1EN_UART6EN              STD_BIT(30)                   /*!< UART6 clock enable */
#define RCU_APB1EN_UART7EN              STD_BIT(31)                   /*!< UART7 clock enable */

/* RCU_APB2EN */
#define RCU_APB2EN_TIMER0EN             STD_BIT(0)                    /*!< TIMER0 clock enable */
#define RCU_APB2EN_TIMER7EN             STD_BIT(1)                    /*!< TIMER7 clock enable */
#define RCU_APB2EN_USART0EN             STD_BIT(4)                    /*!< USART0 clock enable */
#define RCU_APB2EN_USART5EN             STD_BIT(5)                    /*!< USART5 clock enable */
#define RCU_APB2EN_ADC0EN               STD_BIT(8)                    /*!< ADC0 clock enable */
#define RCU_APB2EN_ADC1EN               STD_BIT(9)                    /*!< ADC1 clock enable */
#define RCU_APB2EN_ADC2EN               STD_BIT(10)                   /*!< ADC2 clock enable */
#define RCU_APB2EN_SDIOEN               STD_BIT(11)                   /*!< SDIO clock enable */
#define RCU_APB2EN_SPI0EN               STD_BIT(12)                   /*!< SPI0 clock enable */
#define RCU_APB2EN_SPI3EN               STD_BIT(13)                   /*!< SPI3 clock enable */
#define RCU_APB2EN_SYSCFGEN             STD_BIT(14)                   /*!< SYSCFG clock enable */
#define RCU_APB2EN_TIMER8EN             STD_BIT(16)                   /*!< TIMER8 clock enable */
#define RCU_APB2EN_TIMER9EN             STD_BIT(17)                   /*!< TIMER9 clock enable */
#define RCU_APB2EN_TIMER10EN            STD_BIT(18)                   /*!< TIMER10 clock enable */
#define RCU_APB2EN_SPI4EN               STD_BIT(20)                   /*!< SPI4 clock enable */
#define RCU_APB2EN_SPI5EN               STD_BIT(21)                   /*!< SPI5 clock enable */
#define RCU_APB2EN_TLIEN                STD_BIT(26)                   /*!< TLI clock enable */

/* RCU_AHB1SPEN */
#define RCU_AHB1SPEN_PASPEN             STD_BIT(0)                    /*!< GPIO port A clock enable when sleep mode */
#define RCU_AHB1SPEN_PBSPEN             STD_BIT(1)                    /*!< GPIO port B clock enable when sleep mode */
#define RCU_AHB1SPEN_PCSPEN             STD_BIT(2)                    /*!< GPIO port C clock enable when sleep mode */
#define RCU_AHB1SPEN_PDSPEN             STD_BIT(3)                    /*!< GPIO port D clock enable when sleep mode */
#define RCU_AHB1SPEN_PESPEN             STD_BIT(4)                    /*!< GPIO port E clock enable when sleep mode */
#define RCU_AHB1SPEN_PFSPEN             STD_BIT(5)                    /*!< GPIO port F clock enable when sleep mode */
#define RCU_AHB1SPEN_PGSPEN             STD_BIT(6)                    /*!< GPIO port G clock enable when sleep mode */
#define RCU_AHB1SPEN_PHSPEN             STD_BIT(7)                    /*!< GPIO port H clock enable when sleep mode */
#define RCU_AHB1SPEN_PISPEN             STD_BIT(8)                    /*!< GPIO port I clock enable when sleep mode */
#define RCU_AHB1SPEN_CRCSPEN            STD_BIT(12)                   /*!< CRC clock enable when sleep mode */
#define RCU_AHB1SPEN_FMCSPEN            STD_BIT(15)                   /*!< FMC clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM0SPEN          STD_BIT(16)                   /*!< SRAM0 clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM1SPEN          STD_BIT(17)                   /*!< SRAM1 clock enable when sleep mode */
#define RCU_AHB1SPEN_BKPSRAMSPEN        STD_BIT(18)                   /*!< BKPSRAM clock enable when sleep mode */
#define RCU_AHB1SPEN_SRAM2SPEN          STD_BIT(19)                   /*!< SRAM2 clock enable when sleep mode */
#define RCU_AHB1SPEN_DMA0SPEN           STD_BIT(21)                   /*!< DMA0 clock when sleep mode enable */
#define RCU_AHB1SPEN_DMA1SPEN           STD_BIT(22)                   /*!< DMA1 clock when sleep mode enable */
#define RCU_AHB1SPEN_IPASPEN            STD_BIT(23)                   /*!< IPA clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETSPEN           STD_BIT(25)                   /*!< ENET clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETTXSPEN         STD_BIT(26)                   /*!< Ethernet TX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETRXSPEN         STD_BIT(27)                   /*!< Ethernet RX clock enable when sleep mode */
#define RCU_AHB1SPEN_ENETPTPSPEN        STD_BIT(28)                   /*!< Ethernet PTP clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHSSPEN          STD_BIT(29)                   /*!< USBHS clock enable when sleep mode */
#define RCU_AHB1SPEN_USBHSULPISPEN      STD_BIT(30)                   /*!< USBHS ULPI clock enable when sleep mode */

/* RCU_AHB2SPEN */
#define RCU_AHB2SPEN_DCISPEN            STD_BIT(0)                    /*!< DCI clock enable when sleep mode */
#define RCU_AHB2SPEN_TRNGSPEN           STD_BIT(6)                    /*!< TRNG clock enable when sleep mode */
#define RCU_AHB2SPEN_USBFSSPEN          STD_BIT(7)                    /*!< USBFS clock enable when sleep mode */

/* RCU_AHB3SPEN */
#define RCU_AHB3SPEN_EXMCSPEN           STD_BIT(0)                    /*!< EXMC clock enable when sleep mode */

/* RCU_APB1SPEN */
#define RCU_APB1SPEN_TIMER1SPEN         STD_BIT(0)                    /*!< TIMER1 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER2SPEN         STD_BIT(1)                    /*!< TIMER2 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER3SPEN         STD_BIT(2)                    /*!< TIMER3 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER4SPEN         STD_BIT(3)                    /*!< TIMER4 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER5SPEN         STD_BIT(4)                    /*!< TIMER5 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER6SPEN         STD_BIT(5)                    /*!< TIMER6 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER11SPEN        STD_BIT(6)                    /*!< TIMER11 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER12SPEN        STD_BIT(7)                    /*!< TIMER12 clock enable when sleep mode */
#define RCU_APB1SPEN_TIMER13SPEN        STD_BIT(8)                    /*!< TIMER13 clock enable when sleep mode */
#define RCU_APB1SPEN_WWDGTSPEN          STD_BIT(11)                   /*!< WWDGT clock enable when sleep mode */
#define RCU_APB1SPEN_SPI1SPEN           STD_BIT(14)                   /*!< SPI1 clock enable when sleep mode */
#define RCU_APB1SPEN_SPI2SPEN           STD_BIT(15)                   /*!< SPI2 clock enable when sleep mode */
#define RCU_APB1SPEN_USART1SPEN         STD_BIT(17)                   /*!< USART1 clock enable when sleep mode*/
#define RCU_APB1SPEN_USART2SPEN         STD_BIT(18)                   /*!< USART2 clock enable when sleep mode*/
#define RCU_APB1SPEN_UART3SPEN          STD_BIT(19)                   /*!< UART3 clock enable when sleep mode*/
#define RCU_APB1SPEN_UART4SPEN          STD_BIT(20)                   /*!< UART4 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C0SPEN           STD_BIT(21)                   /*!< I2C0 clock enable when sleep mode */
#define RCU_APB1SPEN_I2C1SPEN           STD_BIT(22)                   /*!< I2C1 clock enable when sleep mode*/
#define RCU_APB1SPEN_I2C2SPEN           STD_BIT(23)                   /*!< I2C2 clock enable when sleep mode */
#define RCU_APB1SPEN_CAN0SPEN           STD_BIT(25)                   /*!< CAN0 clock enable when sleep mode*/
#define RCU_APB1SPEN_CAN1SPEN           STD_BIT(26)                   /*!< CAN1 clock enable when sleep mode */
#define RCU_APB1SPEN_PMUSPEN            STD_BIT(28)                   /*!< PMU clock enable when sleep mode */
#define RCU_APB1SPEN_DACSPEN            STD_BIT(29)                   /*!< DAC clock enable when sleep mode */
#define RCU_APB1SPEN_UART6SPEN          STD_BIT(30)                   /*!< UART6 clock enable when sleep mode */
#define RCU_APB1SPEN_UART7SPEN          STD_BIT(31)                   /*!< UART7 clock enable when sleep mode */

/* RCU_APB2SPEN */
#define RCU_APB2SPEN_TIMER0SPEN         STD_BIT(0)                    /*!< TIMER0 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER7SPEN         STD_BIT(1)                    /*!< TIMER7 clock enable when sleep mode */
#define RCU_APB2SPEN_USART0SPEN         STD_BIT(4)                    /*!< USART0 clock enable when sleep mode */
#define RCU_APB2SPEN_USART5SPEN         STD_BIT(5)                    /*!< USART5 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC0SPEN           STD_BIT(8)                    /*!< ADC0 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC1SPEN           STD_BIT(9)                    /*!< ADC1 clock enable when sleep mode */
#define RCU_APB2SPEN_ADC2SPEN           STD_BIT(10)                   /*!< ADC2 clock enable when sleep mode */
#define RCU_APB2SPEN_SDIOSPEN           STD_BIT(11)                   /*!< SDIO clock enable when sleep mode */
#define RCU_APB2SPEN_SPI0SPEN           STD_BIT(12)                   /*!< SPI0 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI3SPEN           STD_BIT(13)                   /*!< SPI3 clock enable when sleep mode */
#define RCU_APB2SPEN_SYSCFGSPEN         STD_BIT(14)                   /*!< SYSCFG clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER8SPEN         STD_BIT(16)                   /*!< TIMER8 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER9SPEN         STD_BIT(17)                   /*!< TIMER9 clock enable when sleep mode */
#define RCU_APB2SPEN_TIMER10SPEN        STD_BIT(18)                   /*!< TIMER10 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI4SPEN           STD_BIT(20)                   /*!< SPI4 clock enable when sleep mode */
#define RCU_APB2SPEN_SPI5SPEN           STD_BIT(21)                   /*!< SPI5 clock enable when sleep mode */
#define RCU_APB2SPEN_TLISPEN            STD_BIT(26)                   /*!< TLI clock enable when sleep mode*/

/* RCU_BDCTL */
#define RCU_BDCTL_LXTALEN               STD_BIT(0)                    /*!< LXTAL enable */
#define RCU_BDCTL_LXTALSTB              STD_BIT(1)                    /*!< low speed crystal oscillator stabilization flag */
#define RCU_BDCTL_LXTALBPS              STD_BIT(2)                    /*!< LXTAL bypass mode enable */
#define RCU_BDCTL_LXTALDRI              STD_BIT(3)                    /*!< LXTAL drive capability */
#define RCU_BDCTL_RTCSRC                STD_BITS(8,9)                 /*!< RTC clock entry selection */
#define RCU_BDCTL_RTCEN                 STD_BIT(15)                   /*!< RTC clock enable */
#define RCU_BDCTL_BKPRST                STD_BIT(16)                   /*!< backup domain reset */

/* RCU_RSTSCK */
#define RCU_RSTSCK_IRC32KEN             STD_BIT(0)                    /*!< IRC32K enable */
#define RCU_RSTSCK_IRC32KSTB            STD_BIT(1)                    /*!< IRC32K stabilization flag */
#define RCU_RSTSCK_RSTFC                STD_BIT(24)                   /*!< reset flag clear */
#define RCU_RSTSCK_BORRSTF              STD_BIT(25)                   /*!< BOR reset flag */
#define RCU_RSTSCK_EPRSTF               STD_BIT(26)                   /*!< external pin reset flag */
#define RCU_RSTSCK_PORRSTF              STD_BIT(27)                   /*!< power reset flag */
#define RCU_RSTSCK_SWRSTF               STD_BIT(28)                   /*!< software reset flag */
#define RCU_RSTSCK_FWDGTRSTF            STD_BIT(29)                   /*!< free watchdog timer reset flag */
#define RCU_RSTSCK_WWDGTRSTF            STD_BIT(30)                   /*!< window watchdog timer reset flag */
#define RCU_RSTSCK_LPRSTF               STD_BIT(31)                   /*!< low-power reset flag */

/* RCU_PLLSSCTL */
#define RCU_PLLSSCTL_MODCNT             STD_BITS(0,12)                /*!< these bits configure PLL spread spectrum modulation 
                                                                       profile amplitude and frequency. the following criteria 
                                                                       must be met: MODSTEP*MODCNT=215-1 */
#define RCU_PLLSSCTL_MODSTEP            STD_BITS(13,27)               /*!< these bits configure PLL spread spectrum modulation 
                                                                       profile amplitude and frequency. the following criteria 
                                                                       must be met: MODSTEP*MODCNT=215-1 */
#define RCU_PLLSSCTL_SS_TYPE            STD_BIT(30)                   /*!< PLL spread spectrum modulation type select */
#define RCU_PLLSSCTL_SSCGON             STD_BIT(31)                   /*!< PLL spread spectrum modulation enable */

/* RCU_PLLI2S */
#define RCU_PLLI2S_PLLI2SN              STD_BITS(6,14)                /*!< the PLLI2S VCO clock multi factor */
#define RCU_PLLI2S_PLLI2SR              STD_BITS(28,30)               /*!< the PLLI2S R output frequency division factor from PLLI2S VCO clock */

/* RCU_PLLSAI */
#define RCU_PLLSAI_PLLSAIN              STD_BITS(6,14)                /*!< the PLLSAI VCO clock multi factor */
#define RCU_PLLSAI_PLLSAIP              STD_BITS(16,17)               /*!< the PLLSAI P output frequency division factor from PLLSAI VCO clock */
#define RCU_PLLSAI_PLLSAIR              STD_BITS(28,30)               /*!< the PLLSAI R output frequency division factor from PLLSAI VCO clock */

/* RCU_CFG1 */
#define RCU_CFG1_PLLSAIRDIV             STD_BITS(16,17)               /*!< the divider factor from PLLSAIR clock */
#define RCU_CFG1_TIMERSEL               STD_BIT(24)                   /*!< TIMER clock selection */

/* RCU_ADDCTL */
#define RCU_ADDCTL_CK48MSEL             STD_BIT(0)                    /*!< 48MHz clock selection */
#define RCU_ADDCTL_PLL48MSEL            STD_BIT(1)                    /*!< PLL48M clock selection */
#define RCU_ADDCTL_IRC48MEN             STD_BIT(16)                   /*!< internal 48MHz RC oscillator enable */
#define RCU_ADDCTL_IRC48MSTB            STD_BIT(17)                   /*!< internal 48MHz RC oscillator clock stabilization flag */
#define RCU_ADDCTL_IRC48MCAL            STD_BITS(24,31)               /*!< internal 48MHz RC oscillator calibration value register */

/* RCU_ADDINT */
#define RCU_ADDINT_IRC48MSTBIF          STD_BIT(6)                    /*!< IRC48M stabilization interrupt flag */
#define RCU_ADDINT_IRC48MSTBIE          STD_BIT(14)                   /*!< internal 48 MHz RC oscillator stabilization interrupt enable */
#define RCU_ADDINT_IRC48MSTBIC          STD_BIT(22)                   /*!< internal 48 MHz RC oscillator stabilization interrupt clear */

/* RCU_ADDAPB1RST */
#define RCU_ADDAPB1RST_CTCRST           STD_BIT(27)                   /*!< CTC reset */
#define RCU_ADDAPB1RST_IREFRST          STD_BIT(31)                   /*!< IREF reset */

/* RCU_ADDAPB1EN */
#define RCU_ADDAPB1EN_CTCEN             STD_BIT(27)                   /*!< CTC clock enable */
#define RCU_ADDAPB1EN_IREFEN            STD_BIT(31)                   /*!< IREF interface clock enable */

/* RCU_ADDAPB1SPEN */
#define RCU_ADDAPB1SPEN_CTCSPEN         STD_BIT(27)                   /*!< CTC clock enable during sleep mode */
#define RCU_ADDAPB1SPEN_IREFSPEN        STD_BIT(31)                   /*!< IREF interface clock enable during sleep mode */

/* RCU_VKEY */
#define RCU_VKEY_KEY                    STD_BITS(0,31)                /*!< RCU_DSV key register */

/* RCU_DSV */
#define RCU_DSV_DSLPVS                  STD_BITS(0,2)                 /*!< deep-sleep mode voltage select */

/* constants definitions */
/* define the peripheral clock enable bit position and its register index offset */
#define RCU_REGIDX_BIT(regidx, bitpos)      (((uint32_t)(regidx) << 6) | (uint32_t)(bitpos))
#define RCU_REG_VAL(periph)                 (STD_REG32(RCU + ((uint32_t)(periph) >> 6)))
#define RCU_BIT_POS(val)                    ((uint32_t)(val) & 0x1FU)
/* define the voltage key unlock value */
#define RCU_VKEY_UNLOCK                 ((uint32_t)0x1A2B3C4DU)

/* register offset */
/* peripherals enable */
#define AHB1EN_REG_OFFSET               0x30U                     /*!< AHB1 enable register offset */
#define AHB2EN_REG_OFFSET               0x34U                     /*!< AHB2 enable register offset */
#define AHB3EN_REG_OFFSET               0x38U                     /*!< AHB3 enable register offset */
#define APB1EN_REG_OFFSET               0x40U                     /*!< APB1 enable register offset */
#define APB2EN_REG_OFFSET               0x44U                     /*!< APB2 enable register offset */
#define AHB1SPEN_REG_OFFSET             0x50U                     /*!< AHB1 sleep mode enable register offset */
#define AHB2SPEN_REG_OFFSET             0x54U                     /*!< AHB2 sleep mode enable register offset */
#define AHB3SPEN_REG_OFFSET             0x58U                     /*!< AHB3 sleep mode enable register offset */
#define APB1SPEN_REG_OFFSET             0x60U                     /*!< APB1 sleep mode enable register offset */
#define APB2SPEN_REG_OFFSET             0x64U                     /*!< APB2 sleep mode enable register offset */
#define ADD_APB1EN_REG_OFFSET           0xE4U                     /*!< APB1 additional enable register offset */
#define ADD_APB1SPEN_REG_OFFSET         0xE8U                     /*!< APB1 additional sleep mode enable register offset */

/* peripherals reset */                                        
#define AHB1RST_REG_OFFSET              0x10U                     /*!< AHB1 reset register offset */
#define AHB2RST_REG_OFFSET              0x14U                     /*!< AHB2 reset register offset */
#define AHB3RST_REG_OFFSET              0x18U                     /*!< AHB3 reset register offset */
#define APB1RST_REG_OFFSET              0x20U                     /*!< APB1 reset register offset */
#define APB2RST_REG_OFFSET              0x24U                     /*!< APB2 reset register offset */
#define ADD_APB1RST_REG_OFFSET          0xE0U                     /*!< APB1 additional reset register offset */
#define RSTSCK_REG_OFFSET               0x74U                     /*!< reset source/clock register offset */

/* clock control */
#define CTL_REG_OFFSET                  0x00U                     /*!< control register offset */
#define BDCTL_REG_OFFSET                0x70U                     /*!< backup domain control register offset */
#define ADDCTL_REG_OFFSET               0xC0U                     /*!< additional clock control register offset */

/* clock stabilization and stuck interrupt */
#define INT_REG_OFFSET                  0x0CU                     /*!< clock interrupt register offset */
#define ADDINT_REG_OFFSET               0xCCU                     /*!< additional clock interrupt register offset */

/* configuration register */
#define PLL_REG_OFFSET                  0x04U                     /*!< PLL register offset */
#define CFG0_REG_OFFSET                 0x08U                     /*!< clock configuration register 0 offset */
#define PLLSSCTL_REG_OFFSET             0x80U                     /*!< PLL clock spread spectrum control register offset */
#define PLLI2S_REG_OFFSET               0x84U                     /*!< PLLI2S register offset */
#define PLLSAI_REG_OFFSET               0x88U                     /*!< PLLSAI register offset */
#define CFG1_REG_OFFSET                 0x8CU                     /*!< clock configuration register 1 offset */

/* peripheral clock enable */
typedef enum
{
    /* AHB1 peripherals */
    RCU_GPIOA     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 0U),                  /*!< GPIOA clock */
    RCU_GPIOB     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 1U),                  /*!< GPIOB clock */
    RCU_GPIOC     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 2U),                  /*!< GPIOC clock */
    RCU_GPIOD     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 3U),                  /*!< GPIOD clock */
    RCU_GPIOE     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 4U),                  /*!< GPIOE clock */
    RCU_GPIOF     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 5U),                  /*!< GPIOF clock */
    RCU_GPIOG     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 6U),                  /*!< GPIOG clock */
    RCU_GPIOH     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 7U),                  /*!< GPIOH clock */
    RCU_GPIOI     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 8U),                  /*!< GPIOI clock */
    RCU_CRC       = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 12U),                 /*!< CRC clock */
    RCU_BKPSRAM   = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 18U),                 /*!< BKPSRAM clock */
    RCU_TCMSRAM   = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 20U),                 /*!< TCMSRAM clock */
    RCU_DMA0      = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 21U),                 /*!< DMA0 clock */
    RCU_DMA1      = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 22U),                 /*!< DMA1 clock */
    RCU_IPA       = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 23U),                 /*!< IPA clock */
    RCU_ENET      = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 25U),                 /*!< ENET clock */
    RCU_ENETTX    = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 26U),                 /*!< ENETTX clock */
    RCU_ENETRX    = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 27U),                 /*!< ENETRX clock */
    RCU_ENETPTP   = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 28U),                 /*!< ENETPTP clock */
    RCU_USBHS     = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 29U),                 /*!< USBHS clock */
    RCU_USBHSULPI = RCU_REGIDX_BIT(AHB1EN_REG_OFFSET, 30U),                 /*!< USBHSULPI clock */
    /* AHB2 peripherals */
    RCU_DCI       = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 0U),                  /*!< DCI clock */
    RCU_TRNG      = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 6U),                  /*!< TRNG clock */
    RCU_USBFS     = RCU_REGIDX_BIT(AHB2EN_REG_OFFSET, 7U),                  /*!< USBFS clock */
    /* AHB3 peripherals */
    RCU_EXMC      = RCU_REGIDX_BIT(AHB3EN_REG_OFFSET, 0U),                  /*!< EXMC clock */
    /* APB1 peripherals */
    RCU_TIMER1    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 0U),                  /*!< TIMER1 clock */
    RCU_TIMER2    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 1U),                  /*!< TIMER2 clock */
    RCU_TIMER3    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 2U),                  /*!< TIMER3 clock */
    RCU_TIMER4    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 3U),                  /*!< TIMER4 clock */
    RCU_TIMER5    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 4U),                  /*!< TIMER5 clock */
    RCU_TIMER6    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 5U),                  /*!< TIMER6 clock */
    RCU_TIMER11   = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 6U),                  /*!< TIMER11 clock */
    RCU_TIMER12   = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 7U),                  /*!< TIMER12 clock */
    RCU_TIMER13   = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 8U),                  /*!< TIMER13 clock */   
    RCU_WWDGT     = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 11U),                 /*!< WWDGT clock */
    RCU_SPI1      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 14U),                 /*!< SPI1 clock */
    RCU_SPI2      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 15U),                 /*!< SPI2 clock */
    RCU_USART1    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 17U),                 /*!< USART1 clock */
    RCU_USART2    = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 18U),                 /*!< USART2 clock */
    RCU_UART3     = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 19U),                 /*!< UART3 clock */
    RCU_UART4     = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 20U),                 /*!< UART4 clock */
    RCU_I2C0      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 21U),                 /*!< I2C0 clock */
    RCU_I2C1      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 22U),                 /*!< I2C1 clock */
    RCU_I2C2      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 23U),                 /*!< I2C2 clock */   
    RCU_CAN0      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 25U),                 /*!< CAN0 clock */
    RCU_CAN1      = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 26U),                 /*!< CAN1 clock */
    RCU_PMU       = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 28U),                 /*!< PMU clock */
    RCU_DAC       = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 29U),                 /*!< DAC clock */
    RCU_UART6     = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 30U),                 /*!< UART6 clock */
    RCU_UART7     = RCU_REGIDX_BIT(APB1EN_REG_OFFSET, 31U),                 /*!< UART7 clock */
    RCU_RTC       = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 15U),                  /*!< RTC clock */
    /* APB2 peripherals */
    RCU_TIMER0    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 0U),                  /*!< TIMER0 clock */
    RCU_TIMER7    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 1U),                  /*!< TIMER7 clock */
    RCU_USART0    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 4U),                  /*!< USART0 clock */
    RCU_USART5    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 5U),                  /*!< USART5 clock */
    RCU_ADC0      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 8U),                  /*!< ADC0 clock */
    RCU_ADC1      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 9U),                  /*!< ADC1 clock */
    RCU_ADC2      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 10U),                 /*!< ADC2 clock */
    RCU_SDIO      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 11U),                 /*!< SDIO clock */
    RCU_SPI0      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 12U),                 /*!< SPI0 clock */
    RCU_SPI3      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 13U),                 /*!< SPI3 clock */
    RCU_SYSCFG    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 14U),                 /*!< SYSCFG clock */
    RCU_TIMER8    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 16U),                 /*!< TIMER8 clock */
    RCU_TIMER9    = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 17U),                 /*!< TIMER9 clock */
    RCU_TIMER10   = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 18U),                 /*!< TIMER10 clock */
    RCU_SPI4      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 20U),                 /*!< SPI4 clock */
    RCU_SPI5      = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 21U),                 /*!< SPI5 clock */
    RCU_TLI       = RCU_REGIDX_BIT(APB2EN_REG_OFFSET, 26U),                 /*!< TLI clock */
    /* APB1 additional peripherals */
    RCU_CTC       = RCU_REGIDX_BIT(ADD_APB1EN_REG_OFFSET, 27U),             /*!< CTC clock */
    RCU_IREF      = RCU_REGIDX_BIT(ADD_APB1EN_REG_OFFSET, 31U),             /*!< IREF clock */
}rcu_periph_enum;

/* peripheral clock enable when sleep mode*/
typedef enum
{
    /* AHB1 peripherals */
    RCU_GPIOA_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 0U),            /*!< GPIOA clock */
    RCU_GPIOB_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 1U),            /*!< GPIOB clock */
    RCU_GPIOC_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 2U),            /*!< GPIOC clock */
    RCU_GPIOD_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 3U),            /*!< GPIOD clock */
    RCU_GPIOE_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 4U),            /*!< GPIOE clock */
    RCU_GPIOF_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 5U),            /*!< GPIOF clock */
    RCU_GPIOG_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 6U),            /*!< GPIOG clock */
    RCU_GPIOH_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 7U),            /*!< GPIOH clock */
    RCU_GPIOI_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 8U),            /*!< GPIOI clock */
    RCU_CRC_SLP       = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 12U),           /*!< CRC clock */
    RCU_FMC_SLP       = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 15U),           /*!< FMC clock */
    RCU_SRAM0_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 16U),           /*!< SRAM0 clock */
    RCU_SRAM1_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 17U),           /*!< SRAM1 clock */
    RCU_BKPSRAM_SLP   = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 18U),           /*!< BKPSRAM clock */
    RCU_SRAM2_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 19U),           /*!< SRAM2 clock */
    RCU_DMA0_SLP      = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 21U),           /*!< DMA0 clock */
    RCU_DMA1_SLP      = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 22U),           /*!< DMA1 clock */
    RCU_IPA_SLP       = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 23U),           /*!< IPA clock */
    RCU_ENET_SLP      = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 25U),           /*!< ENET clock */
    RCU_ENETTX_SLP    = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 26U),           /*!< ENETTX clock */
    RCU_ENETRX_SLP    = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 27U),           /*!< ENETRX clock */
    RCU_ENETPTP_SLP   = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 28U),           /*!< ENETPTP clock */
    RCU_USBHS_SLP     = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 29U),           /*!< USBHS clock */
    RCU_USBHSULPI_SLP = RCU_REGIDX_BIT(AHB1SPEN_REG_OFFSET, 30U),           /*!< USBHSULPI clock */
    /* AHB2 peripherals */
    RCU_DCI_SLP       = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 0U),            /*!< DCI clock */
    RCU_TRNG_SLP      = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 6U),            /*!< TRNG clock */
    RCU_USBFS_SLP     = RCU_REGIDX_BIT(AHB2SPEN_REG_OFFSET, 7U),            /*!< USBFS clock */
    /* AHB3 peripherals */
    RCU_EXMC_SLP      = RCU_REGIDX_BIT(AHB3SPEN_REG_OFFSET, 0U),            /*!< EXMC clock */
    /* APB1 peripherals */
    RCU_TIMER1_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 0U),            /*!< TIMER1 clock */
    RCU_TIMER2_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 1U),            /*!< TIMER2 clock */
    RCU_TIMER3_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 2U),            /*!< TIMER3 clock */
    RCU_TIMER4_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 3U),            /*!< TIMER4 clock */
    RCU_TIMER5_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 4U),            /*!< TIMER5 clock */
    RCU_TIMER6_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 5U),            /*!< TIMER6 clock */
    RCU_TIMER11_SLP   = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 6U),            /*!< TIMER11 clock */
    RCU_TIMER12_SLP   = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 7U),            /*!< TIMER12 clock */
    RCU_TIMER13_SLP   = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 8U),            /*!< TIMER13 clock */   
    RCU_WWDGT_SLP     = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 11U),           /*!< WWDGT clock */
    RCU_SPI1_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 14U),           /*!< SPI1 clock */
    RCU_SPI2_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 15U),           /*!< SPI2 clock */
    RCU_USART1_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 17U),           /*!< USART1 clock */
    RCU_USART2_SLP    = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 18U),           /*!< USART2 clock */
    RCU_UART3_SLP     = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 19U),           /*!< UART3 clock */
    RCU_UART4_SLP     = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 20U),           /*!< UART4 clock */
    RCU_I2C0_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 21U),           /*!< I2C0 clock */
    RCU_I2C1_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 22U),           /*!< I2C1 clock */
    RCU_I2C2_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 23U),           /*!< I2C2 clock */   
    RCU_CAN0_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 25U),           /*!< CAN0 clock */
    RCU_CAN1_SLP      = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 26U),           /*!< CAN1 clock */
    RCU_PMU_SLP       = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 28U),           /*!< PMU clock */
    RCU_DAC_SLP       = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 29U),           /*!< DAC clock */
    RCU_UART6_SLP     = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 30U),           /*!< UART6 clock */
    RCU_UART7_SLP     = RCU_REGIDX_BIT(APB1SPEN_REG_OFFSET, 31U),           /*!< UART7 clock */
    /* APB2 peripherals */
    RCU_TIMER0_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 0U),            /*!< TIMER0 clock */
    RCU_TIMER7_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 1U),            /*!< TIMER7 clock */
    RCU_USART0_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 4U),            /*!< USART0 clock */
    RCU_USART5_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 5U),            /*!< USART5 clock */
    RCU_ADC0_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 8U),            /*!< ADC0 clock */
    RCU_ADC1_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 9U),            /*!< ADC1 clock */
    RCU_ADC2_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 10U),           /*!< ADC2 clock */
    RCU_SDIO_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 11U),           /*!< SDIO clock */
    RCU_SPI0_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 12U),           /*!< SPI0 clock */
    RCU_SPI3_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 13U),           /*!< SPI3 clock */
    RCU_SYSCFG_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 14U),           /*!< SYSCFG clock */
    RCU_TIMER8_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 16U),           /*!< TIMER8 clock */
    RCU_TIMER9_SLP    = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 17U),           /*!< TIMER9 clock */
    RCU_TIMER10_SLP   = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 18U),           /*!< TIMER10 clock */
    RCU_SPI4_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 20U),           /*!< SPI4 clock */
    RCU_SPI5_SLP      = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 21U),           /*!< SPI5 clock */
    RCU_TLI_SLP       = RCU_REGIDX_BIT(APB2SPEN_REG_OFFSET, 26U),           /*!< TLI clock */
    /* APB1 additional peripherals */
    RCU_CTC_SLP       = RCU_REGIDX_BIT(ADD_APB1SPEN_REG_OFFSET, 27U),       /*!< CTC clock */
    RCU_IREF_SLP      = RCU_REGIDX_BIT(ADD_APB1SPEN_REG_OFFSET, 31U),       /*!< IREF clock */
}rcu_periph_sleep_enum;

/* peripherals reset */
typedef enum
{
    /* AHB1 peripherals */
    RCU_GPIOARST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 0U),              /*!< GPIOA clock reset */
    RCU_GPIOBRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 1U),              /*!< GPIOB clock reset */
    RCU_GPIOCRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 2U),              /*!< GPIOC clock reset */
    RCU_GPIODRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 3U),              /*!< GPIOD clock reset */
    RCU_GPIOERST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 4U),              /*!< GPIOE clock reset */
    RCU_GPIOFRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 5U),              /*!< GPIOF clock reset */
    RCU_GPIOGRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 6U),              /*!< GPIOG clock reset */
    RCU_GPIOHRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 7U),              /*!< GPIOH clock reset */
    RCU_GPIOIRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 8U),              /*!< GPIOI clock reset */
    RCU_CRCRST       = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 12U),             /*!< CRC clock reset */
    RCU_DMA0RST      = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 21U),             /*!< DMA0 clock reset */
    RCU_DMA1RST      = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 22U),             /*!< DMA1 clock reset */
    RCU_IPARST       = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 23U),             /*!< IPA clock reset */
    RCU_ENETRST      = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 25U),             /*!< ENET clock reset */   
    RCU_USBHSRST     = RCU_REGIDX_BIT(AHB1RST_REG_OFFSET, 29U),             /*!< USBHS clock reset */
    /* AHB2 peripherals */
    RCU_DCIRST       = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 0U),              /*!< DCI clock reset */
    RCU_TRNGRST      = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 6U),              /*!< TRNG clock reset */
    RCU_USBFSRST     = RCU_REGIDX_BIT(AHB2RST_REG_OFFSET, 7U),              /*!< USBFS clock reset */
    /* AHB3 peripherals */
    RCU_EXMCRST      = RCU_REGIDX_BIT(AHB3RST_REG_OFFSET, 0U),              /*!< EXMC clock reset */
    /* APB1 peripherals */
    RCU_TIMER1RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 0U),              /*!< TIMER1 clock reset */
    RCU_TIMER2RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 1U),              /*!< TIMER2 clock reset */
    RCU_TIMER3RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 2U),              /*!< TIMER3 clock reset */
    RCU_TIMER4RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 3U),              /*!< TIMER4 clock reset */
    RCU_TIMER5RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 4U),              /*!< TIMER5 clock reset */
    RCU_TIMER6RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 5U),              /*!< TIMER6 clock reset */
    RCU_TIMER11RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 6U),              /*!< TIMER11 clock reset */
    RCU_TIMER12RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 7U),              /*!< TIMER12 clock reset */
    RCU_TIMER13RST   = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 8U),              /*!< TIMER13 clock reset */   
    RCU_WWDGTRST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 11U),             /*!< WWDGT clock reset */
    RCU_SPI1RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 14U),             /*!< SPI1 clock reset */
    RCU_SPI2RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 15U),             /*!< SPI2 clock reset */
    RCU_USART1RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 17U),             /*!< USART1 clock reset */
    RCU_USART2RST    = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 18U),             /*!< USART2 clock reset */
    RCU_UART3RST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 19U),             /*!< UART3 clock reset */
    RCU_UART4RST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 20U),             /*!< UART4 clock reset */
    RCU_I2C0RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 21U),             /*!< I2C0 clock reset */
    RCU_I2C1RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 22U),             /*!< I2C1 clock reset */
    RCU_I2C2RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 23U),             /*!< I2C2 clock reset */   
    RCU_CAN0RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 25U),             /*!< CAN0 clock reset */
    RCU_CAN1RST      = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 26U),             /*!< CAN1 clock reset */
    RCU_PMURST       = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 28U),             /*!< PMU clock reset */
    RCU_DACRST       = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 29U),             /*!< DAC clock reset */
    RCU_UART6RST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 30U),             /*!< UART6 clock reset */
    RCU_UART7RST     = RCU_REGIDX_BIT(APB1RST_REG_OFFSET, 31U),             /*!< UART7 clock reset */
    /* APB2 peripherals */
    RCU_TIMER0RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 0U),              /*!< TIMER0 clock reset */
    RCU_TIMER7RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 1U),              /*!< TIMER7 clock reset */
    RCU_USART0RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 4U),              /*!< USART0 clock reset */
    RCU_USART5RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 5U),              /*!< USART5 clock reset */
    RCU_ADCRST       = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 8U),              /*!< ADCs all clock reset */
    RCU_SDIORST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 11U),             /*!< SDIO clock reset */
    RCU_SPI0RST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 12U),             /*!< SPI0 clock reset */
    RCU_SPI3RST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 13U),             /*!< SPI3 clock reset */
    RCU_SYSCFGRST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 14U),             /*!< SYSCFG clock reset */
    RCU_TIMER8RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 16U),             /*!< TIMER8 clock reset */
    RCU_TIMER9RST    = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 17U),             /*!< TIMER9 clock reset */
    RCU_TIMER10RST   = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 18U),             /*!< TIMER10 clock reset */
    RCU_SPI4RST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 20U),             /*!< SPI4 clock reset */
    RCU_SPI5RST      = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 21U),             /*!< SPI5 clock reset */
    RCU_TLIRST       = RCU_REGIDX_BIT(APB2RST_REG_OFFSET, 26U),             /*!< TLI clock reset */
    /* APB1 additional peripherals */
    RCU_CTCRST       = RCU_REGIDX_BIT(ADD_APB1RST_REG_OFFSET, 27U),         /*!< CTC clock reset */
    RCU_IREFRST      = RCU_REGIDX_BIT(ADD_APB1RST_REG_OFFSET, 31U)          /*!< IREF clock reset */
}rcu_periph_reset_enum;

/* clock stabilization and peripheral reset flags */
typedef enum
{
    /* clock stabilization flags */
    RCU_FLAG_IRC16MSTB     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 1U),            /*!< IRC16M stabilization flags */
    RCU_FLAG_HXTALSTB      = RCU_REGIDX_BIT(CTL_REG_OFFSET, 17U),           /*!< HXTAL stabilization flags */
    RCU_FLAG_PLLSTB        = RCU_REGIDX_BIT(CTL_REG_OFFSET, 25U),           /*!< PLL stabilization flags */
    RCU_FLAG_PLLI2SSTB     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 27U),           /*!< PLLI2S stabilization flags */
    RCU_FLAG_PLLSAISTB     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 29U),           /*!< PLLSAI stabilization flags */
    RCU_FLAG_LXTALSTB      = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 1U),          /*!< LXTAL stabilization flags */
    RCU_FLAG_IRC32KSTB     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 1U),         /*!< IRC32K stabilization flags */
    RCU_FLAG_IRC48MSTB     = RCU_REGIDX_BIT(ADDCTL_REG_OFFSET, 17U),        /*!< IRC48M stabilization flags */
    /* reset source flags */
    RCU_FLAG_BORRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 25U),        /*!< BOR reset flags */
    RCU_FLAG_EPRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 26U),        /*!< External PIN reset flags */
    RCU_FLAG_PORRST        = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 27U),        /*!< power reset flags */
    RCU_FLAG_SWRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 28U),        /*!< Software reset flags */
    RCU_FLAG_FWDGTRST      = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 29U),        /*!< FWDGT reset flags */
    RCU_FLAG_WWDGTRST      = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 30U),        /*!< WWDGT reset flags */
    RCU_FLAG_LPRST         = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 31U),        /*!< Low-power reset flags */
}rcu_flag_enum;

/* clock stabilization and ckm interrupt flags */
typedef enum
{
    RCU_INT_FLAG_IRC32KSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 0U),            /*!< IRC32K stabilization interrupt flag */
    RCU_INT_FLAG_LXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 1U),            /*!< LXTAL stabilization interrupt flag */
    RCU_INT_FLAG_IRC16MSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 2U),            /*!< IRC16M stabilization interrupt flag */
    RCU_INT_FLAG_HXTALSTB  = RCU_REGIDX_BIT(INT_REG_OFFSET, 3U),            /*!< HXTAL stabilization interrupt flag */
    RCU_INT_FLAG_PLLSTB    = RCU_REGIDX_BIT(INT_REG_OFFSET, 4U),            /*!< PLL stabilization interrupt flag */
    RCU_INT_FLAG_PLLI2SSTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 5U),            /*!< PLLI2S stabilization interrupt flag */
    RCU_INT_FLAG_PLLSAISTB = RCU_REGIDX_BIT(INT_REG_OFFSET, 6U),            /*!< PLLSAI stabilization interrupt flag */
    RCU_INT_FLAG_CKM       = RCU_REGIDX_BIT(INT_REG_OFFSET, 7U),            /*!< HXTAL clock stuck interrupt flag */
    RCU_INT_FLAG_IRC48MSTB = RCU_REGIDX_BIT(ADDINT_REG_OFFSET, 6U),         /*!< IRC48M stabilization interrupt flag */
}rcu_int_flag_enum;

/* clock stabilization and stuck interrupt flags clear */
typedef enum
{
    RCU_INT_FLAG_IRC32KSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 16U),       /*!< IRC32K stabilization interrupt flags clear */
    RCU_INT_FLAG_LXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 17U),       /*!< LXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_IRC16MSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 18U),       /*!< IRC16M stabilization interrupt flags clear */
    RCU_INT_FLAG_HXTALSTB_CLR  = RCU_REGIDX_BIT(INT_REG_OFFSET, 19U),       /*!< HXTAL stabilization interrupt flags clear */
    RCU_INT_FLAG_PLLSTB_CLR    = RCU_REGIDX_BIT(INT_REG_OFFSET, 20U),       /*!< PLL stabilization interrupt flags clear */
    RCU_INT_FLAG_PLLI2SSTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 21U),       /*!< PLLI2S stabilization interrupt flags clear */
    RCU_INT_FLAG_PLLSAISTB_CLR = RCU_REGIDX_BIT(INT_REG_OFFSET, 22U),       /*!< PLLSAI stabilization interrupt flags clear */
    RCU_INT_FLAG_CKM_CLR       = RCU_REGIDX_BIT(INT_REG_OFFSET, 23U),       /*!< CKM interrupt flags clear */
    RCU_INT_FLAG_IRC48MSTB_CLR = RCU_REGIDX_BIT(ADDINT_REG_OFFSET, 22U),    /*!< internal 48 MHz RC oscillator stabilization interrupt clear */
}rcu_int_flag_clear_enum;

/* clock stabilization interrupt enable or disable */
typedef enum
{
    RCU_INT_IRC32KSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 8U),           /*!< IRC32K stabilization interrupt */
    RCU_INT_LXTALSTB        = RCU_REGIDX_BIT(INT_REG_OFFSET, 9U),           /*!< LXTAL stabilization interrupt */
    RCU_INT_IRC16MSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 10U),          /*!< IRC16M stabilization interrupt */
    RCU_INT_HXTALSTB        = RCU_REGIDX_BIT(INT_REG_OFFSET, 11U),          /*!< HXTAL stabilization interrupt */
    RCU_INT_PLLSTB          = RCU_REGIDX_BIT(INT_REG_OFFSET, 12U),          /*!< PLL stabilization interrupt */
    RCU_INT_PLLI2SSTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 13U),          /*!< PLLI2S stabilization interrupt */
    RCU_INT_PLLSAISTB       = RCU_REGIDX_BIT(INT_REG_OFFSET, 14U),          /*!< PLLSAI stabilization interrupt */
    RCU_INT_IRC48MSTB       = RCU_REGIDX_BIT(ADDINT_REG_OFFSET, 14U),       /*!< internal 48 MHz RC oscillator stabilization interrupt */
}rcu_int_enum;

/* oscillator types */
typedef enum
{
    RCU_HXTAL      = RCU_REGIDX_BIT(CTL_REG_OFFSET, 16U),                   /*!< HXTAL */
    RCU_LXTAL      = RCU_REGIDX_BIT(BDCTL_REG_OFFSET, 0U),                  /*!< LXTAL */
    RCU_IRC16M     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 0U),                    /*!< IRC16M */
    RCU_IRC48M     = RCU_REGIDX_BIT(ADDCTL_REG_OFFSET, 16U),                /*!< IRC48M */
    RCU_IRC32K     = RCU_REGIDX_BIT(RSTSCK_REG_OFFSET, 0U),                 /*!< IRC32K */
    RCU_PLL_CK     = RCU_REGIDX_BIT(CTL_REG_OFFSET, 24U),                   /*!< PLL */
    RCU_PLLI2S_CK  = RCU_REGIDX_BIT(CTL_REG_OFFSET, 26U),                   /*!< PLLI2S */
    RCU_PLLSAI_CK  = RCU_REGIDX_BIT(CTL_REG_OFFSET, 28U),                   /*!< PLLSAI */
}rcu_osci_type_enum;

/* rcu clock frequency */
typedef enum
{
    CK_SYS      = 0,                                                        /*!< system clock */
    CK_AHB,                                                                 /*!< AHB clock */
    CK_APB1,                                                                /*!< APB1 clock */
    CK_APB2,                                                                /*!< APB2 clock */
}rcu_clock_freq_enum;

/* RCU_CFG0 register bit define */
/* system clock source select */
#define CFG0_SCS(regval)                (STD_BITS(0,1) & ((uint32_t)(regval) << 0))
#define RCU_CKSYSSRC_IRC16M             CFG0_SCS(0)                        /*!< system clock source select IRC16M */
#define RCU_CKSYSSRC_HXTAL              CFG0_SCS(1)                        /*!< system clock source select HXTAL */
#define RCU_CKSYSSRC_PLLP               CFG0_SCS(2)                        /*!< system clock source select PLLP */

/* system clock source select status */
#define CFG0_SCSS(regval)               (STD_BITS(2,3) & ((uint32_t)(regval) << 2))
#define RCU_SCSS_IRC16M                 CFG0_SCSS(0)                       /*!< system clock source select IRC16M */
#define RCU_SCSS_HXTAL                  CFG0_SCSS(1)                       /*!< system clock source select HXTAL */
#define RCU_SCSS_PLLP                   CFG0_SCSS(2)                       /*!< system clock source select PLLP */

/* AHB prescaler selection */
#define CFG0_AHBPSC(regval)             (STD_BITS(4,7) & ((uint32_t)(regval) << 4))
#define RCU_AHB_CKSYS_DIV1              CFG0_AHBPSC(0)                     /*!< AHB prescaler select CK_SYS */
#define RCU_AHB_CKSYS_DIV2              CFG0_AHBPSC(8)                     /*!< AHB prescaler select CK_SYS/2 */
#define RCU_AHB_CKSYS_DIV4              CFG0_AHBPSC(9)                     /*!< AHB prescaler select CK_SYS/4 */
#define RCU_AHB_CKSYS_DIV8              CFG0_AHBPSC(10)                    /*!< AHB prescaler select CK_SYS/8 */
#define RCU_AHB_CKSYS_DIV16             CFG0_AHBPSC(11)                    /*!< AHB prescaler select CK_SYS/16 */
#define RCU_AHB_CKSYS_DIV64             CFG0_AHBPSC(12)                    /*!< AHB prescaler select CK_SYS/64 */
#define RCU_AHB_CKSYS_DIV128            CFG0_AHBPSC(13)                    /*!< AHB prescaler select CK_SYS/128 */
#define RCU_AHB_CKSYS_DIV256            CFG0_AHBPSC(14)                    /*!< AHB prescaler select CK_SYS/256 */
#define RCU_AHB_CKSYS_DIV512            CFG0_AHBPSC(15)                    /*!< AHB prescaler select CK_SYS/512 */

/* APB1 prescaler selection */
#define CFG0_APB1PSC(regval)            (STD_BITS(10,12) & ((uint32_t)(regval) << 10))
#define RCU_APB1_CKAHB_DIV1             CFG0_APB1PSC(0)                    /*!< APB1 prescaler select CK_AHB */
#define RCU_APB1_CKAHB_DIV2             CFG0_APB1PSC(4)                    /*!< APB1 prescaler select CK_AHB/2 */
#define RCU_APB1_CKAHB_DIV4             CFG0_APB1PSC(5)                    /*!< APB1 prescaler select CK_AHB/4 */
#define RCU_APB1_CKAHB_DIV8             CFG0_APB1PSC(6)                    /*!< APB1 prescaler select CK_AHB/8 */
#define RCU_APB1_CKAHB_DIV16            CFG0_APB1PSC(7)                    /*!< APB1 prescaler select CK_AHB/16 */

/* APB2 prescaler selection */
#define CFG0_APB2PSC(regval)            (STD_BITS(13,15) & ((uint32_t)(regval) << 13))
#define RCU_APB2_CKAHB_DIV1             CFG0_APB2PSC(0)                    /*!< APB2 prescaler select CK_AHB */
#define RCU_APB2_CKAHB_DIV2             CFG0_APB2PSC(4)                    /*!< APB2 prescaler select CK_AHB/2 */
#define RCU_APB2_CKAHB_DIV4             CFG0_APB2PSC(5)                    /*!< APB2 prescaler select CK_AHB/4 */
#define RCU_APB2_CKAHB_DIV8             CFG0_APB2PSC(6)                    /*!< APB2 prescaler select CK_AHB/8 */
#define RCU_APB2_CKAHB_DIV16            CFG0_APB2PSC(7)                    /*!< APB2 prescaler select CK_AHB/16 */

/* RTC clock divider factor from HXTAL clock */
#define CFG0_RTCDIV(regval)             (STD_BITS(16,20) & ((uint32_t)(regval) << 16))
#define RCU_RTC_HXTAL_NONE              CFG0_RTCDIV(0)                     /*!< no clock for RTC */
#define RCU_RTC_HXTAL_DIV2              CFG0_RTCDIV(2)                     /*!< RTCDIV clock select CK_HXTAL/2 */
#define RCU_RTC_HXTAL_DIV3              CFG0_RTCDIV(3)                     /*!< RTCDIV clock select CK_HXTAL/3 */
#define RCU_RTC_HXTAL_DIV4              CFG0_RTCDIV(4)                     /*!< RTCDIV clock select CK_HXTAL/4 */
#define RCU_RTC_HXTAL_DIV5              CFG0_RTCDIV(5)                     /*!< RTCDIV clock select CK_HXTAL/5 */
#define RCU_RTC_HXTAL_DIV6              CFG0_RTCDIV(6)                     /*!< RTCDIV clock select CK_HXTAL/6 */
#define RCU_RTC_HXTAL_DIV7              CFG0_RTCDIV(7)                     /*!< RTCDIV clock select CK_HXTAL/7 */
#define RCU_RTC_HXTAL_DIV8              CFG0_RTCDIV(8)                     /*!< RTCDIV clock select CK_HXTAL/8 */
#define RCU_RTC_HXTAL_DIV9              CFG0_RTCDIV(9)                     /*!< RTCDIV clock select CK_HXTAL/9 */
#define RCU_RTC_HXTAL_DIV10             CFG0_RTCDIV(10)                    /*!< RTCDIV clock select CK_HXTAL/10 */
#define RCU_RTC_HXTAL_DIV11             CFG0_RTCDIV(11)                    /*!< RTCDIV clock select CK_HXTAL/11 */
#define RCU_RTC_HXTAL_DIV12             CFG0_RTCDIV(12)                    /*!< RTCDIV clock select CK_HXTAL/12 */
#define RCU_RTC_HXTAL_DIV13             CFG0_RTCDIV(13)                    /*!< RTCDIV clock select CK_HXTAL/13 */
#define RCU_RTC_HXTAL_DIV14             CFG0_RTCDIV(14)                    /*!< RTCDIV clock select CK_HXTAL/14 */
#define RCU_RTC_HXTAL_DIV15             CFG0_RTCDIV(15)                    /*!< RTCDIV clock select CK_HXTAL/15 */
#define RCU_RTC_HXTAL_DIV16             CFG0_RTCDIV(16)                    /*!< RTCDIV clock select CK_HXTAL/16 */
#define RCU_RTC_HXTAL_DIV17             CFG0_RTCDIV(17)                    /*!< RTCDIV clock select CK_HXTAL/17 */
#define RCU_RTC_HXTAL_DIV18             CFG0_RTCDIV(18)                    /*!< RTCDIV clock select CK_HXTAL/18 */
#define RCU_RTC_HXTAL_DIV19             CFG0_RTCDIV(19)                    /*!< RTCDIV clock select CK_HXTAL/19 */
#define RCU_RTC_HXTAL_DIV20             CFG0_RTCDIV(20)                    /*!< RTCDIV clock select CK_HXTAL/20 */
#define RCU_RTC_HXTAL_DIV21             CFG0_RTCDIV(21)                    /*!< RTCDIV clock select CK_HXTAL/21 */
#define RCU_RTC_HXTAL_DIV22             CFG0_RTCDIV(22)                    /*!< RTCDIV clock select CK_HXTAL/22 */
#define RCU_RTC_HXTAL_DIV23             CFG0_RTCDIV(23)                    /*!< RTCDIV clock select CK_HXTAL/23 */
#define RCU_RTC_HXTAL_DIV24             CFG0_RTCDIV(24)                    /*!< RTCDIV clock select CK_HXTAL/24 */
#define RCU_RTC_HXTAL_DIV25             CFG0_RTCDIV(25)                    /*!< RTCDIV clock select CK_HXTAL/25 */
#define RCU_RTC_HXTAL_DIV26             CFG0_RTCDIV(26)                    /*!< RTCDIV clock select CK_HXTAL/26 */
#define RCU_RTC_HXTAL_DIV27             CFG0_RTCDIV(27)                    /*!< RTCDIV clock select CK_HXTAL/27 */
#define RCU_RTC_HXTAL_DIV28             CFG0_RTCDIV(28)                    /*!< RTCDIV clock select CK_HXTAL/28 */
#define RCU_RTC_HXTAL_DIV29             CFG0_RTCDIV(29)                    /*!< RTCDIV clock select CK_HXTAL/29 */
#define RCU_RTC_HXTAL_DIV30             CFG0_RTCDIV(30)                    /*!< RTCDIV clock select CK_HXTAL/30 */
#define RCU_RTC_HXTAL_DIV31             CFG0_RTCDIV(31)                    /*!< RTCDIV clock select CK_HXTAL/31 */

/* CKOUT0 Clock source selection */
#define CFG0_CKOUT0SEL(regval)          (STD_BITS(21,22) & ((uint32_t)(regval) << 21))
#define RCU_CKOUT0SRC_IRC16M            CFG0_CKOUT0SEL(0)                  /*!< internal 16M RC oscillator clock selected */
#define RCU_CKOUT0SRC_LXTAL             CFG0_CKOUT0SEL(1)                  /*!< low speed crystal oscillator clock (LXTAL) selected */
#define RCU_CKOUT0SRC_HXTAL             CFG0_CKOUT0SEL(2)                  /*!< high speed crystal oscillator clock (HXTAL) selected */
#define RCU_CKOUT0SRC_PLLP              CFG0_CKOUT0SEL(3)                  /*!< CK_PLLP clock selected */

/* I2S Clock source selection */
#define RCU_I2SSRC_PLLI2S               ((uint32_t)0x00000000U)             /*!< PLLI2S output clock selected as I2S source clock */
#define RCU_I2SSRC_I2S_CKIN             RCU_CFG0_I2SSEL                    /*!< external I2S_CKIN pin selected as I2S source clock */

/* The CK_OUT0 divider */
#define CFG0_CKOUT0DIV(regval)          (STD_BITS(24,26) & ((uint32_t)(regval) << 24))
#define RCU_CKOUT0_DIV1                 CFG0_CKOUT0DIV(0)                  /*!< CK_OUT0 is divided by 1 */
#define RCU_CKOUT0_DIV2                 CFG0_CKOUT0DIV(4)                  /*!< CK_OUT0 is divided by 2 */
#define RCU_CKOUT0_DIV3                 CFG0_CKOUT0DIV(5)                  /*!< CK_OUT0 is divided by 3 */
#define RCU_CKOUT0_DIV4                 CFG0_CKOUT0DIV(6)                  /*!< CK_OUT0 is divided by 4 */
#define RCU_CKOUT0_DIV5                 CFG0_CKOUT0DIV(7)                  /*!< CK_OUT0 is divided by 5 */

/* The CK_OUT1 divider */
#define CFG0_CKOUT1DIV(regval)          (STD_BITS(27,29) & ((uint32_t)(regval) << 27))
#define RCU_CKOUT1_DIV1                 CFG0_CKOUT1DIV(0)                  /*!< CK_OUT1 is divided by 1 */
#define RCU_CKOUT1_DIV2                 CFG0_CKOUT1DIV(4)                  /*!< CK_OUT1 is divided by 2 */
#define RCU_CKOUT1_DIV3                 CFG0_CKOUT1DIV(5)                  /*!< CK_OUT1 is divided by 3 */
#define RCU_CKOUT1_DIV4                 CFG0_CKOUT1DIV(6)                  /*!< CK_OUT1 is divided by 4 */
#define RCU_CKOUT1_DIV5                 CFG0_CKOUT1DIV(7)                  /*!< CK_OUT1 is divided by 5 */

/* CKOUT1 Clock source selection */
#define CFG0_CKOUT1SEL(regval)          (STD_BITS(30,31) & ((uint32_t)(regval) << 30))
#define RCU_CKOUT1SRC_SYSTEMCLOCK       CFG0_CKOUT1SEL(0)                  /*!< system clock selected */
#define RCU_CKOUT1SRC_PLLI2SR           CFG0_CKOUT1SEL(1)                  /*!< CK_PLLI2SR clock selected */
#define RCU_CKOUT1SRC_HXTAL             CFG0_CKOUT1SEL(2)                  /*!< high speed crystal oscillator clock (HXTAL) selected */
#define RCU_CKOUT1SRC_PLLP              CFG0_CKOUT1SEL(3)                  /*!< CK_PLLP clock selected */

/* RCU_CFG1 register bit define */
/* the divider factor from PLLI2SQ clock */
#define CFG1_PLLI2SQDIV(regval)         (STD_BITS(0,4) & ((uint32_t)(regval) << 0))
#define RCU_PLLI2SQ_DIV1                CFG1_PLLI2SQDIV(0)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/1 */
#define RCU_PLLI2SQ_DIV2                CFG1_PLLI2SQDIV(1)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/2 */
#define RCU_PLLI2SQ_DIV3                CFG1_PLLI2SQDIV(2)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/3 */
#define RCU_PLLI2SQ_DIV4                CFG1_PLLI2SQDIV(3)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/4 */
#define RCU_PLLI2SQ_DIV5                CFG1_PLLI2SQDIV(4)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/5 */
#define RCU_PLLI2SQ_DIV6                CFG1_PLLI2SQDIV(5)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/6 */
#define RCU_PLLI2SQ_DIV7                CFG1_PLLI2SQDIV(6)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/7 */
#define RCU_PLLI2SQ_DIV8                CFG1_PLLI2SQDIV(7)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/8 */
#define RCU_PLLI2SQ_DIV9                CFG1_PLLI2SQDIV(8)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/9 */
#define RCU_PLLI2SQ_DIV10               CFG1_PLLI2SQDIV(9)                 /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/10 */
#define RCU_PLLI2SQ_DIV11               CFG1_PLLI2SQDIV(10)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/11 */
#define RCU_PLLI2SQ_DIV12               CFG1_PLLI2SQDIV(11)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/12 */
#define RCU_PLLI2SQ_DIV13               CFG1_PLLI2SQDIV(12)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/13 */
#define RCU_PLLI2SQ_DIV14               CFG1_PLLI2SQDIV(13)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/14 */
#define RCU_PLLI2SQ_DIV15               CFG1_PLLI2SQDIV(14)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/15 */
#define RCU_PLLI2SQ_DIV16               CFG1_PLLI2SQDIV(15)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/16 */
#define RCU_PLLI2SQ_DIV17               CFG1_PLLI2SQDIV(16)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/17 */
#define RCU_PLLI2SQ_DIV18               CFG1_PLLI2SQDIV(17)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/18 */
#define RCU_PLLI2SQ_DIV19               CFG1_PLLI2SQDIV(18)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/19 */
#define RCU_PLLI2SQ_DIV20               CFG1_PLLI2SQDIV(19)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/20 */
#define RCU_PLLI2SQ_DIV21               CFG1_PLLI2SQDIV(20)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/21 */
#define RCU_PLLI2SQ_DIV22               CFG1_PLLI2SQDIV(21)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/22 */
#define RCU_PLLI2SQ_DIV23               CFG1_PLLI2SQDIV(22)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/23 */
#define RCU_PLLI2SQ_DIV24               CFG1_PLLI2SQDIV(23)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/24 */
#define RCU_PLLI2SQ_DIV25               CFG1_PLLI2SQDIV(24)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/25 */
#define RCU_PLLI2SQ_DIV26               CFG1_PLLI2SQDIV(25)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/26 */
#define RCU_PLLI2SQ_DIV27               CFG1_PLLI2SQDIV(26)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/27 */
#define RCU_PLLI2SQ_DIV28               CFG1_PLLI2SQDIV(27)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/28 */
#define RCU_PLLI2SQ_DIV29               CFG1_PLLI2SQDIV(28)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/29 */
#define RCU_PLLI2SQ_DIV30               CFG1_PLLI2SQDIV(29)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/30 */
#define RCU_PLLI2SQ_DIV31               CFG1_PLLI2SQDIV(30)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/31 */
#define RCU_PLLI2SQ_DIV32               CFG1_PLLI2SQDIV(31)                /*!< CK_PLLI2SQDIV clock select CK_PLLI2SQ/32 */

/* the divider factor from PLLSAIR clock */
#define CFG1_PLLSAIRDIV(regval)         (STD_BITS(16,17) & ((uint32_t)(regval) << 16))
#define RCU_PLLSAIR_DIV2                CFG1_PLLSAIRDIV(0)                 /*!< CK_PLLSAIRDIV clock select CK_PLLSAIR/2 */
#define RCU_PLLSAIR_DIV4                CFG1_PLLSAIRDIV(1)                 /*!< CK_PLLSAIRDIV clock select CK_PLLSAIR/4 */
#define RCU_PLLSAIR_DIV8                CFG1_PLLSAIRDIV(2)                 /*!< CK_PLLSAIRDIV clock select CK_PLLSAIR/8 */
#define RCU_PLLSAIR_DIV16               CFG1_PLLSAIRDIV(3)                 /*!< CK_PLLSAIRDIV clock select CK_PLLSAIR/16 */

/* TIMER clock selection */
#define RCU_TIMER_PSC_MUL2              ~RCU_CFG1_TIMERSEL                 /*!< if APB1PSC/APB2PSC in RCU_CFG0 register is 0b0xx(CK_APBx = CK_AHB) 
                                                                                or 0b100(CK_APBx = CK_AHB/2), the TIMER clock is equal to CK_AHB(CK_TIMERx = CK_AHB).
                                                                                or else, the TIMER clock is twice the corresponding APB clock (TIMER in APB1 domain: CK_TIMERx = 2 x CK_APB1; 
                                                                                TIMER in APB2 domain: CK_TIMERx = 2 x CK_APB2) */
#define RCU_TIMER_PSC_MUL4              RCU_CFG1_TIMERSEL                  /*!< if APB1PSC/APB2PSC in RCU_CFG0 register is 0b0xx(CK_APBx = CK_AHB), 
                                                                                0b100(CK_APBx = CK_AHB/2), or 0b101(CK_APBx = CK_AHB/4), the TIMER clock is equal to CK_AHB(CK_TIMERx = CK_AHB). 
                                                                                or else, the TIMER clock is four timers the corresponding APB clock (TIMER in APB1 domain: CK_TIMERx = 4 x CK_APB1;  
                                                                                TIMER in APB2 domain: CK_TIMERx = 4 x CK_APB2) */

/* RCU_PLLSSCTL register bit define */
/* PLL spread spectrum modulation type select */
#define RCU_SS_TYPE_CENTER              ((uint32_t)0x00000000U)            /*!< center type is selected */
#define RCU_SS_TYPE_DOWN                RCU_PLLSSCTL_SS_TYPE               /*!< down type is selected */

/* RCU_PLL register bit define */
/* The PLL VCO source clock prescaler */
#define RCU_PLLPSC_DIV_MIN              ((uint32_t)2U)                     /*!< PLLPSC_DIV min value */
#define RCU_PLLPSC_DIV_MAX              ((uint32_t)63U)                    /*!< PLLPSC_DIV max value */

/* The PLL VCO clock multi factor */
#define RCU_PLLN_MUL_MIN                ((uint32_t)64U)                    /*!< PLLN_MUL min value */
#define RCU_PLLN_MUL_MAX                ((uint32_t)500U)                   /*!< PLLN_MUL max value */
#define RCU_SS_MODULATION_CENTER_INC    ((uint32_t)5U)                     /*!< minimum factor of PLLN in center mode */
#define RCU_SS_MODULATION_DOWN_INC      ((uint32_t)7U)                     /*!< minimum factor of PLLN in down mode */

/* The PLLP output frequency division factor from PLL VCO clock */
#define RCU_PLLP_DIV_MIN                ((uint32_t)2U)                     /*!< PLLP_DIV min value */
#define RCU_PLLP_DIV_MAX                ((uint32_t)8U)                     /*!< PLLP_DIV max value */
                                         
/* PLL Clock Source Selection  */
#define RCU_PLLSRC_IRC16M               ((uint32_t)0x00000000U)            /*!< IRC16M clock selected as source clock of PLL, PLLSAI, PLLI2S */
#define RCU_PLLSRC_HXTAL                RCU_PLL_PLLSEL                     /*!< HXTAL clock selected as source clock of PLL, PLLSAI, PLLI2S */

/* The PLL Q output frequency division factor from PLL VCO clock */
#define RCU_PLLQ_DIV_MIN                ((uint32_t)2U)                     /*!< PLLQ_DIV min value */
#define RCU_PLLQ_DIV_MAX                ((uint32_t)15U)                    /*!< PLLQ_DIV max value */

#define CHECK_PLL_PSC_VALID(val)        (((val) >= RCU_PLLPSC_DIV_MIN)&&((val) <= RCU_PLLPSC_DIV_MAX))            
#define CHECK_PLL_N_VALID(val, inc)     (((val) >= (RCU_PLLN_MUL_MIN + (inc)))&&((val) <= RCU_PLLN_MUL_MAX))      
#define CHECK_PLL_P_VALID(val)          (((val) == 2U) || ((val) == 4U) || ((val) == 6U) || ((val) == 8U))         
#define CHECK_PLL_Q_VALID(val)          (((val) >= RCU_PLLQ_DIV_MIN)&&((val) <= RCU_PLLQ_DIV_MAX))                 

/* RCU_BDCTL register bit define */
/* LXTAL drive capability */
#define RCU_LXTALDRI_LOWER_DRIVE        ((uint32_t)0x00000000)             /*!< LXTAL drive capability is selected lower */
#define RCU_LXTALDRI_HIGHER_DRIVE       RCU_BDCTL_LXTALDRI                 /*!< LXTAL drive capability is selected higher */

/* RTC clock entry selection */
#define BDCTL_RTCSRC(regval)            (STD_BITS(8,9) & ((uint32_t)(regval) << 8))
#define RCU_RTCSRC_NONE                 BDCTL_RTCSRC(0)                    /*!< no clock selected */
#define RCU_RTCSRC_LXTAL                BDCTL_RTCSRC(1)                    /*!< RTC source clock select LXTAL  */
#define RCU_RTCSRC_IRC32K               BDCTL_RTCSRC(2)                    /*!< RTC source clock select IRC32K */
#define RCU_RTCSRC_HXTAL_DIV_RTCDIV     BDCTL_RTCSRC(3)                    /*!< RTC source clock select HXTAL/RTCDIV */

/* RCU_PLLI2S register bit define */
/* The PLLI2S VCO clock multi factor */
#define RCU_PLLI2SN_MUL_MIN             50U
#define RCU_PLLI2SN_MUL_MAX             500U

/* The PLLI2S Q output frequency division factor from PLLI2S VCO clock */
#define RCU_PLLI2SQ_DIV_MIN             2U
#define RCU_PLLI2SQ_DIV_MAX             15U

/* The PLLI2S R output frequency division factor from PLLI2S VCO clock */
#define RCU_PLLI2SR_DIV_MIN             2U
#define RCU_PLLI2SR_DIV_MAX             7U

/* RCU_PLLSAI register bit define */
/* The PLLSAI VCO clock multi factor */
#define RCU_PLLSAIN_MUL_MIN             50U
#define RCU_PLLSAIN_MUL_MAX             500U

/* The PLLSAI P output frequency division factor from PLLSAI VCO clock */
#define RCU_PLLSAIP_DIV_MIN             2U
#define RCU_PLLSAIP_DIV_MAX             8U

/* The PLLSAI Q output frequency division factor from PLLSAI VCO clock */
#define RCU_PLLSAIQ_DIV_MIN             2U
#define RCU_PLLSAIQ_DIV_MAX             15U

/* The PLLSAI R output frequency division factor from PLLSAI VCO clock */
#define RCU_PLLSAIR_DIV_MIN             2U
#define RCU_PLLSAIR_DIV_MAX             7U

#define CHECK_PLLI2S_PSC_VALID(val)     (((val) >= RCU_PLLI2SPSC_DIV_MIN)&&((val) <= RCU_PLLI2SPSC_DIV_MAX))
#define CHECK_PLLI2S_N_VALID(val)       (((val) >= RCU_PLLI2SN_MUL_MIN)&&((val) <= RCU_PLLI2SN_MUL_MAX))
#define CHECK_PLLI2S_Q_VALID(val)       (((val) >= RCU_PLLI2SQ_DIV_MIN)&&((val) <= RCU_PLLI2SQ_DIV_MAX))
#define CHECK_PLLI2S_R_VALID(val)       (((val) >= RCU_PLLI2SR_DIV_MIN)&&((val) <= RCU_PLLI2SR_DIV_MAX))

#define CHECK_PLLSAI_N_VALID(val)       (((val) >= (RCU_PLLSAIN_MUL_MIN))&&((val) <= RCU_PLLSAIN_MUL_MAX))
#define CHECK_PLLSAI_P_VALID(val)       (((val) == 2U) || ((val) == 4U) || ((val) == 6U) || ((val) == 8U))
#define CHECK_PLLSAI_Q_VALID(val)       (((val) >= RCU_PLLSAIQ_DIV_MIN)&&((val) <= RCU_PLLSAIQ_DIV_MAX))
#define CHECK_PLLSAI_R_VALID(val)       (((val) >= RCU_PLLSAIR_DIV_MIN)&&((val) <= RCU_PLLSAIR_DIV_MAX))

/* RCU_ADDCTL register bit define */
/* 48MHz clock selection */ 
#define RCU_CK48MSRC_PLL48M             ((uint32_t)0x00000000U)            /*!< CK48M source clock select PLL48M */
#define RCU_CK48MSRC_IRC48M             RCU_ADDCTL_CK48MSEL                /*!< CK48M source clock select IRC48M */

/* PLL48M clock selection */
#define RCU_PLL48MSRC_PLLQ              ((uint32_t)0x00000000U)            /*!< PLL48M source clock select PLLQ */
#define RCU_PLL48MSRC_PLLSAIP           RCU_ADDCTL_PLL48MSEL               /*!< PLL48M source clock select PLLSAIP */

/* Deep-sleep mode voltage */
#define DSV_DSLPVS(regval)              (STD_BITS(0,2) & ((uint32_t)(regval) << 0))
#define RCU_DEEPSLEEP_V_0               DSV_DSLPVS(0)                      /*!< core voltage is default value in deep-sleep mode */
#define RCU_DEEPSLEEP_V_1               DSV_DSLPVS(1)                      /*!< core voltage is (default value-0.1)V in deep-sleep mode(customers are not recommended to use it)*/
#define RCU_DEEPSLEEP_V_2               DSV_DSLPVS(2)                      /*!< core voltage is (default value-0.2)V in deep-sleep mode(customers are not recommended to use it)*/
#define RCU_DEEPSLEEP_V_3               DSV_DSLPVS(3)                      /*!< core voltage is (default value-0.3)V in deep-sleep mode(customers are not recommended to use it)*/


/* function declarations */
/* peripherals clock configure functions */
/* deinitialize the RCU */
void rcu_deinit(void);
/* enable the peripherals clock */
void rcu_periph_clock_enable(rcu_periph_enum periph);
/* disable the peripherals clock */
void rcu_periph_clock_disable(rcu_periph_enum periph);
/* enable the peripherals clock when sleep mode */
void rcu_periph_clock_sleep_enable(rcu_periph_sleep_enum periph);
/* disable the peripherals clock when sleep mode */
void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph);
/* reset the peripherals */
void rcu_periph_reset_enable(rcu_periph_reset_enum periph_reset);
/* disable reset the peripheral */
void rcu_periph_reset_disable(rcu_periph_reset_enum periph_reset);
/* reset the BKP */
void rcu_bkp_reset_enable(void);
/* disable the BKP reset */
void rcu_bkp_reset_disable(void);

/* system and peripherals clock source, system reset configure functions */
/* configure the system clock source */
void rcu_system_clock_source_config(uint32_t ck_sys);
/* get the system clock source */
uint32_t rcu_system_clock_source_get(void);
/* configure the AHB prescaler selection */
void rcu_ahb_clock_config(uint32_t ck_ahb);
/* configure the APB1 prescaler selection */
void rcu_apb1_clock_config(uint32_t ck_apb1);
/* configure the APB2 prescaler selection */
void rcu_apb2_clock_config(uint32_t ck_apb2);
/* configure the CK_OUT0 clock source and divider */
void rcu_ckout0_config(uint32_t ckout0_src, uint32_t ckout0_div);
/* configure the CK_OUT1 clock source and divider */
void rcu_ckout1_config(uint32_t ckout1_src, uint32_t ckout1_div);
/* configure the PLL clock source selection and PLL multiply factor */
ErrStatus rcu_pll_config(uint32_t pll_src, uint32_t pll_psc, uint32_t pll_n, uint32_t pll_p, uint32_t pll_q);
/* configure the PLLI2S clock */
ErrStatus rcu_plli2s_config(uint32_t plli2s_n, uint32_t plli2s_r);
/* configure the PLLSAI clock */
ErrStatus rcu_pllsai_config(uint32_t pllsai_n, uint32_t pllsai_p, uint32_t pllsai_r);
/* configure the RTC clock source selection */
void rcu_rtc_clock_config(uint32_t rtc_clock_source);
/* cconfigure the frequency division of RTC clock when HXTAL was selected as its clock source */
void rcu_rtc_div_config(uint32_t rtc_div);
/* configure the I2S clock source selection */
void rcu_i2s_clock_config(uint32_t i2s_clock_source);
/* configure the CK48M clock selection */
void rcu_ck48m_clock_config(uint32_t ck48m_clock_source);
/* configure the PLL48M clock selection */
void rcu_pll48m_clock_config(uint32_t pll48m_clock_source);
/* configure the TIMER clock prescaler selection */
void rcu_timer_clock_prescaler_config(uint32_t timer_clock_prescaler);       
/* configure the TLI clock division selection */
void rcu_tli_clock_div_config(uint32_t pllsai_r_div);

/* LXTAL, IRC8M, PLL and other oscillator configure functions */
/* configure the LXTAL drive capability */
void rcu_lxtal_drive_capability_config(uint32_t lxtal_dricap);
/* wait for oscillator stabilization flags is SET or oscillator startup is timeout */
ErrStatus rcu_osci_stab_wait(rcu_osci_type_enum osci);
/* turn on the oscillator */
void rcu_osci_on(rcu_osci_type_enum osci);
/* turn off the oscillator */
void rcu_osci_off(rcu_osci_type_enum osci);
/* enable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it */
void rcu_osci_bypass_mode_enable(rcu_osci_type_enum osci);
/* disable the oscillator bypass mode, HXTALEN or LXTALEN must be reset before it */
void rcu_osci_bypass_mode_disable(rcu_osci_type_enum osci);
/* set the IRC16M adjust value */
void rcu_irc16m_adjust_value_set(uint32_t irc16m_adjval);
/* configure the spread spectrum modulation for the main PLL clock */
void rcu_spread_spectrum_config(uint32_t spread_spectrum_type, uint32_t modstep, uint32_t modcnt);
/* enable the spread spectrum modulation for the main PLL clock */
void rcu_spread_spectrum_enable(void);
/* disable the spread spectrum modulation for the main PLL clock */
void rcu_spread_spectrum_disable(void);

/* clock monitor configure functions */
/* enable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_enable(void);
/* disable the HXTAL clock monitor */
void rcu_hxtal_clock_monitor_disable(void);

/* voltage configure and clock frequency get functions */
/* unlock the voltage key */
void rcu_voltage_key_unlock(void);
/* set the deep sleep mode voltage */
void rcu_deepsleep_voltage_set(uint32_t dsvol);
/* get the system clock, bus and peripheral clock frequency */
uint32_t rcu_clock_freq_get(rcu_clock_freq_enum clock);

/* flag & interrupt functions */
/* get the clock stabilization and periphral reset flags */
FlagStatus rcu_flag_get(rcu_flag_enum flag);
/* clear the reset flag */
void rcu_all_reset_flag_clear(void);
/* get the clock stabilization interrupt and ckm flags */
FlagStatus rcu_interrupt_flag_get(rcu_int_flag_enum int_flag);
/* clear the interrupt flags */
void rcu_interrupt_flag_clear(rcu_int_flag_clear_enum int_flag);
/* enable the stabilization interrupt */
void rcu_interrupt_enable(rcu_int_enum interrupt);
/* disable the stabilization interrupt */
void rcu_interrupt_disable(rcu_int_enum interrupt);

#endif /* GD32F4XX_RCU_H */
