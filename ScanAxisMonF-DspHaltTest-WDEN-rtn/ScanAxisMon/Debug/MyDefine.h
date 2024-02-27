

#ifndef MY_DEFINE_H
#define MY_DEFINE_H


#ifdef __cplusplus
extern "C" {
#endif

/* type define part --------------------------------------------------------------- */

#define 	ULONG       unsigned long
#define 	UINT		unsigned int
#define 	UCHAR		unsigned char
#define 	ExDOUBLE 	extern double
#define 	ExULONG		extern unsigned long
#define		ExLONG		extern long
#define     ExUINT      extern unsigned int
#define     ExINT       extern int
#define 	ExUCHAR		extern unsigned char
#define 	ExCHAR		extern char
#define 	VUINT		volatile unsigned int
#define 	VINT		volatile int
#define 	VUCHAR		volatile unsigned char
#define 	VCHAR		volatile char
#define 	VDOUBLE 	volatile double
#define     VEUINT	    volatile extern unsigned int
#define 	VEINT		volatile extern int
#define 	VEUCHAR 	volatile extern unsigned char
#define 	VECHAR		volatile extern char


#define   TARGET   1
//---------------------------------------------------------------------------
// User To Select Target Device:

#define   DSP28_F2812   TARGET
#define   DSP28_F2810   0

//---------------------------------------------------------------------------
// Common CPU Definitions:
//

extern cregister volatile unsigned int IFR;
extern cregister volatile unsigned int IER;

#define  EINT   asm(" clrc INTM")
#define  DINT   asm(" setc INTM")
#define  ERTM   asm(" clrc DBGM")
#define  DRTM   asm(" setc DBGM")
#define  EALLOW asm(" EALLOW")
#define  EDIS   asm(" EDIS")
#define  ESTOP0 asm(" ESTOP0")

#define M_INT1  0x0001
#define M_INT2  0x0002
#define M_INT3  0x0004
#define M_INT4  0x0008
#define M_INT5  0x0010
#define M_INT6  0x0020
#define M_INT7  0x0040
#define M_INT8  0x0080
#define M_INT9  0x0100
#define M_INT10 0x0200
#define M_INT11 0x0400
#define M_INT12 0x0800
#define M_INT13 0x1000
#define M_INT14 0x2000
#define M_DLOG  0x4000
#define M_RTOS  0x8000

#define BIT0    0x0001
#define BIT1    0x0002
#define BIT2    0x0004
#define BIT3    0x0008
#define BIT4    0x0010
#define BIT5    0x0020
#define BIT6    0x0040
#define BIT7    0x0080
#define BIT8    0x0100
#define BIT9    0x0200
#define BIT10   0x0400
#define BIT11   0x0800
#define BIT12   0x1000
#define BIT13   0x2000
#define BIT14   0x4000
#define BIT15   0x8000


// IMR register bit
#define INT6_MASK	BIT5
#define INT5_MASK	BIT4
#define INT4_MASK	BIT3
#define INT3_MASK	BIT2
#define INT2_MASK	BIT1
#define INT1_MASK	BIT0

/* Interrupt part ------------------------------------------------------------- */
#define	DI          	asm("   DINT    ")		//interrupt disable
#define	EI          	asm("   EINT    ")		//interrupt inable

#define	NOP         	asm("   NOP     ")
/* ---------------------------------------------------------------------------- */

/* Function part --------------------------------------------------------------- */

#define SET(port, bit)		(port) |= (bit)
#define CLEAR(port, bit)	(port) &= (~(bit))
#define TOGGLE(port, bit)	(port) ^= (bit)

/* PLL(Phase Locked Loop) part --------------------------------------------------------------- */

#define PLLx5_0		0x0A
#define PLLx4_5		0x09
#define PLLx4_0		0x08
#define PLLx3_5		0x07
#define PLLx3_0		0x06
#define PLLx2_5		0x05
#define PLLx2_0		0x04
#define PLLx1_5		0x03
#define PLLx1_0		0x02
#define PLLx0_5		0x01
#define PLL_ByPass	0x00

/* IO port part --------------------------------------------------------------- */

#define MCRA	GpioMuxRegs.GPAMUX.all
#define MCRB	GpioMuxRegs.GPBMUX.all
#define MCRC	GpioMuxRegs.GPCMUX.all
#define MCRD	GpioMuxRegs.GPDMUX.all
#define MCRE	GpioMuxRegs.GPEMUX.all
#define MCRF	GpioMuxRegs.GPFMUX.all
#define MCRG	GpioMuxRegs.GPGMUX.all
#define MCRH	GpioMuxRegs.GPHMUX.all

#define PADIR	GpioMuxRegs.GPADIR.all
#define PBDIR	GpioMuxRegs.GPBDIR.all
#define PCDIR	GpioMuxRegs.GPCDIR.all
#define PDDIR	GpioMuxRegs.GPDDIR.all
#define PEDIR	GpioMuxRegs.GPEDIR.all
#define PFDIR	GpioMuxRegs.GPFDIR.all
#define PGDIR	GpioMuxRegs.GPGDIR.all
#define PHDIR	GpioMuxRegs.GPHDIR.all

#define PADAT	GpioDataRegs.GPADAT.all
#define PBDAT	GpioDataRegs.GPBDAT.all
#define PCDAT	GpioDataRegs.GPCDAT.all
#define PDDAT	GpioDataRegs.GPDDAT.all
#define PEDAT	GpioDataRegs.GPEDAT.all
#define PFDAT	GpioDataRegs.GPFDAT.all
#define PGDAT	GpioDataRegs.GPGDAT.all
#define PHDAT	GpioDataRegs.GPHDAT.all

#define PAQUAL	GpioMuxRegs.GPAQUAL.all
#define PBQUAL	GpioMuxRegs.GPBQUAL.all
#define PCQUAL	GpioMuxRegs.GPCQUAL.all
#define PDQUAL	GpioMuxRegs.GPDQUAL.all
#define PEQUAL	GpioMuxRegs.GPEQUAL.all
#define PFQUAL	GpioMuxRegs.GPFQUAL.all
#define PGQUAL	GpioMuxRegs.GPGQUAL.all
#define PHQUAL	GpioMuxRegs.GPHQUAL.all

#define SET_PA_OUT(x)	(MCRA &= (~(x)));	PADIR |= (x)
#define SET_PB_OUT(x)	(MCRB &= (~(x)));	PBDIR |= (x)
#define SET_PC_OUT(x)	(MCRC &= (~(x)));	PCDIR |= (x)
#define SET_PD_OUT(x)	(MCRD &= (~(x)));	PDDIR |= (x)
#define SET_PE_OUT(x)	(MCRE &= (~(x)));	PEDIR |= (x)
#define SET_PF_OUT(x)	(MCRF &= (~(x)));	PFDIR |= (x)
#define SET_PG_OUT(x)	(MCRG &= (~(x)));	PGDIR |= (x)
#define SET_PH_OUT(x)	(MCRH &= (~(x)));	PHDIR |= (x)

#define SET_PA_IN(x)	(MCRA &= (~(x)));	PADIR &= (~(x))
#define SET_PB_IN(x)	(MCRB &= (~(x)));	PBDIR &= (~(x))
#define SET_PC_IN(x)	(MCRC &= (~(x)));	PCDIR &= (~(x))
#define SET_PD_IN(x)	(MCRD &= (~(x)));	PDDIR &= (~(x))
#define SET_PE_IN(x)	(MCRE &= (~(x)));	PEDIR &= (~(x))
#define SET_PF_IN(x)	(MCRF &= (~(x)));	PFDIR &= (~(x))
#define SET_PG_IN(x)	(MCRG &= (~(x)));	PGDIR &= (~(x))
#define SET_PH_IN(x)	(MCRH &= (~(x)));	PHDIR &= (~(x))

#define SET_PA_DAT(x)	PADAT |= (x)
#define SET_PB_DAT(x)	PBDAT |= (x)
#define SET_PC_DAT(x)	PCDAT |= (x)
#define SET_PD_DAT(x)	PDDAT |= (x)
#define SET_PE_DAT(x)	PEDAT |= (x)
#define SET_PF_DAT(x)	PFDAT |= (x)
#define SET_PG_DAT(x)	PGDAT |= (x)
#define SET_PH_DAT(x)	PHDAT |= (x)

#define SET_PA_QUAL(x)	PAQUAL |= (x)
#define SET_PB_QUAL(x)	PBQUAL |= (x)
#define SET_PC_QUAL(x)	PCQUAL |= (x)
#define SET_PD_QUAL(x)	PDQUAL |= (x)
#define SET_PE_QUAL(x)	PEQUAL |= (x)
#define SET_PF_QUAL(x)	PFQUAL |= (x)
#define SET_PG_QUAL(x)	PGQUAL |= (x)
#define SET_PH_QUAL(x)	PHQUAL |= (x)

#define PORTA		PADAT
#define PORTB		PBDAT
#define PORTC		PCDAT
#define PORTD		PDDAT
#define PORTE		PEDAT
#define PORTF		PFDAT
#define PORTG		PGDAT
#define PORTH		PHDAT

#define OUT_PA(x) 	(PADAT = ((x) & 0xFF))	//output x to portA
#define OUT_PB(x) 	(PBDAT = ((x) & 0xFF))	//output x to portB
#define OUT_PC(x) 	(PCDAT = ((x) & 0xFF))	//output x to portC
#define OUT_PD(x) 	(PDDAT = ((x) & 0xFF))	//output x to portD
#define OUT_PE(x) 	(PEDAT = ((x) & 0xFF))	//output x to portE
#define OUT_PF(x) 	(PFDAT = ((x) & 0xFF))	//output x to portD
#define OUT_PG(x) 	(PGDAT = ((x) & 0xFF))	//output x to portE
#define OUT_PH(x) 	(PHDAT = ((x) & 0xFF))	//output x to portE

#define IN_PA(x)		(PADAT&(x))			//input from portA
#define IN_PB(x)		(PBDAT&(x))			//input from portB
#define IN_PC(x)		(PCDAT&(x))			//input from portC
#define IN_PD(x)		(PDDAT&(x))			//input from portD
#define IN_PE(x)		(PEDAT&(x))			//input from portE


// QUALPRD = SYSCLKOUT / (2 * Val)
#define QUALxNONE		0x00
#define QUALx2			0x01
#define QUALx4			0x02
//		:				:
//		:				:
#define QUALx512		0xFF



// 사용자가 지정한 인터럽트 이외의 인터럽트가 발생할 경우를 대비한 함수.
// 사용하지 않는 인터럽트는 이 함수와 연결되어 있다.
//void bad_trip(void)
//{
//	while(1) ;
//}

//###########################################################################
/* 
	GPAMUX: GPIO_A function 0=IOP,1=FUN	I(0)/O(1)
	bit15	0:	C3TRIP,PA15		;IOP		1
	bit14	0:	C2TRIP,PA14		;IOP		1
	bit13	0:	C1TRIP,PA13		;IOP		1
	bit12	0:	TCLKINA,PA12	;IOP		1
	bit11	0:	TDIRA,PA11		;IOP		1
	bit10	0:	CAP3_QEPI1,PA10	;IOP		1
	bit9	0:	CAP2_QEP2,PA9	;IOP		0
	bit8	0:	CAP1_QEP1,PA8	;IOP		0
	bit7	0:	T2PWM_T2CMP,PA7	;IOP		1
	bit6	0:	T1PWM_T1CMP,PA6	;IOP		1
	bit5	0:	PWM6,PA5		;IOP		1	
	bit4	0:	PWM5,PA4		;IOP		1	
	bit3	0:	PWM4,PA3		;IOP		1	
	bit2	0:	PWM3,PA2		;IOP		1	
	bit1	0:	PWM2,PA1		;IOP		1	
	bit0	0:	PWM1,PA0		;IOP		1

	GPBMUX: GPIO_B function 0=IOP,1=FUN	I(0)/O(1)
	bit15	0:	C6TRIP,PB15		;IOP		1
	bit14	0:	C5TRIP,PB14		;IOP		1
	bit13	0:	C4TRIP,PB13		;IOP		1
	bit12	0:	TCLKINB,PB12	;IOP		1
	bit11	0:	TDIRB,PB11		;IOP		1
	bit10	0:	CAP6_QEPI2,PB10	;IOP		1
	bit9	0:	CAP5_QEP4,PB9	;IOP		1
	bit8	0:	CAP4_QEP3,PB8	;IOP		1
	bit7	0:	T4PWM_T4CMP,PB7	;IOP		1
	bit6	0:	T3PWM_T3CMP,PB6	;IOP		1
	bit5	0:	PWM12,PB5		;IOP		1	
	bit4	0:	PWM11,PB4		;IOP		1	
	bit3	0:	PWM10,PB3		;IOP		1	
	bit2	0:	PWM9,PB2		;IOP		1	
	bit1	0:	PWM8,PB1		;IOP		1	
	bit0	0:	PWM7,PB0		;IOP		1

	GPDMUX: GPIO_D function 0=IOP,1=FUN
	bit6	0:	T4CTRIP,PD6			;IOP
	bit5	1:	T3CTRIP_PDPINTB,PD5	;FUN
	bit4	0:	res
	bit3	0:	res
	bit2	0:	res
	bit1	0:	T2CTRIP,PD1			;IOP		
	bit0	1:	T1CTRIP_PDPINTA,PD0	;FUN
	//	6(PD6),5(PDPINTB),4,3,2,1(PD1),0(PDPINTA)
	//     1       0      1 1 1    1       0	= 0x5e


	GPEMUX: GPIO_E function 0=IOP,1=FUN		I(0)/O(1)
	bit2	0:	XNMI_XINT13,PE2		;IOP		1
	bit1	0:	XINT2_ADCSOC,PE1	;IOP		0	
	bit0	0:	XINT1_XBIO,PE0		;IOP		0

	GPFMUX: GPIO_F function 0=IOP,1=FUN		I(0)/O(1)
	bit14	0:	XF,PF14			;IOP			1(CPU_LED)
	bit13	0:	MDR,PF13		;IOP			1
	bit12	0:	MDX,PF12		;IOP			1	
	bit11	0:	MFSR,PF11		;IOP			1
	bit10	0:	MFSX,PF10		;IOP			1
	bit9	0:	MCLKR,PF9		;IOP			1
	bit8	0:	MCLKX,PF8		;IOP			1	
	bit7	1:	CANRX,PF7		;FUN			0
	bit6	1:	CANTX,PF6		;FUN			1
	bit5	1:	SCIRXDA,PF5		;FUN			0
	bit4	1:	SCITXDA,PF4		;FUN			1	
	bit3	0:	SPISTE,PF3		;IOP			1
	bit2	0:	SPICLK,PF2		;IOP			1
	bit1	0:	SPISOMI,PF1		;IOP			1
	bit0	0:	SPISIMO,PF0		;IOP			1

	GPGMUX: GPIO_G function 0=IOP,1=FUN
	bit5	1:	SCIRXDB,PG5		;FUN
	bit4	1:	SCITXDB,PG4		;FUN		
	//	5(RXDB),4(TXDB),3,2,1,0
	//     0       1    0 0 0 0	= 0x10

	DIR: 1=output,0=input
	0x00=No, 0x01=SYSCLK/2, 0x02=SYSCLK/4, 0x04=SYSCLK/510
*/
//###########################################################################

#define IMR                *((volatile int *)0x0004)    /*   Interrupt Mask Register             */
#define IFR                *((volatile int *)0x0006)    /*   Interrupt Flag Register             */

#define SCSR1              *((volatile int *)0x7018)    /*   System Control &  Status Reg. 1     */
#define SCSR2              *((volatile int *)0x7019)    /*   System Control &  Status Reg. 2     */
#define T1CNT              *((volatile int *)0x7401)    /*   GP Timer 1 counter register.        */
#define T1CMPR             *((volatile int *)0x7402)    /*   GP Timer 1 compare register.        */
#define T1PR               *((volatile int *)0x7403)    /*   GP Timer 1 period register.         */
#define T1CON              *((volatile int *)0x7404)    /*   GP Timer 1 control register.        */
#define T2CNT              *((volatile int *)0x7405)    /*   GP Timer 2 counter register.        */
#define T2CMPR             *((volatile int *)0x7406)    /*   GP Timer 2 compare register.        */
#define T2PR               *((volatile int *)0x7407)    /*   GP Timer 2 period register.         */
#define T2CON              *((volatile int *)0x7408)    /*   GP Timer 2 control register.        */
#define EVAIMRA            *((volatile int *)0x742C)    /*   Group A Interrupt Mask Register A   */
#define EVAIMRB            *((volatile int *)0x742D)    /*   Group B Interrupt Mask Register A   */
#define EVAIMRC            *((volatile int *)0x742E)    /*   Group C Interrupt Mask Register A   */
#define EVAIFRA            *((volatile int *)0x742F)    /*   Group A Interrupt Flag Register A   */
#define EVAIFRB            *((volatile int *)0x7430)    /*   Group B Interrupt Flag Register A   */
#define EVAIFRC            *((volatile int *)0x7431)    /*   Group C Interrupt Flag Register A   */



// SCSR1 register
#define	ILLADR			BIT0	// illegal address detect bit 
#define	EVA_CLKEN		BIT2	// EVA module clock enable control bit 
#define	EVB_CLKEN		BIT3	// EVB mudule clock enable control bit 
#define CAN_CLKEN      	BIT4	// CAN module clock enable control bit 
#define SPI_CLKEN		BIT5	// SPI module clock enable control bit 
#define SCI_CLKEN		BIT6	// SCI module clock enable control bit 
#define ADC_CLKEN		BIT7	// ADC module clock enable control bit 
#define CLK_PS0			BIT9	// PLL clock prescale select 
#define CLK_PS1			BIT10	// PLL clock prescale select 
#define CLK_PS2			BIT11	// PLL clock prescale select 
#define	LPM0 			BIT12	// low power mode select 
#define LPM1			BIT13
#define CLKSRC          BIT14      // CLKOUT pin source select 


/*-------------------------------------------------------------------------------------------------
 EVx modeul
-------------------------------------------------------------------------------------------------*/
// SCIRXST register bit
#define RX_ERROR	    BIT7		// receiver-error flag 
#define RXRDY 		    BIT6  		// RX buffer full flag

// T1CON register bit
#define T1_ENA		    BIT6		// timer1 enable [W]


// T2CON register bit
#define T2_ENA		    BIT6		// timer2 enable [W]


// COMCONA
#define CENABLE		BIT15
#define	CLD1		BIT14
#define	CLD0		BIT13
#define SVENABLE	BIT12
#define	FCOMPOE		BIT9


// GPTCONA register bit
#define TCOMPOE		BIT6

// EVAIMRA register bit
#define T1OFINT_ENA	BIT10
#define T1UFINT_ENA	BIT9
#define T1CINT_ENA	BIT8
#define T1PINT_ENA	BIT7

// EVAIMRB register bit
#define T2OFINT_ENA	BIT3
#define T2UFINT_ENA	BIT2
#define T2CINT_ENA	BIT1
#define T2PINT_ENA	BIT0

// EVBIMRA register bit
#define T3OFINT_ENA	BIT10
#define T3UFINT_ENA	BIT9
#define T3CINT_ENA	BIT8
#define T3PINT_ENA	BIT7

// EVBIMRB register bit
#define T4OFINT_ENA	BIT3
#define T4UFINT_ENA	BIT2
#define T4CINT_ENA	BIT1
#define T4PINT_ENA	BIT0


// EVAIFRA
#define	T1OFINT_FLAG	BIT10
#define T1UFINT_FLAG	BIT9
#define T1CINT_FLAG	    BIT8
#define T1PINT_FLAG	    BIT7


//EVAIFRB
#define	T2OFINT_FLAG	BIT3
#define T2UFINT_FLAG	BIT2
#define T2CINT_FLAG	    BIT1
#define T2PINT_FLAG	    BIT0


/* Module ENABLE part --------------------------------------------------------- */
#define ADC_ENABLE	SET(SCSR1,ADC_CLKEN)
#define SCI_ENABLE	SET(SCSR1,SCI_CLKEN)
#define SPI_ENABLE  SET(SCSR1,SPI_CLKEN) 
#define CAN_ENABLE	SET(SCSR1,CAN_CLKEN)
#define EVB_ENABLE	SET(SCSR1,EVB_CLKEN)
#define EVA_ENABLE	SET(SCSR1,EVA_CLKEN)



/* TIMER part ----------------------------------------------------------------- */
#define T1ENABLE	    SET(T1CON, T1_ENA)
#define T1DISABLE	    CLEAR(T1CON, T1_ENA)
#define IS_T1OFINT_FLAG	((EVAIFRA) & (T1OFINT_FLAG))
#define IS_T1PINT_FLAG	((EVAIFRA) & (T1PINT_FLAG))
#define IS_T1CINT_FLAG	((EVAIFRA) & (T1CINT_FLAG))

#define T2ENABLE	    SET(T2CON, T2_ENA)
#define T2DISABLE	    CLEAR(T2CON, T2_ENA)
#define IS_T2OFINT_FLAG	((EVAIFRB) & (T2OFINT_FLAG))
#define IS_T2PINT_FLAG	((EVAIFRB) & (T2PINT_FLAG))
#define IS_T2CINT_FLAG	((EVAIFRB) & (T2CINT_FLAG))


#define T3ENABLE	    SET(T3CON, T2_ENA)
#define T3DISABLE	    CLEAR(T3CON, T2_ENA)
#define IS_T3OFINT_FLAG	((EVBIFRA) & (T3OFINT_FLAG))
#define IS_T3PINT_FLAG	((EVBIFRA) & (T3PINT_FLAG))
#define IS_T3CINT_FLAG	((EVBIFRA) & (T3CINT_FLAG))

#define T4ENABLE	    SET(T4CON, T2_ENA)
#define T4DISABLE	    CLEAR(T4CON, T2_ENA)
#define IS_T4OFINT_FLAG	((EVBIFRB) & (T4OFINT_FLAG))
#define IS_T4PINT_FLAG	((EVBIFRB) & (T4PINT_FLAG))
#define IS_T4CINT_FLAG	((EVBIFRB) & (T4CINT_FLAG))



/* ADC part ----------------------------------------------------------------- */
#define AD_START		AdcRegs.ADCTRL2.bit.SOC_SEQ1=1
#define IS_AD_BUSY		AdcRegs.ADCST.bit.SEQ1_BSY
#define AD_INT_CLR		AdcRegs.ADCST.bit.INT_SEQ1_CLR=1
#define AD_END			AdcRegs.ADCTRL2.bit.RST_SEQ1=1;






//---------------------------------------------------------------------------
// For Portability, User Is Recommended To Use Following Data Type Size
// Definitions For 16-bit and 32-Bit Signed/Unsigned Integers:
//

#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int             int16;
typedef long            int32;
typedef unsigned int    Uint16;
typedef unsigned long   Uint32;
typedef float           float32;
typedef long double     float64;
#endif


//---------------------------------------------------------------------------
// Include All Peripheral Header Files:
//
#include "DSP281x_SysCtrl.h"            // System Control/Power Modes
#include "DSP281x_Gpio.h"               // General Purpose I/O Registers
#include "DSP281x_PieCtrl.h"            // PIE Control Registers
#include "DSP281x_PieVect.h"            // PIE Vector Table
#include "DSP281x_XIntrupt.h"           // External Interrupts
#include "DSP281x_Adc.h"   		        // ADC Registers
#include "DSP281x_CpuTimers.h"
#include "DSP281x_DefaultIsr.h"

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // end of MY_DEFINE_H definition

//===========================================================================
// No more.
//===========================================================================
