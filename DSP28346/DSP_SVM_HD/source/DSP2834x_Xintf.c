// TI File $Revision: /main/5 $
// Checkin $Date: August 28, 2008   16:54:21 $
//###########################################################################
//
// FILE:   DSP2834x_Xintf.c
//
// TITLE:   DSP2834x Device External Interface Init & Support Functions.
//
// DESCRIPTION:
//
//          Example initialization function for the external interface (XINTF).
//          This example configures the XINTF to its default state.  For an
//          example of how this function being used refer to the
//          examples/run_from_xintf project.
//
//###########################################################################
// $TI Release: DSP2834x C/C++ Header Files V1.10 $
// $Release Date: July 27, 2009 $
//###########################################################################

#include "DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "DSP2834x_Examples.h"   // DSP2834x Examples Include File

//---------------------------------------------------------------------------
// InitXINTF:
//---------------------------------------------------------------------------
// This function initializes the External Interface the default reset state.
//
// Do not modify the timings of the XINTF while running from the XINTF.  Doing
// so can yield unpredictable results


void InitXintf(void)
{
    // This shows how to write to the XINTF registers.  The
    // values used here are the default state after reset.
    // Different hardware will require a different configuration.

    // For an example of an XINTF configuration used with the
    // F28345 eZdsp, refer to the examples/run_from_xintf project.

    // Any changes to XINTF timing should only be made by code
    // running outside of the XINTF.

    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = SYSCLKOUT/4
    EALLOW;
	// XTIMCLK = SYSCLKOUT/2
    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
    // No write buffering
    XintfRegs.XINTCNF2.bit.WRBUFF = 0;
    // XCLKOUT is enabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 0;
    // XCLKOUT = XTIMCLK/2
    XintfRegs.XINTCNF2.bit.CLKMODE = 1;
    // XCLKOUT = XTIMCLK/4 (SYSCLKOUT/8)
    XintfRegs.XINTCNF2.bit.BY4CLKMODE = 1;


	//Setup Zine Timing
	//Note: These timing are assuming SYSCLKOUT is 300Mhz

    // Zone 0------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead (Read) must always be 1 or greater
    // Lead and Trail (Write) must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING0.bit.XWRLEAD = 6;
    XintfRegs.XTIMING0.bit.XWRACTIVE = 14;
    XintfRegs.XTIMING0.bit.XWRTRAIL = 6;
    // Zone read timing
    XintfRegs.XTIMING0.bit.XRDLEAD = 6;
    XintfRegs.XTIMING0.bit.XRDACTIVE = 14;
    XintfRegs.XTIMING0.bit.XRDTRAIL = 6;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING0.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING0.bit.USEREADY = 1;
    XintfRegs.XTIMING0.bit.READYMODE = 1;  // sample asynchronous

    // Size must be either:
    // 0,1 = x32 or
    // 1,1 = x16 other values are reserved
    XintfRegs.XTIMING0.bit.XSIZE = 3;

    // Zone 6------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead (Read) must always be 1 or greater
    // Lead and Trail (Write) must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING6.bit.XWRLEAD = 3;
    XintfRegs.XTIMING6.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING6.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING6.bit.XRDLEAD = 3;
    XintfRegs.XTIMING6.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING6.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING6.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING6.bit.USEREADY = 1;
    XintfRegs.XTIMING6.bit.READYMODE = 1;  // sample asynchronous

    // Size must be either:
    // 0,1 = x32 or
    // 1,1 = x16 other values are reserved
    XintfRegs.XTIMING6.bit.XSIZE = 3;


    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead (Read) must always be 1 or greater
    // Lead and Trail (Write) must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING7.bit.XWRLEAD = 3;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 7;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 3;
    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 3;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 7;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 3;

    // double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING7.bit.X2TIMING = 1;

    // Zone will sample XREADY signal
    XintfRegs.XTIMING7.bit.USEREADY = 1;
    XintfRegs.XTIMING7.bit.READYMODE = 1;  // sample asynchronous

    // Size must be either:
    // 0,1 = x32 or
    // 1,1 = x16 other values are reserved
    XintfRegs.XTIMING7.bit.XSIZE = 3;

    // Bank switching
    // Assume Zone 7 is slow, so add additional BCYC cycles
    // when ever switching from Zone 7 to another Zone.
    // This will help avoid bus contention.
    XintfRegs.XBANK.bit.BANK = 7;
    XintfRegs.XBANK.bit.BCYC = 7;
    EDIS;
   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.

   InitXintf16Gpio();
// InitXintf32Gpio();

   asm(" RPT #7 || NOP");

}

void InitXintf32Gpio()
{
	 // To configure the GPIOs for XINTF, set the GPIO MUX setting equal to:
	 //	- 2 for C2834x/C2824x devices
	 // Always refer to the device data manual and XINTF User's Guide for
	 // correct GPIO MUX settings

     EALLOW;
     GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 2;  // XD31
     GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 2;  // XD30
     GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 2;  // XD29
     GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 2;  // XD28
     GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 2;  // XD27
     GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 2;  // XD26
     GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 2;  // XD25
     GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 2;  // XD24
     GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 2;  // XD23
     GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 2;  // XD22
     GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 2;  // XD21
     GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 2;  // XD20
     GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 2;  // XD19
     GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 2;  // XD18
     GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 2;  // XD17
     GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 2;  // XD16

     GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 2;  // XD31 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 2;  // XD30 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 2;  // XD29 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 2;  // XD28 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO52 = 2;  // XD27 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO53 = 2;  // XD26 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2;  // XD25 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2;  // XD24 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 2;  // XD23 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 2;  // XD22 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO58 = 2;  // XD21 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO59 = 2;  // XD20 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO60 = 2;  // XD19 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO61 = 2;  // XD18 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO62 = 2;  // XD17 asynchronous input
     GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 2;  // XD16 asynchronous input


     InitXintf16Gpio();
}

void InitXintf16Gpio()
{
	EALLOW;

	GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;   // Enable pull-up on GPIO54 (SPISIMOA)
	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;   // Enable pull-up on GPIO55 (SPISOMIA)
	GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;   // Enable pull-up on GPIO56 (SPICLKA)
	GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;   // Enable pull-up on GPIO57 (SPISTEA)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3; // Asynch input GPIO16 (SPISIMOA)
	GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3; // Asynch input GPIO17 (SPISOMIA)
	GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 3; // Asynch input GPIO18 (SPICLKA)
	GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 3; // Asynch input GPIO19 (SPISTEA)

/* Configure SPI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.

	GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1; // Configure GPIO54 as SPISIMOA
	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1; // Configure GPIO55 as SPISOMIA
	GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1; // Configure GPIO56 as SPICLKA
	GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1; // Configure GPIO57 as SPISTEA

	GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0; // 配置GPIO17为GPIO口
	GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;      // 定义GPIO17输出引脚
	GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;      // 禁止上啦 GPIO17引脚

	GpioCtrlRegs.GPAMUX1.bit.GPIO0  = 1;	// epwm1A
	GpioCtrlRegs.GPAPUD.bit.GPIO0   = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO0   = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO2  = 1;	// epwm2A
	GpioCtrlRegs.GPAPUD.bit.GPIO2   = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO2   = 1;
	GpioCtrlRegs.GPAMUX1.bit.GPIO4  = 1;	// epwm3A
	GpioCtrlRegs.GPAPUD.bit.GPIO4   = 0;
	GpioCtrlRegs.GPADIR.bit.GPIO4   = 1;

	EDIS;
}

//===========================================================================
// No more.
//===========================================================================
