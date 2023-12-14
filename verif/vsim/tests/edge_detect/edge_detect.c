/*
 * SPDX-FileCopyrightText: 2020 Efabless Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * SPDX-License-Identifier: Apache-2.0
 */

// copy from "counter_la.c"
// This include is relative to $CARAVEL_PATH (see Makefile)
#include <defs.h>
#include <stub.c>
#ifdef USER_PROJ_IRQ0_EN
#include <irq_vex.h>
#endif

#define UP_BASE (0x30000000)
#define AA_BASE (0x30002000)
#define IS_BASE (0x30003000)
#define REG_UP_BASE (*(volatile uint32_t*)0x30000000)
#define REG_AA_BASE (*(volatile uint32_t*)0x30002000)
#define REG_IS_BASE (*(volatile uint32_t*)0x30003000)

#define  AA_MailBox_Reg_Offset (0x000)
#define  AA_Internal_Reg_Offset (0x100)

// --------------------------------------------------------
// define only one of below items
// --------------------------------------------------------
//#define SYSTEM_test111 1
//#define SYSTEM_test112 1
//#define SYSTEM_test103 1
//#define SYSTEM_test104 1
//#define SYSTEM_test113 1
//#define SYSTEM_test114 1
// --------------------------------------------------------

/*
	MPRJ Logic Analyzer Test:
		- Observes counter value through LA probes [31:0] 
		- Sets counter initial value through LA probes [63:32]
		- Flags when counter value exceeds 500 through the management SoC gpio
		- Outputs message to the UART when the test concludes successfuly
*/

void main()
{
	int j;

  #ifdef USER_PROJ_IRQ0_EN	
    int mask;
  #endif
  
	/* Set up the housekeeping SPI to be connected internally so	*/
	/* that external pin changes don't affect it.			*/

    reg_spi_enable = 1;
    reg_wb_enable = 1;
	
	// reg_spimaster_cs = 0x00000;

	// reg_spimaster_control = 0x0801;

	// reg_spimaster_control = 0xa002;	// Enable, prescaler = 2,
                                        // connect to housekeeping SPI

	// Connect the housekeeping SPI to the SPI master
	// so that the CSB line is not left floating.  This allows
	// all of the GPIO pins to be used for user functions.

	// The upper GPIO pins are configured to be output
	// and accessble to the management SoC.
	// Used to flad the start/end of a test 
	// The lower GPIO pins are configured to be output
	// and accessible to the user project.  They show
	// the project count value, although this test is
	// designed to read the project count through the
	// logic analyzer probes.
	// I/O 6 is configured for the UART Tx line

  #ifdef USER_PROJECT_SIDEBAND_SUPPORT	
        reg_mprj_io_36 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //IO_CLK from FPGA
        reg_mprj_io_35 = GPIO_MODE_USER_STD_OUTPUT;           //TX_CLK
        reg_mprj_io_34 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_33 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_32 = GPIO_MODE_USER_STD_OUTPUT;           //TXD


        reg_mprj_io_31 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_30 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_29 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_28 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_27 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_26 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_25 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_24 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_23 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_22 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_21 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RX_CLK
        reg_mprj_io_20 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_19 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_18 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_17 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_16 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD

        reg_mprj_io_15 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_14 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_13 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_12 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_11 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_10 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_9  = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_8  = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
  #else //USER_PROJECT_SIDEBAND_SUPPORT	
        reg_mprj_io_34 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //IO_CLK from FPGA
        reg_mprj_io_33 = GPIO_MODE_USER_STD_OUTPUT;           //TX_CLK
        reg_mprj_io_32 = GPIO_MODE_USER_STD_OUTPUT;           //TXD


        reg_mprj_io_31 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_30 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_29 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_28 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_27 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_26 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_25 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_24 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_23 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_22 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_21 = GPIO_MODE_USER_STD_OUTPUT;           //TXD
        reg_mprj_io_20 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RX_CLK
        reg_mprj_io_19 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_18 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_17 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_16 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD

        reg_mprj_io_15 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_14 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_13 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_12 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_11 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_10 = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_9  = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
        reg_mprj_io_8  = GPIO_MODE_USER_STD_INPUT_PULLDOWN;   //RXD
  
  #endif //USER_PROJECT_SIDEBAND_SUPPORT	

        
        //mprj_io_0 ~ mprj_io_7 CANNOT control here
        /*
        reg_mprj_io_7  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_5  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_4  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_3  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_2  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_1  = GPIO_MODE_USER_STD_OUTPUT;
        reg_mprj_io_0  = GPIO_MODE_USER_STD_OUTPUT;

        reg_mprj_io_6  = GPIO_MODE_MGMT_STD_OUTPUT;
		*/

	// Set UART clock to 64 kbaud (enable before I/O configuration)
	// reg_uart_clkdiv = 625;
	reg_uart_enable = 1;
	
	// Now, apply the configuration
	reg_mprj_xfer = 1;
	while (reg_mprj_xfer == 1);

    // test110 - [FW] soc CFG write to internal register - target to REG_IS_BASE
    REG_IS_BASE = 1;
    //print("Monitor: set REG_IS_BASE = 1\n\n");	// Makes simulation very long!
    REG_IS_BASE = 3;    
    //print("Monitor: set REG_IS_BASE = 3\n\n");	// Makes simulation very long!
    
    uint32_t aa_base = AA_BASE;
    uint32_t value;
    uint32_t io_serdes_base = IS_BASE;

   
    
    while(1);
}






