`ifndef  PROJECT_SVH
`define  PROJECT_SVH

// ----------------------------------------------------
// Release: 11172023

// ----------------------------------------------------
// FPGA or ASIC
`define FPGA        // or ASIC


// ----------------------------------------------------
`ifdef   FPGA
                          // AMD for Xilinx, INTEL for Altera
  `define  AMD                // or INTEL

  `ifdef   AMD
  `define  AMD_FPGA_X1234
  `endif //AMD


  `ifdef   INTEL
  `define  INTEL_FPGA_PART I1234
  `endif //INTEL

`endif //FPGA

// ----------------------------------------------------
`define SIM_ONLY    // or SYNTHESIS


`endif //PROJECT_SVH
