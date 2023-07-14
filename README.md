# fsic_asic

## How to do verilog simulation
   1. cd to REPO/verif/vsim/rsim
   2. ./run_xsim base

   What run_xsim will do:
   1. Call **gen_rtl** script at REPO/src to collect all files to REPO/rtl
   2. Start verilog simulaton based on the collected file at REPO/rtl

   top bench file is located at REPO/verif/vsim/env.  All models and library reference path are also placed there

   Please modify **src.f** or **src.rtl.f** to reflect your fsic_fpga repository path of your working environment.
