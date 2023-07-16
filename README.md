# fsic_asic

## How to do verilog simulation
   1. cd to REPO/verif/vsim/rsim
   2. ./run_xsim base

   What run_xsim will do:
   1. Call **gen_rtl** script at REPO/src to collect all files to REPO/dsn/rtl
   2. Start verilog simulaton based on the collected file at REPO/dsn/rtl

   top bench file is located at REPO/verif/vsim/env.  All models and library reference path are also placed there

   Please modify **REPO/src/src.f** or **REPO/src/src.user.f** to reflect your fsic_fpga repository path of your working environment.     
   please modidy line `57` or `58` of `run_xsim` script to decide to run default FSIC demo project or with user project replaced.  
   Efabless's repo **https://github.com/efabless/caravel**, **https://github.com/efabless/caravel_mgmt_soc_litex** also need to clone first.
