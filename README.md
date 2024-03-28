# fsic_asic

## How to do verilog simulation
   1. cd to REPO/verif/vsim/rsim
   2. ./run_xsim base

   What run_xsim will do:
   1. Call **gen_rtl** script at REPO/src to collect all files to REPO/dsn/rtl
   2. Start verilog simulaton based on the collected file at REPO/dsn/rtl

   top bench file is located at REPO/verif/vsim/env.  All models and library reference path are also placed there

   Please modify **REPO/src/src.f** or **REPO/src/src.user.f** to reflect your fsic_fpga repository path of your working environment.     
   please modidy line `71`,`72` and `138`,`139` of `run_xsim` script to decide to run default FSIC demo project or with user project replaced.  
   Efabless's repo also need to clone first.  
   
   **https://github.com/efabless/caravel** - refence note 1
   
   **https://github.com/efabless/caravel_mgmt_soc_litex** - refence note 2
   
   **https://github.com/efabless/caravel_user_project**  

   About How to setup user project of efabless, please refer this link first
   **https://github.com/bol-edu/caravel-lab#2-caravel-user-flow**  

   Expected Repo Hierarchy  
   <pre>
   --+--fork---+--`caravel`  
     |         |
     |         +--`caravel_mgmt_soc_litex`
     |  
     |  
     +--clone--+--`fsic_fpga`  
               |  
               +--`fsic_asic`  
               |  
               +--`caravel_user_project`  
</pre>


## Note1
git clone and create a banch from mpw-8c tag
```
$ git clone https://github.com/efabless/caravel
$ cd caravel
$ git checkout -b mpw-8c-branch mpw-8c
```

## Note2
git clone and create a banch from mpw-8c tag
```
$ git clone https://github.com/efabless/caravel_mgmt_soc_litex
$ cd caravel_mgmt_soc_litex
$ git checkout -b mpw-8c-branch mpw-8c
```

log in detail
```
tonyho@HLS05:~/workspace/fork$ git clone https://github.com/efabless/caravel_mgmt_soc_litex
Cloning into 'caravel_mgmt_soc_litex'...
remote: Enumerating objects: 8763, done.
remote: Counting objects: 100% (1475/1475), done.
remote: Compressing objects: 100% (667/667), done.
remote: Total 8763 (delta 829), reused 1405 (delta 788), pack-reused 7288
Receiving objects: 100% (8763/8763), 4.59 GiB | 11.00 MiB/s, done.
Resolving deltas: 100% (5357/5357), done.
Updating files: 100% (770/770), done.
tonyho@HLS05:~/workspace/fork$ cd caravel_mgmt_soc_litex/
tonyho@HLS05:~/workspace/fork/caravel_mgmt_soc_litex$ git checkout -b mpw-8c-branch mpw-8c
Updating files: 100% (251/251), done.
Switched to a new branch 'mpw-8c-branch'
tonyho@HLS05:~/workspace/fork/caravel_mgmt_soc_litex$ 
```
