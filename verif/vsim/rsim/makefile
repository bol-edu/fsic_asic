.PHONY:clean wave w wb wo edge test111

sim:
	xvlog -sv -i ../ tb_test.v -f filelist
	xelab tb_test -debug typical --snapshot tb_test_xelab --timescale 1ns/1ns
	xsim tb_test_xelab --tclbatch log_wave.tcl

edge:
	./run_xsim_edge edge_detect

test111:
	./run_xsim system_test111

clean:
	- rm vivado*.log vivado*.jou
	- rm xsim*.log xsim*.jou
	- rm xvlog.* xelab.*
	- rm -rf xsim.dir
	- rm -rf .Xil

wave:
	# gtkwave top_bench.vcd &
	vivado -source open_wave.tcl &

# for open current waveform
w:	wave

# for open ok waveform
wo: 
	vivado -source open_wave.ok.tcl &

# for open bad waveform
wb: 
	vivado -source open_wave.bad.tcl &
