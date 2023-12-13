//single-port RAM model 
module SPRAM
#(
parameter data_width = 8,
parameter addr_width = 7,
parameter depth = 128
)(
	adr, d, en, we, clk, q
);

input wire [addr_width-1:0] adr;
input wire [data_width-1:0] d;
input wire en;
input wire we;
input wire clk;
output reg [data_width-1:0] q;

//	reg [data_width-1:0] q;

	reg [data_width-1:0] mem [depth-1:0];

	always @(posedge clk) begin
		if (en) begin
			if (we) begin
				mem[adr] <= d;
				q <= {data_width{1'bX}} ;
			end
			else begin
				q <= mem[adr];
			end
		end
	end

endmodule
