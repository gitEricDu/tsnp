// Ver B
// 2020.0608
// 数据同步，解决之前的数据闪烁问题

// Ver A 
// 利用 mfc来同步显示 

//两个数据码管的显示控制
// 每个显示2秒钟，循环显示8个内容


`include "../../comm_verilog/misc/disp_88/hex2seg.v"
module disp_88_sender (
	input			nrst,
	input 			clk2m,		// 2mhz
	input [7:0]		dain,		// 第一位数据码管的显示内容 0-f
	
	output [2:0]	addr,		// 显示内容选择
	output			sdo,
	output			sclk,
	output			latch,

	output			scanp
,	input [1:0]		dp

);

wire [7:0]	dataA;
wire [7:0]	dataB;

// 产生秒信号
reg [24:0]		mcnt;
always @(posedge clk2m or negedge nrst)
	if(~nrst) mcnt <= 0;
	else mcnt <= mcnt + 1'b1;

reg		cry_reg;
always @(posedge clk2m or negedge nrst)
	if(~nrst) cry_reg <= 0;
	else cry_reg <= &mcnt[21:0];

reg		scan_reg;
always @(posedge clk2m or negedge nrst)
	if(~nrst) scan_reg <= 0;
	else scan_reg <= &mcnt[15:0];
	
assign scanp = scan_reg;
	
assign addr = mcnt[24:22];

// 定时器，将16位的数据发出去

reg [3:0]		cnt;
always @(posedge clk2m or negedge nrst)
	if(~nrst) cnt <= 0;
	else if(cry_reg) cnt <= 0;
	else if(~(&cnt))cnt <= cnt + 1'b1;


// 显示程序
reg 	en_shift;
always @(posedge clk2m or negedge nrst)
	if(~nrst) en_shift <= 0;
	  else if (cry_reg) en_shift <= 1'b1;
	  else if (&cnt) en_shift <= 1'b0;


  reg [15:0]	shift_reg;	
  always @(posedge clk2m or negedge nrst)
	  if(~nrst) shift_reg <= 0;
	  else if (cry_reg) shift_reg <= {dataA,dataB};	//{个位，十位}
	  else if(en_shift) shift_reg <= {shift_reg[14:0],1'b0};
		  
reg	latch_reg;
  always @(posedge clk2m or negedge nrst)
	  if(~nrst) latch_reg <= 0;
	  else latch_reg <= en_shift;


assign latch 	= latch_reg & (~en_shift);
assign sclk 	= ~clk2m;
assign sdo 		= shift_reg[15];

hex2seg hs0(.dain(dain[3:0]),.daq(dataA),.dp(dp[0]));
hex2seg hs1(.dain(dain[7:4]),.daq(dataB),.dp(dp[1]));

  
	
endmodule

