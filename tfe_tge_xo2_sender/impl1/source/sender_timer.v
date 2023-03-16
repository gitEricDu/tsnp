// 1、6/3/2/1/0.1 K 的信号
// 2、秒信号



`timescale	1ns/100ps
module sender_timer(
	// system inf
	input			nrst,				//复位信号	
	input			sysclk,
	input			exSync,				// 外部同步信号，电平 
	output			tri_6k,				// 电平
	output			tri_3k,				// 电平
	output			tri_1k,				// 电平
	output			sec,
	output			sec_p

,	output			spd_4k
,	output			spd_16k
,	output			spd_64k
,	output			spd_32k
,	output			spd_400

);

//parameter DIV_CODE_10US			= 11'd1249;	// 利用125M产生10us的计数器用
parameter DIV_CODE_10US			= 11'd1299;	// 应为1301，为了提高一定的速度


///////////  exSYnc上升沿提取
reg [1:0]	eg_ex_sync;

always@(posedge sysclk or negedge nrst)
if (!nrst) eg_ex_sync <= 0;
else eg_ex_sync <= {eg_ex_sync[0],exSync};

wire egr_ex_sync = ~eg_ex_sync[1] & eg_ex_sync[0];


/////////   帧定时器        ///////// 
reg [10:0]	ch_cnt_reg;
reg 		ch_cnt_cry_Reg;
reg [4:0]	framCnt;


// 10us计数器
always@(posedge sysclk or negedge nrst)
if (!nrst) ch_cnt_reg <= 0;
else if (ch_cnt_cry_Reg | egr_ex_sync) ch_cnt_reg <= 0;	
else   ch_cnt_reg <= ch_cnt_reg + 1'b1;

always@(posedge sysclk or negedge nrst)
if (!nrst) ch_cnt_cry_Reg <= 0;
else ch_cnt_cry_Reg <= (ch_cnt_reg ==  (DIV_CODE_10US - 11'd1));

// 时隙计数器，32时隙
always@(posedge sysclk or negedge nrst)
if (!nrst) framCnt <= 0;
else if(egr_ex_sync) framCnt <= 0;
else if(ch_cnt_cry_Reg) framCnt <= framCnt + 1'b1;

reg		ts1_reg;
reg		ts2_reg;

// 主要时隙产生
always @ (posedge sysclk or negedge nrst)
		if(~nrst) begin
			        ts1_reg		<= 1'b0;
					ts2_reg	<= 1'b0;		
				  end
		else  if (ch_cnt_cry_Reg) begin

					ts1_reg		<= framCnt ==5'd0;
					ts2_reg		<= framCnt ==5'd16;
					
				  end



assign tri_6k = ts1_reg | ts2_reg;		// 6250
assign tri_3k = ts1_reg;				// 3125


//    1K  ///////////////////////////////////////
reg [1:0]	cnt_1k;	
always @(posedge sysclk or negedge nrst)
if(~nrst) cnt_1k <=0;
else if (ch_cnt_cry_Reg & (framCnt== 5'd16) & (cnt_1k == 2'b10) ) cnt_1k <=0;
else if (ch_cnt_cry_Reg & (framCnt== 5'd16)) cnt_1k <= cnt_1k + 1'b1;

reg		tri_1k_reg;
always @(posedge sysclk or negedge nrst)
if(~nrst) tri_1k_reg <=0;
else tri_1k_reg <= (framCnt== 5'd16) & (cnt_1k == 2'b10);	


assign tri_1k = tri_1k_reg;		 

	
//    1秒  ///////////////////////////////////////

// 125M = 64 * 125 * 15625 ( 6b + 7b + 14b)

reg [13:0]		cnt_15625;		// 8Khz
reg [6:0]		cnt_125;		// 64hz
reg [5:0]		cnt_64;			// 1hz
reg				cry_15625;
reg				cry_125;
reg				cry_64;


always @(posedge sysclk or negedge nrst)
if(~nrst) cnt_15625 <=0;
else if(cry_15625) cnt_15625 <= 0;
else cnt_15625 <= cnt_15625 + 1'b1;

always @(posedge sysclk or negedge nrst)
if(~nrst) cry_15625 <=0;
else cry_15625 <= cnt_15625 == 14'd15623;

always @(posedge sysclk or negedge nrst)
if(~nrst) cnt_125 <=0;
else if(cry_15625 & cry_125) 	cnt_125 <=0;
else if (cry_15625) 			cnt_125 <= cnt_125 + 1'b1;

always @(posedge sysclk or negedge nrst)
if(~nrst) cry_125 <=0;
else cry_125 <= cnt_125 == 7'd124;

always @(posedge sysclk or negedge nrst)
if(~nrst) cnt_64 <=0;
else if(cry_15625 & cry_125) cnt_64 <= cnt_64 + 1'b1;

always @(posedge sysclk or negedge nrst)
if(~nrst) cry_64 <=0;
else cry_64 <= cnt_64 == 6'd63;


assign sec = cnt_64[5];	// 占空比1：1
assign sec_p = cry_64 & cry_125 & cry_15625;



//    产生 64K的触发 ///////////////////////////////////////
// 比正常的速度提高了 0.5%

reg [10:0]	htk_cnt;
reg			htkc_cry;

always @(posedge sysclk or negedge nrst)
if(~nrst) htk_cnt <=0;
else if(htkc_cry) htk_cnt <=0;
else htk_cnt <= htk_cnt + 1'b1;

always @(posedge sysclk or negedge nrst)
if(~nrst) htkc_cry <=0;
else htkc_cry <= (htk_cnt == 11'd1940);



reg [3:0]	l_hkt_cnt;
always @(posedge sysclk or negedge nrst)
if(~nrst) l_hkt_cnt <=0;
else if(htkc_cry) l_hkt_cnt <= l_hkt_cnt + 1'b1;


assign spd_64k = htk_cnt[10];
assign spd_32k = l_hkt_cnt[0];
assign spd_16k = l_hkt_cnt[1];
assign spd_4k = l_hkt_cnt[3];


//    产生 400的触发 ///////////////////////////////////////
//利用 8K的信号
reg [4:0]	cnt400_reg;
always @(posedge sysclk or negedge nrst)
if(~nrst) cnt400_reg <=0;
else if(cry_15625 &  (cnt400_reg == 5'd19)) cnt400_reg <=0;
else if(cry_15625 ) cnt400_reg <= cnt400_reg + 1'b1;

assign spd_400 = cnt400_reg[4];


endmodule
