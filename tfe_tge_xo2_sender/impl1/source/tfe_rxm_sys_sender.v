// 2022。0215 
// 会对 sender 的应用，需要提取时间戳以及提供 SFD两组信号
// 所以由 tfe_rxm_sys 通用文件改为  tfe_rxm_sys_sender 专用文件
//

// 2021.0607 使用系统时钟完成全部功能

// ctrWd 数据结构

// D[31:24]		PT,packet type
// D[23:21]		REV
// D[20:16]		IEB,index of end block (max 32 block)
// D[15:11]		ISB,index of start block (max 32 block)
// D[10:0]		DL,data length (max 1520)


// 2020.0916 建立 
// 目标识别出PAUSE/SV两种报文
// 支持MII接口
`timescale 1ns/1ps

module tfe_rxm_sys_sender (  

input           nrst, 
input			sysclk,		// 用于MIB的时钟

//MII interface  
input			rxClk,		//25Mhz
input           rxDv,                                       
input  [3:0]   	rxDa,                                       
input           rxErr, 

// RAM inf  (4k)
output [11:0]	ramAddr,
output  		ramClkEn,	
output [7:0]	ramWrDa,
 
input [4:0]		ramRdBlk,
output			ramFull,
output			ramEmpty,
// fifo
input			full,
output  		f_wrEn,
output [31:0]	f_wrDa,

// 以下信号在CRC的效后，才有效
output [7:0]	rxm_tst,
output [47:0]	dMAC,
output [47:0]	sMAC,

output			chk_pkt_cnt_l,
output [15:0]	pkt_cnt,
output [15:0]	sv_cnt,

//MIB
output [4:0]	mib_da5

, output [23:0]	rxtimestamp		// ref sysclk
, output		lts				// ref sysclk

);

reg [10:0]		data_cnt;
wire			egr_rxDv;
reg				crc_chk_eof;
wire 	   		crc_chk_err_l;

//******************************************************************************
//internal signals
//******************************************************************************

`define VLAN_TAG				16'h8100
`define TYPE_SV					16'h88ba
`define TYPE_PAUSE				16'h8808
`define TYPE_GOOSE				16'h88b8

//******************************************************************************
//    PHY 输入信号处理                                                         
//******************************************************************************
reg [7:0]		shift_reg;
reg				div_reg;
reg				da_av;
reg	[7:0]		da5_reg;
reg				sfd_reg;


// find 5
always @(posedge rxClk or negedge nrst) 
if(~nrst) da5_reg <= 0;
else da5_reg <= {da5_reg[6:0],rxDa == 4'h5};


always @(posedge rxClk or negedge nrst) 
if(~nrst) sfd_reg <= 0;
else sfd_reg <= (~da_av) & (&da5_reg) & (rxDa == 4'hd);


always @(posedge rxClk or negedge nrst) 
if(~nrst) div_reg <= 0;
else if(sfd_reg) div_reg <= 0;
else div_reg <= ~div_reg;


always @(posedge rxClk or negedge nrst) 
if(~nrst) da_av <= 0;
else if(~rxDv) da_av <= 0;
else if(sfd_reg) da_av <= 1'b1;

always @(posedge rxClk or negedge nrst) 
if(~nrst) shift_reg <= 0;
//else if(rxDv) shift_reg <= {shift_reg[3:0],rxDa};
//else  shift_reg <= 0;
else shift_reg <= {shift_reg[3:0],rxDa};



wire [7:0] 	lda = {shift_reg[3:0],shift_reg[7:4]};

//******************************************************************************
//    时钟转换 rxclk -> sysclk                                                        
//
//	1. 接收数据有效 sys_dclkEn
//  2. sfd
// 	3. rxDv
//  4. 接收数据
// 	5. egr_rxDv
// 	6. 接收数据计数 data_cnt
// 
//******************************************************************************


reg	[2:0]		sys_dclk_reg;		//12.5hz
reg [1:0]		sys_sfd_reg;
reg	[2:0]		sys_rxDv;
reg [23:0]		sys_da;
reg	[2:0]		eg_rxDv;


//1 rxClk
always @ (posedge sysclk or negedge nrst)
if (~nrst) sys_dclk_reg <= 0;
else sys_dclk_reg <= {~sys_dclk_reg[1] & sys_dclk_reg[0],sys_dclk_reg[0],div_reg};

wire sys_dclkEn 	= sys_dclk_reg[2];		// 上升沿

//2  sfd
always @ (posedge sysclk or negedge nrst)
if (~nrst) sys_sfd_reg <= 0;
else sys_sfd_reg <= {~sys_sfd_reg[0] & sys_rxDv[0],sys_rxDv[0]};

wire		sys_sfd = sys_sfd_reg[1];

// 3 rxDv
always @ (posedge sysclk or negedge nrst)
if (~nrst) sys_rxDv <= 0;
else if(sys_dclkEn) sys_rxDv <= {sys_rxDv[1:0],da_av};

//4. rxData
always @ (posedge sysclk or negedge nrst)
if (~nrst) sys_da <= 0;
else if(sys_dclkEn) sys_da <= {sys_da[15:0],lda};

// 5.
always @ (posedge sysclk or negedge nrst)
    if (~nrst) eg_rxDv <= 0; 
	else eg_rxDv 	<=  {~eg_rxDv[1]& eg_rxDv[0],eg_rxDv[0],rxDv};

assign egr_rxDv = eg_rxDv[2];

// 6.
always @ (posedge sysclk or negedge nrst)
	if(~nrst) data_cnt <= 0;
	else if(egr_rxDv) data_cnt <= 0;
	else if(sys_rxDv[0] & sys_dclkEn) data_cnt <= data_cnt + 1'b1;
							
//******************************************************************************
//  从报文中提取相关信息
//
//****************************************************************************** 
reg [23:0]		damc_da_l;
reg [23:0]		damc_da_h;
reg [23:0]		samc_da_l;
reg [23:0]		samc_da_h;
reg	[23:0]		rxTsReg;		//time stamp
reg [15:0]		tge_pkt_cnt;
reg [15:0]		tge_sv_cnt;

reg				is_tge_type;

reg				latch_ts;

always @ (posedge sysclk or negedge nrst)
    if (~nrst) begin 
    			damc_da_l  <= 0;  
    			damc_da_h  <= 0; 
    			samc_da_l  <= 0; 
    			samc_da_h  <= 0;
    			rxTsReg		<= 0;
    			latch_ts	<= 0;
    			is_tge_type <= 0;
    			tge_pkt_cnt <= 0;
    			end
    else  if(sys_dclkEn) begin
    			if (data_cnt == 11'd2 ) damc_da_h  <= sys_da[23:0];  // offset 3 
    			if (data_cnt == 11'd5 ) damc_da_l  <= sys_da[23:0];  // offset 6 
    			if (data_cnt == 11'd8 ) samc_da_h  <= sys_da[23:0];  // offset 9 
    			if (data_cnt == 11'd11) samc_da_l  <= sys_da[23:0];  // offset 12 
    			if (data_cnt == 11'd16) rxTsReg    <= sys_da[23:0];  // offset 18  2022。0215 增加
    			if (data_cnt == 11'd13) is_tge_type <= sys_da[15:0] == 16'hb000;  // offset 14  2022。0325 增加
				if (data_cnt == 11'd24) tge_pkt_cnt <= sys_da[15:0];  // offset 14  2022。0325 增加
				if (data_cnt == 11'd67) tge_sv_cnt <= sys_da[15:0];  // offset 14  2022。0325 增加
    			
    			latch_ts <= (data_cnt == 11'd18);					 // 用于外部锁存 timestamp
    			end

reg		is_tge_tpye_ok;
always @ (posedge sysclk or negedge nrst)
    if (~nrst) is_tge_tpye_ok <= 0;
    else if (sfd_reg) is_tge_tpye_ok <= 0;
    else if (crc_chk_eof) is_tge_tpye_ok <= ~crc_chk_err_l & is_tge_type;
    
assign chk_pkt_cnt_l 	= is_tge_tpye_ok;
assign pkt_cnt 			= tge_pkt_cnt;
assign sv_cnt 			= tge_sv_cnt;



reg		pack_is_vlan;
reg		pack_is_sv;
reg		pack_is_goose;
reg		pack_is_pause;


// 由于SV 或 GOOSE 可能有VLAN，所以实际的数据类型位置会变化

reg		posi_vlan;			// VLAN 的位置
reg		posi_vlan_x;		// 当有VLAN 时，状态的位置

 always @ (posedge sysclk or negedge nrst)
    if (~nrst) begin 
    			posi_vlan 	<= 0;
    			posi_vlan_x <= 0;
    			end
		else begin 
    			posi_vlan 	<= data_cnt == 11'd13;
    			posi_vlan_x <= data_cnt == 11'd17;
    			end

 always @ (posedge sysclk or negedge nrst)
    if (~nrst) begin 
    			pack_is_vlan  	<= 0;  
    			pack_is_sv  	<= 0; 
    			pack_is_goose  	<= 0; 
    			pack_is_pause  	<= 0; 
    			end
    else if (egr_rxDv) begin 
    			pack_is_vlan  	<= 0;  
    			pack_is_sv  	<= 0; 
    			pack_is_goose  	<= 0; 
    			pack_is_pause  	<= 0; 
    			end
    else if(sys_dclkEn) begin
        		if (posi_vlan) 	pack_is_vlan  	<= (sys_da[15:0] == `VLAN_TAG);  
    			if (posi_vlan | (posi_vlan_x & pack_is_vlan)) 	pack_is_pause  	<= (sys_da[15:0] == `TYPE_PAUSE);
    			if (posi_vlan | (posi_vlan_x & pack_is_vlan)) 	pack_is_sv  	<= (sys_da[15:0] == `TYPE_SV); 
    			if (posi_vlan | (posi_vlan_x & pack_is_vlan)) 	pack_is_goose  	<= (sys_da[15:0] == `TYPE_GOOSE); 

    			end
 
assign dMAC = {damc_da_h,damc_da_l};
assign sMAC = {samc_da_h,samc_da_l};
			 
//******************************************************************************
//   CRC 产生 ,用于TxDa/TxEn的数据的FCS                                                         
//******************************************************************************
reg				ram_clkEn_reg;

CRC_chk U_CRC_chk(
.Reset                      (~nrst), 
.Clk                        (sysclk),
.CRC_init                   (egr_rxDv),		// 此对应 RxD_reg的SFD
.CRC_data                   (sys_da[15:8]),		// 使用输入的原始数据
.CRC_en                     (ram_clkEn_reg),		// 对应RxD_reg的L2数据 

 //From CPU  
.CRC_chk_en                 (1'b1),		// CRC_err 信号
.CRC_err                    (),				// 逻辑输出，当前没用
.CRC_err_l					(crc_chk_err_l)	// One clock delay compared to CRC_err
); 

//******************************************************************************
//
//   RAM  INF
//
//   sys_rxDa / crc_daEn 包括CRC在内的全部报文的数据，可以此为参考，时钟clk的上升沿
//
//******************************************************************************

reg				dis_ram_wr;
reg [11:0]		ram_addr_cnt;
reg [4:0]		index_sb;
wire [4:0]		blk_addr 	= ram_addr_cnt[11:7];
reg				ram_full_reg;
reg				dis_card;
reg				undis_card;

always @ (posedge sysclk or negedge nrst)
if(~nrst) crc_chk_eof <= 0;
else crc_chk_eof <= sys_rxDv[2] & (~sys_rxDv[1]) & sys_dclkEn;

always @ (posedge sysclk or negedge nrst)
if(~nrst) dis_card <= 0;
else dis_card <= (dis_ram_wr |crc_chk_err_l) & crc_chk_eof;

always @ (posedge sysclk or negedge nrst)
if(~nrst) undis_card <= 0;
else undis_card <= (~(dis_ram_wr |crc_chk_err_l)) & crc_chk_eof;


always @ (posedge sysclk or negedge nrst)
    if (~nrst) ram_clkEn_reg <= 0;
    else ram_clkEn_reg <= sys_rxDv[1] & (~sys_dclk_reg[0] & sys_dclk_reg[1]);		// 相当于使用下降沿做为RAM的写入时刻

assign ramClkEn = ram_clkEn_reg;
assign ramWrDa  = sys_da[15:8];
 

// 1。接收报文的开始
// 2。写完一个blk
// 在以上两个条件下，对 ramFull 及 full 进行检查 

always @ (posedge sysclk or negedge nrst)
if (!nrst) 	dis_ram_wr <= 0;
else if(egr_rxDv | &{ram_addr_cnt[6:0]} ) dis_ram_wr <= ramFull | full;

// 1. 释放当前ram，条件：FCS错 或 RAM/FIFO 满，数据不完整，在crc_chk_eof处检查
// 2. 指向下一个BLK
// 3. 地址计数器
always @ (posedge sysclk or negedge nrst)
if (!nrst) 	ram_addr_cnt <= 0;
else  begin
	     if(dis_card) 						ram_addr_cnt <= {index_sb,7'h0};					// 释放RAM
	else if(undis_card & (~ram_full_reg)) 	ram_addr_cnt <= {(ram_addr_cnt[11:7] + 1'b1),7'h0};	// 指向下一个数据块，应对下一下报文的到来
	else if(sys_rxDv[1] & sys_dclkEn)		ram_addr_cnt <= ram_addr_cnt + 1'b1;				// 正常地址处理
end

assign ramAddr  = ram_addr_cnt;

//******************************************************************************
//
//   FIFO control
//
//  data format {packet_type[7:0],3'h0,index_eb[4:0],index_sb[4:0],data_length[10:0]}
//
//******************************************************************************
reg	[10:0]	data_length;
reg [4:0]	index_eb;

wire [7:0] 	packet_type = {4'h0,pack_is_vlan,pack_is_goose,pack_is_sv,pack_is_pause};

always @ (posedge sysclk or negedge nrst)
    if (~nrst) data_length <= 0; 
    else if(egr_rxDv) 	  	data_length <= 0;
	else if(crc_chk_eof)	data_length <= data_cnt;// - 11'h4;		// 包括FCS的数据长度

always @ (posedge sysclk or negedge nrst)
    if (~nrst) index_sb <= 0; 
    else if(egr_rxDv) index_sb <= 0; 
    else if(sys_sfd) index_sb <= blk_addr;	

wire [11:0]	tmp_ram_addr = ramAddr - 1'b1;

always @ (posedge sysclk or negedge nrst)
    if (~nrst) index_eb <= 0;
    else if(egr_rxDv) index_eb <= 0;
    else if(crc_chk_eof) index_eb <= tmp_ram_addr[11:7];

reg [2:0]	fifo_wr_cnt;
reg			fifo_wr_reg;

always @ (posedge sysclk or negedge nrst)
    if (~nrst) fifo_wr_cnt <= 0; 
    else if(fifo_wr_reg) fifo_wr_cnt <= fifo_wr_cnt + 1'b1;

always @ (posedge sysclk or negedge nrst)
    if (~nrst) fifo_wr_reg <= 0; 
    else if(undis_card) 	fifo_wr_reg <= 1'b1; 
    else if(&fifo_wr_cnt)	fifo_wr_reg <= 0;

assign f_wrEn 	= fifo_wr_reg;
assign f_wrDa	= {packet_type,3'h0,index_eb,index_sb,data_length};


//****************************************************************************** 
//   tfe_tbuf RAM——FULL
//
//  rAddr == wAddr   	=> empty   	复位时状态
//  rAddr == wAddr - 1  => empty   	工作时状态
//  rAddr == wAddr + 1  => full   	工作时状态
//
//  当前有一个BLK，一直无法使用，参照仿真图形
//   
//****************************************************************************** 
reg 		ram_empty_reg;

always @(posedge sysclk or negedge nrst)
if(~nrst) begin ram_full_reg <= 0; ram_empty_reg <= 0; end
else   begin 
			ram_full_reg 	<= (ramRdBlk == (blk_addr + 1'b1));
			ram_empty_reg 	<= (ramRdBlk == (blk_addr - 1'b1)) || (ramRdBlk == blk_addr);
	   end

assign ramFull 	= ram_full_reg;
assign ramEmpty = ram_empty_reg;


//****************************************************************************** 
//    RAM / FIFO 写入禁止
//   
//****************************************************************************** 


//****************************************************************************** 
//   MIB 
//
//   1、rxDv 			用于统计收到的全部报文数量
//   2、|pkt_ok_reg		展宽的信号，用于统计正确的报文的数量,（不包括FCS错及RAM、FIFO溢出造成的丢包）
//	 3、|drop_pkt_reg	展宽的信号，用于统计drop报文 （FCS正确，RAM、FIFO溢出造成的丢包）
//	 4、pack_is_goose 	goose报文统计
//	 5、pack_is_sv 		SV报文统计
//   
//****************************************************************************** 

reg 	pkt_ok_reg;
reg 	drop_pkt_reg;

always @ (posedge sysclk or negedge nrst)
    if (~nrst) pkt_ok_reg <= 0; 
    else if(egr_rxDv) 	pkt_ok_reg <= 0; 
    else if(crc_chk_eof) pkt_ok_reg <= ~crc_chk_err_l; 

   // else if(undis_card) pkt_ok_reg <= 1'b1; crc_chk_eof

always @ (posedge sysclk or negedge nrst)
    if (~nrst) drop_pkt_reg <= 0; 
    else if(egr_rxDv) drop_pkt_reg <= 0; 
    else if(dis_card) drop_pkt_reg <= 1'b1;
    
//assign mib_da5	= {full,ram_full_reg,drop_pkt_reg,pkt_ok_reg,fifo_wr_reg};
assign mib_da5	= {full,ram_full_reg,drop_pkt_reg,pkt_ok_reg,sys_rxDv[0]};


//****************************************************************************** 
//   TEST 
//   
//****************************************************************************** 

assign rxm_tst = 0;


assign rxtimestamp = rxTsReg;
assign lts = latch_ts;

endmodule  
