// 2022.0215
// 从任意报文中提出时间戳

// 2021.0208

// V 2.3
// 增加 MASTER初始化时自动分配 通信信道的功能

// 2020.0701 

// 增加了对本地自动分配的地址的判断输出
// rxLocalMac : 由MASTER自动分配给DNM的地址
// rxDnmNum	:	MASTER工作时，进行的设备所在位置的测试值，此值应与rxLocalMac相等
// rxLocalMac == rxDnmNum 则说明自动分配的地址是正确的，localIndexAviable = 1'b1;



// 2020.0609  :::: 20200609
// Ver 2.2
// 改为从SYNC中提取config的值，并进行可靠性处理
// 目的：	
// 		1、环路控制
//		2、DNM终端检测（数量检测）
//		3、从type中提取DNM的数量，only for MAM

// Ver 2.1 20191225
//  		增加了复帧提取功能
// 20200428 复帧计数器由8位调整为16位

//
// Ver 2.0  20191016
// 源文件来自于 gmii_rx_v1.1
// 优化，提高速度，适用于xo2-4000hc-4的芯片（速度最高为 -6）

// rev 1.1  2018.12.04
//   		增加维护、管理状态寄存器

// 20180713： 来自于eddst_rx文件


module rxm_v23_sender (
input           nrst,         
input           enLatch,		// H 允许更改，当进入激活状态后，不再更新local_mac/host_mac/delay_time/config_value的值
input			rst_latch,		// H 复位local_mac/host_mac/delay_time/config_value的值 
input			rst_p,			// H 脉冲复位，2clk，复位这些数据assign rxLocalMac/assign rxHostMac/assign rxLastCmd/assign rxDelayTime/assign rxCfgValue，slave 有效
input			one_second_l,	// H 一个2m时钟宽,用于统计数据

//GMII interface  
input			gmiiClk,
input           gmiiRxDv,                                       
input  [7:0]   	gmiiRxD,                                       
input           gmiiRxErr, 

// GMII output interface
output			gmiiTxEn,	// 用于phy的enable
output [7:0]	gmiiTxDa,	// 发送至phy的数据 

// ***************  Drop *********************************

output [7:0]	rx_data,		// 来自寄存器 dly_reg4，有部份内容被改变，但不会产生不良的影响
output			rx_end,			// 寄存器值，外部没用
output			rx_da_en,		// 两个寄存器，去掉MAC及FCS后的用户数据
output			rxCrcErr,		// 电平


// 以下信号在CRC的效后，才有效
output          findReset_l,
output          findDevice_l,		// 以下两个命令用于MASTER
output          findDeviceEnd_l,
output			findInit_l,			// 用于不同的时钟
output			findActive_l,
output			findSync_l,			// 电平信号，用于同步外部，与报文同长

// 以下信号接收到报文后即有效
output          findCtrl_l,
output          findEctrl_l,
output          findEdata_l,
output          findData_l,
output          findStstus_l,
output			findUnknow_l,

output			chk_pkt_cnt_l,
output [15:0]	pkt_cnt,
output [15:0]	sv_cnt,


// ***************  ADD/DROP *********************************

// data ADD/DROP
input [15:0]	dropDaAddr,			// {start,end} 数据下线的起始地址，若=0，表示关闭插入功能，数据范围1-128，包括此址
output			dropDaEn,			// 数据下线允许
output [7:0]	dropDa,				// 此信号与dropDaEn 配合，可直接将接收到的数据写到RAM中去，这是本地接收的数据

input [15:0]	addDaAddr,			// {start,end} 数据上线的起始地址，若=0，表示关闭插入功能，数据范围1-128，包括此址
output			addDaEn,			// 数据插入允许
input [7:0]		addDa,				// 插入的数据报文的数据


// status ADD/DROP
input [15:0]	dropStAddr,		// {start,end} 状态的下线的起始地址，若=0，表示关闭插入功能，数据范围1-64，包括此址
output			dropStEn,			// 数据下线允许
output [7:0]	dropStDa,			// 此信号与dropStEn 配合，可直接将接收到的数据写到RAM中去，这是本地接收的数据

input [15:0]	addStAddr,			// {start,end} 状态的上线的起始地址，若=0，表示关闭插入功能，数据范围1-128，包括此址
output			addStEn,			// 数据插入允许
input [7:0]		addStDa,			// 插入的状态报文的数据

// ***************  misc *********************************

// 输出报文
output [15:0]	rxHostMac,		// only for slave init报文中提取，本模块内部没用
output [7:0]	rxLocalMac,		// only for slave init报文中提取，
output [23:0]	rxDelayTime,	// only for slave  ACTIVE/init 报文提取，delay_time/timsstamp 二组数据，本模块没用
output [31:0]	rxCfgValue,		// only for slave ACTIVE报文提取，本模块没用
output [7:0]	rxLastCmd,		// master/slave ctrl/ectrl/edata 三个报文中提取，本模块没用
output [1:0]	mfc,			// only for slave sync报文中提取，用于复帧同步,{H,L}
output [39:0]	rxRev,			// 用于测试
output [7:0]	star_chNo,		// 从 master 接收到的起始地址

input [7:0]		localMac,		// 四个命令 ctrl/ectrl/edata/active 使用此地址判断

// 检测及管理
output [15:0]	rxPacketNum,
output [15:0]	rxPacketCrcErrNum,
output [7:0]	rxDnmNum,			// 输出DNM的数量，只有MAM使用
output 			localIndexAviable,	// 本地地址校验正确，内部不使用，用于系统纠错

// mib 
output [1:0]	mib_da2,
// test
output			led_crc,			// 每秒更新一次，用于LED指示是否有CRC错误
output			rxSfd
, output		lts					// 电平

);
//******************************************************************************
//internal signals
//******************************************************************************

parameter       State_idle          =4'd00;
parameter       State_preamble      =4'd01;
parameter       State_SFD           =4'd02;
parameter       State_data          =4'd03;
parameter       State_checkCRC      =4'd04;
parameter       State_OkEnd         =4'd05;
parameter       State_drop          =4'd06;
parameter       State_IFG           =4'd07;

reg [2:0]       Current_state /* synthesis syn_keep=1 */;                                                   
reg [3:0]       ifgCnt;   
reg             RxDv_reg;      
reg [7:0]       RxD_reg;
reg             RxErr_reg;
reg             Too_short;
reg             rx_data_en;	// 对应RxD_reg的数据
reg [7:0]		local_mac_reg;
reg [23:0]		delay_reg;
reg [31:0]		conf_reg;
reg [39:0]		rev_reg;
reg [15:0]		host_mac_reg;
reg [7:0]		last_cmd_reg;
reg				rx_end_reg;
reg [7:0]		dly_reg3;
reg [7:0]		dly_reg2;
reg [7:0]		dly_reg1;
reg [7:0]		dly_reg0;
reg				dly_en_reg2;
reg				dly_en_reg1;
reg				dly_en_reg0;
reg				tx_en_reg3;
reg				tx_en_reg2;
reg				tx_en_reg1;
reg				tx_en_reg0;
reg				rx_sfd_reg;
wire [7:0]		crc_gen_daout;
wire			crc_gen_enable;
reg				crc_chk_err_level_reg;
reg 			crc_chk_eof;
reg [6:0]		genCnt;
reg				insert_device_cnt;		// DNM 计数
wire			crc_chk_err_l;


 assign rxCrcErr 	= crc_chk_err_level_reg;
 assign rxSfd		= rx_sfd_reg;
 wire   Clk			= gmiiClk;


//******************************************************************************
//delay signals                                                          
//******************************************************************************
    
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) 
        begin  
            RxDv_reg      <=0;
            RxD_reg       <=0;                                                            
            RxErr_reg     <=0; 
        end
    else
        begin  
            RxDv_reg	<=gmiiRxDv;
            RxD_reg		<=gmiiRxD;                                                            
            RxErr_reg	<=gmiiRxErr; 
        end


// 优化	
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) crc_chk_eof <= 0;
    else crc_chk_eof <= tx_en_reg0 & ~RxDv_reg;// & ~gmiiRxDv;

                   
//******************************************************************************
//State_machine                                                           
//******************************************************************************
	
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) Current_state   <=State_idle;                   
    else  begin              
                                                                                                 
        case (Current_state)                            
            State_idle:		if (RxDv_reg&&RxD_reg==8'h55)	Current_state <=State_preamble;  	// 正常					
            State_preamble:	if (!RxDv_reg | RxErr_reg)		Current_state <=State_drop;   	//    
							//else if (RxErr_reg)			Current_state <=State_drop;        
							else if (RxD_reg==8'hd5)	Current_state <=State_SFD;                 
							else if (RxD_reg!=8'h55)	Current_state <=State_drop;  // pre状态下的错误统计   

						
            State_SFD:		if (!RxDv_reg | RxErr_reg)		Current_state <=State_drop;      
							//else if (RxErr_reg)	Current_state <=State_drop;        
							else				Current_state <=State_data;    
						
            State_data:		if (!RxDv_reg&&!Too_short) 		Current_state <=State_checkCRC;   
							else if (!RxDv_reg&&Too_short || RxErr_reg)	Current_state <=State_drop;		// 小于64字节的短报文
						//	else if (RxErr_reg)				Current_state <=State_drop;        
						
            State_checkCRC:	if (crc_chk_err_level_reg)Current_state <=State_IFG;
							else         			Current_state <=State_OkEnd; 
						 
            State_drop:		if (!RxDv_reg)  		Current_state <=State_IFG;    // 在接收过程中，出现 RxErr用于错误统计  
   						
            State_OkEnd:  	Current_state <=State_IFG;         						// 接收正常，用于正常报文统计
       //     State_ErrEnd:	Current_state <=State_IFG;                        		// RxDv出错，用于错误统计                                              
       //     State_CRCErrEnd:Current_state <=State_IFG;                          	// CRC出错，用于错误统计 
		//	State_err_pre:	Current_state <=State_IFG;                          	// 同步码存在问题，用于错误统计
            State_IFG: 		if (ifgCnt==4'd8)	Current_state <=State_idle;   		//remove some additional time                                                     
            default:		Current_state <=State_idle;        
        endcase                                         
		end

//******************************************************************************
//  计数据器
//  1、ifgCnt 用于报文间隔定时
//  2、genCnt 用于小于128个字前的头部数据定位
//****************************************************************************** 
always @ (negedge nrst or posedge Clk)                 
    if (~nrst)   						ifgCnt     <=0;   
    else if (Current_state==State_IFG) 	ifgCnt     <=ifgCnt + 1;                               
    else  								ifgCnt     <=0; 

always @ (posedge Clk or negedge nrst)
    if (~nrst)          					genCnt        <=0;  //7bit 
    else if (Current_state==State_SFD ) 	genCnt        <=0;
    else if (Current_state==State_data && ~(&genCnt)) 	genCnt  <=genCnt+ 1'b1;
		
//******************************************************************************
//  公用变量
//  1、rx_data_en 包括DMAC/SMAC的数据有效信号，与RxD_reg相对应
//  2、crc_chk_init_reg 用于初始化CRC模块，之前使用的是组合信号，现改为reg信号，与 RxD_reg 流匹配
//  3、too_short 用于短报文时的告警提示
//****************************************************************************** 
// RxD_reg的数据 
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) rx_data_en <= 1'b0;   
    else if ((Current_state==State_data||Current_state==State_SFD ||rx_sfd_reg ) && gmiiRxDv ) 	rx_data_en <= 1'b1;                              
    else rx_data_en <= 1'b0;
		
// 对于 RxD_reg的数据
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) rx_sfd_reg <= 1'b0;   
    else if (Current_state==State_preamble && gmiiRxD==8'hd5 ) 	rx_sfd_reg <= 1'b1;                              
    else rx_sfd_reg <= 1'b0;


// 优化
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) Too_short <= 1'b0;   
    else if (genCnt < 7'd63)  Too_short   <=1'b1;                            
    else  Too_short   <=1'b0;


  
//************************************************************************************************
//   1、数据定位，除了命令报文，主从不同外，其它报文的MAC址完全一样，所以只需要定位DMAC地址即可
//   2、用于内部数据提取，清除CRC报文
//************************************************************************************************

reg			posi_dmac_l;
reg			posi_type;
reg			posi_pkt_cnt;
reg			posi_sv_cnt;
reg [23:0]	gmacdata;
reg [7:0]	rx_tmp_da;
								
// 数据位位置定位，针对数据流RxD_reg的定位
always @ (posedge Clk or negedge nrst)
    if (~nrst) 	begin 
    			posi_dmac_l <= 1'b0;
    			posi_type 	<= 1'b0;
    			posi_pkt_cnt<= 0;
    			posi_sv_cnt<= 0;
    			end
	else if(rx_data_en) begin
			posi_dmac_l 	<= (genCnt == 7'd4);		// 对应DMAS的最后2位
			posi_type 		<= (genCnt == 7'd12);		// 对应type的数据
			posi_pkt_cnt 	<=  (genCnt == 7'd23);		// 对应type的数据
			posi_sv_cnt 	<=  (genCnt == 7'd66);		// 对应type的数据
			end
	else posi_dmac_l <= 1'b0;
	

always @ (posedge Clk or negedge nrst)
    if (~nrst) begin gmacdata <= 0;rx_tmp_da <= 0; end
	else begin 
			gmacdata[7:0] 	<= RxD_reg;
			gmacdata[15:8] 	<= gmacdata[7:0];
			gmacdata[23:16] <= gmacdata[15:8];
			rx_tmp_da		<= gmacdata[23:16];
		end
			 

//******************************************************************************
//
//   报文解析，共10类报文，全部解出电平信号，对于电平信号没有做CRC检查
//
//******************************************************************************


	reg		init_l_reg;
	reg		active_l_reg;
	reg		sync_l_reg;
	reg		reset_l_reg;
	reg		device_reg;
	reg		device_end_reg;
	
	reg		ctrl_l_reg;
	reg		ectrl_l_reg;
	reg		edata_l_reg;
	reg		data_l_reg;
	reg		status_l_reg;
	reg		unknow_l;


	// 此处使用16位数据，主要是提高速度
	wire [15:0]		source_da = {gmacdata[23:16],gmacdata[7:0]};

	
	always @ (posedge Clk or negedge nrst)
    if (~nrst) begin
				init_l_reg 		<= 1'b0;
				active_l_reg 	<= 1'b0;
				sync_l_reg 		<= 1'b0;
				reset_l_reg 	<= 1'b0;
				device_reg		<= 1'b0;
				device_end_reg	<= 1'b0;
				ctrl_l_reg 		<= 1'b0;
				ectrl_l_reg 	<= 1'b0;
				edata_l_reg 	<= 1'b0;
				data_l_reg 		<= 1'b0;
				status_l_reg 	<= 1'b0;
				unknow_l		<= 1'b0;
				
		end
	
	else if(~dly_en_reg2)begin
				init_l_reg 		<= 1'b0;
				active_l_reg 	<= 1'b0;
				sync_l_reg 		<= 1'b0;
				reset_l_reg 	<= 1'b0;
				device_reg		<= 1'b0;
				device_end_reg	<= 1'b0;			
				ctrl_l_reg 		<= 1'b0;
				ectrl_l_reg 	<= 1'b0;
				edata_l_reg 	<= 1'b0;
				data_l_reg 		<= 1'b0;
				status_l_reg 	<= 1'b0;
				unknow_l		<= 1'b0;
		end

	else if( posi_dmac_l) begin

	
				if(source_da == 16'h70_00) init_l_reg 		<= 1'b1;	
		   else if(source_da == 16'h70_01) device_reg 		<= 1'b1;
		   else if(source_da == 16'h70_02) device_end_reg	<= 1'b1;
		   else if(source_da == 16'h73_00) sync_l_reg 		<= 1'b1;
		   else if(source_da == 16'h72_00) reset_l_reg 		<= 1'b1;
		   else if(source_da == 16'h00_00) data_l_reg 		<= 1'b1;
		   else if(source_da == 16'h01_00) status_l_reg 	<= 1'b1;
				
		   else if(source_da == {8'h71,localMac}) 	active_l_reg 	<= 1'b1;
		   else if(source_da == {8'h61,localMac}) 	ctrl_l_reg 		<= 1'b1;
		   else if(source_da == {8'h62,localMac}) 	ectrl_l_reg 	<= 1'b1;
		   else if(source_da == {8'h63,localMac}) 	edata_l_reg 	<= 1'b1;
		   else   unknow_l <= 1'b1;
				
				end	

		
	// 电平输出信号		
	assign findCtrl_l 	= ctrl_l_reg;
	assign findEctrl_l 	= ectrl_l_reg;
	assign findEdata_l 	= edata_l_reg;
	assign findData_l 	= data_l_reg;
	assign findStstus_l = status_l_reg;
	assign findUnknow_l	= unknow_l;
	


	
//******************************************************************************
//
//   命令解析，共六个命令类报文，输出为电平信号，宽度为3 cycle
//
//******************************************************************************
	
reg [2:0]	init_w1_reg;
reg [2:0]	active_w1_reg;
reg [2:0]	reset_w1_reg;
reg [2:0]	sync_w1_reg;
reg [2:0]	device_w1_reg;
reg [2:0]	device_end_w1_reg;
reg 		crc_err_reg;
	
	
always @ (posedge Clk or negedge nrst) 
    if (~nrst)  crc_err_reg <= 0; 
	else crc_err_reg 	<= rx_end_reg & ~crc_chk_err_level_reg;  

		
always @ (posedge Clk or negedge nrst)
    if (~nrst) begin
				init_w1_reg 	<= 0;
				active_w1_reg 	<= 0;
				reset_w1_reg 	<= 0;
				sync_w1_reg 	<= 0;
				device_w1_reg 	<= 0;
				device_end_w1_reg 	<= 0;
		end
	else begin
				init_w1_reg 	<= {init_w1_reg[1:0],	init_l_reg 		& crc_err_reg};
				active_w1_reg 	<= {active_w1_reg[1:0],	active_l_reg 	& crc_err_reg};
				reset_w1_reg 	<= {reset_w1_reg[1:0],	reset_l_reg 	& crc_err_reg};
				sync_w1_reg 	<= {sync_w1_reg[1:0],	sync_l_reg 		& crc_err_reg};
				device_w1_reg 	<= {device_w1_reg[1:0],	device_reg 		& crc_err_reg};
		device_end_w1_reg 	<= {device_end_w1_reg[1:0],device_end_reg 	& crc_err_reg};
		end

// 电平信号（3*cycle),此信号在报文发送完成后有效,否则会出现不完整报文（状态机切换造成的）
assign findInit_l 		= |init_w1_reg;
assign findActive_l 	= |active_w1_reg;
assign findDevice_l 	= |device_w1_reg;
assign findDeviceEnd_l 	= |device_end_w1_reg;
assign findSync_l 		= |sync_w1_reg;
assign findReset_l 		= |reset_w1_reg;



reg				type_reg;
reg [15:0]		pkt_cnt_reg;
reg [15:0]		pkt_sv_reg;
reg				chk_pkt_cnt;


always @ (posedge Clk or negedge nrst)
    if (~nrst) type_reg <= 0;
    else if(rx_sfd_reg) type_reg <= 0;
    else if(posi_type) type_reg <= gmacdata[15:0] == 16'hc000;		// type is tfe

always @ (posedge Clk or negedge nrst)
    if (~nrst) pkt_cnt_reg <= 0;
    else if(rx_sfd_reg) pkt_cnt_reg <= 0;
    else if(posi_pkt_cnt) pkt_cnt_reg <= gmacdata[15:0] ;	
    
always @ (posedge Clk or negedge nrst)
    if (~nrst) pkt_sv_reg <= 0;
    else if(rx_sfd_reg) pkt_sv_reg <= 0;
    else if(posi_sv_cnt) pkt_sv_reg <= gmacdata[15:0] ;

always @ (posedge Clk or negedge nrst)
    if (~nrst) chk_pkt_cnt <= 0;
    else if (rx_sfd_reg) chk_pkt_cnt <= 0;
    else if (rx_end_reg) chk_pkt_cnt <= ~crc_chk_err_l & type_reg;		

assign chk_pkt_cnt_l 	= chk_pkt_cnt;
assign pkt_cnt			= pkt_cnt_reg;
assign sv_cnt			= pkt_sv_reg;

	
//******************************************************************************
//
// 插入数据控制
//
// 1. STATUS报文，
// 2. data报文，
//
//******************************************************************************

reg [5:0]   insert_cnt;			// 公用计数据器，本处使用
reg 		drop_status_en;
reg 		add_status_en;
reg 		drop_data_en;
reg 		add_data_en;
reg [7:0]	ch_cnt_64B;
reg [7:0]	ch_cnt_16B;
reg			rx_da_en_reg;
reg			avaib_status_drop;
reg			avaib_status_add;
reg			avaib_data_drop;
reg			avaib_data_add;


// 这两信信号用于内部切换数据
reg		dly_add_data_en;
reg		dly_add_status_en;

// 5 bit counter
always @ (posedge Clk or negedge nrst)
    if (~nrst)          					insert_cnt        <= 0; 
    else if (Current_state==State_SFD ) 	insert_cnt        <= 0;
    else if (Current_state==State_data) 	insert_cnt        <= insert_cnt+ 1'b1;
    

// data oacket counter (64 byte per channel)  与数据流 RxD_reg 合并
always @ (posedge Clk or negedge nrst)
    if (~nrst)   ch_cnt_64B <= 0;
    else if(data_l_reg )   begin 
    			if (insert_cnt == 6'ha) ch_cnt_64B <= ch_cnt_64B + 1'b1;
    			end
    else ch_cnt_64B <= 0;
		
// status packet counter (16 byte per channel)  与数据流 dly_reg0 合并 
always @ (posedge Clk or negedge nrst)
    if (~nrst)   ch_cnt_16B <= 0;
    else if(status_l_reg)   begin 
    			if (insert_cnt[3:0] == 4'hb)ch_cnt_16B <= ch_cnt_16B + 1'b1;
    			end
    else ch_cnt_16B <= 0;

// 为了提高速度，数据的有效性判断预处理
always @ (posedge Clk or negedge nrst)
    if (~nrst)    begin avaib_status_drop <= 0;avaib_status_add <= 0; avaib_data_drop <= 0;  avaib_data_add <= 0; end
    else begin 
    		avaib_status_drop 	<= (dropStAddr[15:8] != 0) && (dropStAddr[7:0]> dropStAddr[15:8]) && (dropStAddr[7:0] <= 8'd65);
    		avaib_status_add 	<= (addStAddr[15:8]  != 0) && (addStAddr[7:0] > addStAddr[15:8])  && (addStAddr[7:0] 	<= 8'd65);
    		avaib_data_drop 	<= (dropDaAddr[15:8] != 0) && (dropDaAddr[7:0]> dropDaAddr[15:8]) && (dropDaAddr[7:0] <= 8'd129);
    		avaib_data_add 		<= (addDaAddr[15:8]  != 0) && (addDaAddr[7:0] > addDaAddr[15:8])  && (addDaAddr[7:0]  <= 8'd129);
    	end	


// 定位status数据的插入位置	
always @ (posedge Clk or negedge nrst)
    if (~nrst)   drop_status_en <= 1'b0;
    else if((ch_cnt_16B == dropStAddr[15:8]) && avaib_status_drop && status_l_reg)   drop_status_en <= 1'b1;
		 else  if(ch_cnt_16B == dropStAddr[7:0])drop_status_en <= 1'b0;			 			 
always @ (posedge Clk or negedge nrst)
    if (~nrst)   add_status_en <= 1'b0;
    else if((ch_cnt_16B == addStAddr[15:8]) && avaib_status_add && status_l_reg)   add_status_en <= 1'b1;
		 else  if(ch_cnt_16B == addStAddr[7:0]) add_status_en <= 1'b0;


// 定位data数据的插入位置	
always @ (posedge Clk or negedge nrst)
    if (~nrst)   drop_data_en <= 1'b0;
    else if((ch_cnt_64B == dropDaAddr[15:8]) && avaib_data_drop && data_l_reg)   drop_data_en <= 1'b1;
		 else  if(ch_cnt_64B == dropDaAddr[7:0]) drop_data_en <= 1'b0;	
always @ (posedge Clk or negedge nrst)
    if (~nrst)   add_data_en <= 1'b0;
    else if((ch_cnt_64B == addDaAddr[15:8]) && avaib_data_add && data_l_reg)   add_data_en <= 1'b1;
		 else  if(ch_cnt_64B == addDaAddr[7:0]) add_data_en <= 1'b0;	



// 这两个信号用于读取外部RAM，提前了一个cycle
assign dropDaEn 	= drop_data_en;						//RAM 写入
assign dropDa	 	= gmiiRxD;//drop_data_en ? gmiiRxD : 8'h0;	// MUX 主要用于测试方便
assign addDaEn 		= add_data_en;						// RAM 读出

assign dropStEn		= drop_status_en;
assign dropStDa	 	= RxD_reg;//drop_status_en ? RxD_reg : 8'h0;	// MUX 主要用于测试方便
assign addStEn		= add_status_en;



// 这两个信号用于控制内部插入操作，比上述信号滞后一个时钟周期
always @ (posedge Clk or negedge nrst)
    if (~nrst)    begin 
    				dly_add_data_en 	<= 1'b0; 
    				dly_add_status_en 	<= 1'b0; 
    				end
    else 		 begin 
    				dly_add_data_en 	<= add_data_en;
    				dly_add_status_en 	<= add_status_en;
    			end
    

// 数据插入及时序控制输出                                                            

always @ (posedge Clk or negedge nrst)
    if (~nrst) begin
				dly_reg0 	<= 8'h0;
				dly_reg1 	<= 8'h0;
				dly_reg2 	<= 8'h0;
				dly_reg3 	<= 8'h0;
				
				dly_en_reg0 <= 1'b0;
				dly_en_reg1 <= 1'b0;
				dly_en_reg2 <= 1'b0;
				
				tx_en_reg0	<= 1'b0;
				tx_en_reg1	<= 1'b0;
				tx_en_reg2	<= 1'b0;
				tx_en_reg3	<= 1'b0;	
		end
	else begin
				dly_reg0 	<= dly_add_data_en   ? addDa   : RxD_reg;
				dly_reg1 	<= dly_add_status_en ? addStDa : dly_reg0;			
				dly_reg2 	<= insert_device_cnt ? (rev_reg[7:0] + 1'b1) : dly_reg1;		// 此处加1，每通过一个DNM，则增1
				dly_reg3 	<= crc_gen_enable ? crc_gen_daout : dly_reg2;					// 插入CRC的值  
				
				dly_en_reg0 <= rx_data_en;    
				dly_en_reg1 <= dly_en_reg0;
				dly_en_reg2 <= dly_en_reg1;
				
				tx_en_reg0	<= RxDv_reg;
				tx_en_reg1	<= tx_en_reg0;
				tx_en_reg2	<= tx_en_reg1;
				tx_en_reg3	<= tx_en_reg2;
		end
		
			  
always @ (posedge Clk or negedge nrst)
    if (~nrst) rx_end_reg <= 1'b0;
	else  rx_end_reg <= crc_chk_eof;
		
		
// 去掉MAC、TYPE后的数据（少了14yte)，此信号调整从数据流中提出数据写入到FIFO的控制信号的起始
always @ (posedge Clk or negedge nrst)
    if (~nrst) rx_da_en_reg <= 1'b0;
	else  if(genCnt == 7'h10) rx_da_en_reg <= 1'b1;
		  else if(Current_state==State_IFG) rx_da_en_reg <= 1'b0;

		
assign rx_data 	= rx_tmp_da; 
assign rx_da_en	= RxDv_reg & rx_da_en_reg;	// 去掉CRC/MAC/TYPE后的Ｌ２的用户数据
assign rx_end	= rx_end_reg;


assign gmiiTxEn		= tx_en_reg3;
assign gmiiTxDa 	= dly_reg3;


//******************************************************************************
// 从命令报文中提出相关信息
//
// 1. init报文中提取 local mac / host_mac_reg 的值
// 2. ctrl/ectrl/edata 提取 last_cmd的值 
// 3. active 报文提取 delay/config
//
//******************************************************************************

// INIT 报文提取local mac / host_mac_reg


// 此处需要特别说明
// 1、由于所有的命令在CRC后有效，所以第一个init报文，是rst_latch有效时产生的。
// 2、若在enLatch下检测，则永远无法同步
// 3、允许复位且收到复位报文时复位

always @ (posedge Clk or negedge nrst)
    if (~nrst)   begin local_mac_reg <=0; host_mac_reg <= 0; end
	else if(rst_p) begin local_mac_reg <=0; host_mac_reg <= 0; end
    else if(init_l_reg && rst_latch && genCnt == 7'hd)begin
			 local_mac_reg 	<= gmiiRxD;
			 host_mac_reg 	<= {rx_tmp_da,gmacdata[23:16]};
			end
			
// ctrl/ectrl/edata 提取 last_cmd的值 
always @ (posedge Clk or negedge nrst)
    if (~nrst)   last_cmd_reg <=0; 
    else if(rst_p) last_cmd_reg <=0;
    else if(ctrl_l_reg|ectrl_l_reg|edata_l_reg)begin
			 if(genCnt == 7'h6) last_cmd_reg <= rx_tmp_da;
			end

// ACTIVE报文提取，delay_reg / start_dn_ch_num;  2021.02.08 增加
// 允许复位且收到复位报文时复位
//reg [7:0]	start_dn_ch_num;
/*
always @ (posedge Clk or negedge nrst)
    if (~nrst)  	begin delay_reg <=16'h0; start_dn_ch_num <= 0; end
	else if(rst_p) 	begin delay_reg <=16'h0; start_dn_ch_num <= 0; end
    else if(active_l_reg & (enLatch))begin
			if(genCnt == 7'h11) delay_reg 		<= gmacdata[15:0];
			if(genCnt == 7'h14) start_dn_ch_num <= gmacdata[23:16];		//2021.02.08 增加
			end	
*/
reg		lts_reg;
always @ (posedge Clk or negedge nrst)
    if (~nrst)  delay_reg <=0;
    else if(genCnt == 7'h10) delay_reg 		<= gmacdata[23:0];
    		
always @ (posedge Clk or negedge nrst)
    if (~nrst)   lts_reg <= 0;
    else if(~RxDv_reg) lts_reg <= 0;
    else if (genCnt == 7'h12) lts_reg <= 1'b1;
   
			
// SYNC 报文提取，conf_reg   20200609
// 允许复位且收到复位报文时复位
reg [1:0]	conf_cnt_reg;		// 用于连续记录几次，若数据都正确才锁存并输出
reg [31:0]	conf_q;
			
always @ (posedge Clk or negedge nrst)
    if (~nrst)  begin 	
    			conf_reg 	<=32'h0; 
    			conf_cnt_reg <= 0;
    			conf_q		<=	0;
    			end
	else if(rst_p)   begin 	
    			conf_reg 	<=32'h0;
    			conf_cnt_reg <= 0;
    			conf_q		<=	0;
    			end
    else if(sync_l_reg & (genCnt == 7'h14))begin
							conf_reg	<= {gmacdata,RxD_reg};
							conf_cnt_reg <= {conf_cnt_reg[0],(conf_reg == {gmacdata,RxD_reg})};
							if(&conf_cnt_reg) conf_q <= conf_reg;
							
			end	


always @ (posedge Clk or negedge nrst)
    if (~nrst) rev_reg <= 40'h0;
    else if(sync_l_reg & (genCnt == 7'h19)) rev_reg <= {dly_reg3,dly_reg2,dly_reg1,dly_reg0,RxD_reg};

// 20200609 用于改变SYNC报文中的TTL
always @ (posedge Clk or negedge nrst)
    if (~nrst) insert_device_cnt <= 0;
    else insert_device_cnt <= sync_l_reg & (genCnt == 7'h1a);

// 20200609 提取DNM数据，only for DNM
reg [7:0]		dnm_num_reg;
reg [1:0]		dnm_cnt;
reg [7:0]		dnm_q;
always @ (posedge Clk or negedge nrst)
    if (~nrst) begin 
    			dnm_num_reg <= 0;
    			dnm_cnt 	<= 0;
    			dnm_q		<= 0;
    		   end
    else if(rst_p) begin 
    			dnm_num_reg <= 0;
    			dnm_cnt 	<= 0;
    			dnm_q		<= 0;
    		   end
 	 else if(status_l_reg & (genCnt == 7'hc))begin
							dnm_num_reg	<= RxD_reg;
							dnm_cnt 	<= {dnm_cnt[0],(dnm_num_reg == RxD_reg)};
							if(&dnm_cnt) dnm_q <= dnm_num_reg;
							
			end	


// 20200428 修改
// 1、改成了16位，原8位
// 2、输出两个标志，mfc_l/mfc_h;

 reg mfCntl_reg ;
 reg mfCnth_reg ;
always @ (posedge Clk or negedge nrst)
    if (~nrst) begin mfCntl_reg <= 0; mfCnth_reg <= 0; end
    // mfc低位数据在前，高位数据在后
    else if(sync_l_reg & (genCnt == 7'd14)) begin mfCntl_reg <= &gmacdata[15:8];mfCnth_reg <= &gmacdata[7:0]; end 		


assign rxLocalMac	= local_mac_reg;
assign rxHostMac	= host_mac_reg;
assign rxLastCmd 	= last_cmd_reg;
assign rxDelayTime	= delay_reg;	
assign rxCfgValue 	= conf_q;
assign rxRev 		= rev_reg;
assign mfc 			= {mfCnth_reg,mfCntl_reg};

assign star_chNo	= 0;//tart_dn_ch_num;		// 2021.0208 

assign rxDnmNum		= dnm_q;

assign localIndexAviable = dnm_q == local_mac_reg;


//******************************************************************************
//   CRC 产生 ,用于TxDa/TxEn的数据的FCS                                                         
//******************************************************************************
 reg [2:0]	crc_en_cnt;
 wire		crc_gen_en;
 
// 启动CRC_gen模块输出数据
always @ (posedge Clk or negedge nrst)
    if (~nrst) crc_en_cnt <= 3'h7;
	else  if(crc_gen_en) crc_en_cnt <= 3'h0;
		  else if(!crc_en_cnt[2]) crc_en_cnt <= crc_en_cnt + 1'b1;

assign  crc_gen_enable 	= ~crc_en_cnt[2] & ~crc_gen_en;		// CRC 数据输出允许
assign 	crc_gen_en 		= dly_en_reg2 & gmiiRxDv;			// 此信号与dly_reg2相配

CRC_gen rxm_crc_gen(
.Reset                    (~nrst),
.Clk                      (Clk),
.Init                     (rx_sfd_reg),	// 此对应 dly_reg0的SFD
.Frame_data               (dly_reg2),   		// 使用合成后的数据
.Data_en                  (crc_gen_en), 
.CRC_rd                   (crc_gen_enable),
.CRC_out                  (crc_gen_daout),		// output  [7:0]
.CRC_end                  ()					//此数据暂时没用 
);
	
//******************************************************************************
//   CRC 检查，检查输入RxD/RxDv的数据的有效性                                                           
//******************************************************************************

wire			crc_chk_err;
	
CRC_chk rxm_crc_chk(
.Reset                      (~nrst), 
.Clk                        (Clk),
.CRC_init                   (rx_sfd_reg),	// 此对应 RxD_reg的SFD
.CRC_data                   (RxD_reg),			// 使用输入的原始数据
.CRC_en                     (rx_data_en),		// 对应RxD_reg的L2数据 

 //From CPU  
.CRC_chk_en                 (crc_chk_eof),		// 在此检查点输出 CRC_err 信号
.CRC_err                    (crc_chk_err),		// 逻辑输出，当前没用
.CRC_err_l					(crc_chk_err_l)		// One clock delay compared to CRC_err
);   


// 以下信号是为了提高速度而增加的reg变量，此信号为电平信号
always @ (posedge Clk or negedge nrst)
    if (~nrst) crc_chk_err_level_reg <= 0;
	else if(crc_chk_err_l) crc_chk_err_level_reg <= 1'b1;
	     else if(Current_state==State_idle) crc_chk_err_level_reg <= 1'b0;
			 
//******************************************************************************
//
//   维护管理模块
//
//	1. latch_packet_all_cnt 	接收到的报文数量计数器
//	2. latch_packet_all_crc_err_cnt 	接收到的报文C错误RC数量计数器
//   
//******************************************************************************	


reg [15:0]		packet_all_cnt_reg;
reg [15:0]		latch_packet_all_cnt;
reg	[1:0]		one_second_edge;

always @ (posedge Clk or negedge nrst)
    if (!nrst) one_second_edge <= 0;
    else one_second_edge <= {one_second_edge[0],one_second_l};

//wire one_second_e = ~one_second_edge[0] & one_second_edge[1];	// 下降沿

reg 	one_second_e;
always @ (posedge Clk or negedge nrst)
    if (!nrst) one_second_e <= 0;
    else one_second_e <=  ~one_second_edge[0] & one_second_edge[1];	


// 接收的全部报文计数
always @ (posedge Clk or negedge nrst)
    if (~nrst) packet_all_cnt_reg <= 0;
    else if(one_second_e) packet_all_cnt_reg <= 0;
    else if(Current_state==State_SFD) packet_all_cnt_reg <= packet_all_cnt_reg + 1'b1;

always @ (posedge Clk or negedge nrst)
    if (~nrst) latch_packet_all_cnt <= 0;
    else if(one_second_e) latch_packet_all_cnt <= packet_all_cnt_reg;


// 接收的全部报文CRC校验

reg [15:0]		packet_all_crc_err_cnt_reg;
reg [15:0]		latch_packet_all_crc_err_cnt;
reg				led_crc_reg;


always @ (posedge Clk or negedge nrst)
    if (~nrst) packet_all_crc_err_cnt_reg <= 0;
    else if(one_second_e) packet_all_crc_err_cnt_reg <= 0;
    else if(crc_chk_err_l) packet_all_crc_err_cnt_reg <= packet_all_crc_err_cnt_reg + 1'b1;

always @ (posedge Clk or negedge nrst)
    if (~nrst) begin latch_packet_all_crc_err_cnt <= 0; led_crc_reg <= 0; end
    else if(one_second_e) begin 
    							latch_packet_all_crc_err_cnt <= packet_all_crc_err_cnt_reg;  
    							led_crc_reg <=  |packet_all_crc_err_cnt_reg; 
    						end


assign rxPacketNum			= latch_packet_all_cnt;	
assign rxPacketCrcErrNum	= latch_packet_all_crc_err_cnt;
assign led_crc 				= led_crc_reg;

//****************************************************************************** 
//   MIB 
//
//   1、rxDv 			用于统计收到的全部报文数量
//   2、pkt_ok_reg		用于统计正确的报文的数量,（不包括FCS错及RAM、FIFO溢出造成的丢包）
//   
//****************************************************************************** 

reg		pkt_ok_reg;
always @ (posedge Clk or negedge nrst)
    if (~nrst) pkt_ok_reg <= 1'b0;
    else if(Current_state==State_SFD)  pkt_ok_reg <= 1'b0;
	else if(rx_end_reg)  pkt_ok_reg <= ~crc_chk_err_l;


assign mib_da2 = {pkt_ok_reg,RxDv_reg};


assign lts = lts_reg;
endmodule            
                                                
