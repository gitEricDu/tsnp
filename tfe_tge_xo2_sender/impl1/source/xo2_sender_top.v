 // 2022。03。30 
 // 当发出SV报文时，外部了拨位开关决定APPID、发送MAC的值
 //
 // 当启动SV时，除 D[13:8] D5/D2 外，其它选择开关，都无效，全部三个网口都发出SV报文
 //
 // PORT A/B 用于 POE供电及同步，在所有的模块中，需要的一个MASTER，由D2=0选择，其它的是SLAVE，D2=1
 //
 // PORT C，发出 SV 信号，速度(PPS) 4K * 1.005 即比正常提高了 0.5%
 // 报文长度 332 （包括FCS），相当于 10.88Mbps （包括同步，不包括报文间隙） 
 // 发送报文时长27.20us(FE)，同期为250us 占比10.88%


// 2022。0128
// 每秒钟发送 1k/3k/6k/ 个报文       
//  
//	1、sw_data[7:0] 对应开关 sw1, sw_data[7] 对应PIN8，sw_data[0] 对应PIN1。
//
//			D[7:6]		报文发送速度选择
//			  00  		1000pps
//			  01		6000pps  
//			  10		3000pps
//			  11		400pps
//
//			D[5]		数码管显示内容选择
//			  0			接收到的正确报文	  
//			  1			接收到的全部报文
//
//			D[4]		报文传输时延的数据源选择
//			  0			FE接收到的报文的延时数据
//			  1			GE接收到的报文的延时数据
//
//			D[3]		复位错误计数器 err_cnt_tge/err_cnt_tfe
//			  0->1		上升沿的动作，复位
//
//			D[2]		同步选择 1：同步 0：不同步  only fot SV

// 
//	2、sw_data[15:8] 对应开关 sw5, sw_data[15] 对应PIN8，sw_data[8] 对应PIN1。
//
//			 D[15]		REG/RAM 选择
//			  0			报文长度 60 byte  ，将内部的 三个 64位的寄存器的数据发出，带时间戳，用于报文的时延测试
//			  1			报文长度 1532(0x5fc) byte  ，将ROM中的数据发出，无时间戳。不能用于报文时延测试
//
//			 D[14]		PORT B 输出报文选择，当SV_PACKET_SENDER有效时，此选择无效
//			  0			报文长度 60 byte  ，将内部的 三个 64位的寄存器的数据发出，用于监控
//			  1			报文长度 1024 byte  ，将报文传输延时数据发出，用于获取传输时延数据
//
//			  
//			 D[13:11]		APPID (fe:400x  ge:401x) only for SV
//			  
//			 D[10:8]		SMAC (fe:120x  ge:130x) only for SV



/*
 * gTxA,gTxB 发出同样的SV报文，报文长度328 + 4（FCS）,PPS = 4020，, 线速度，11.320Mbps
 * 宏定义，一定放到文档的开始，否则，会出现没定义的状态，即使，你已定义，但放的位置有问题
 * SV期间，geB用于同步，不用于正常工作
*/

// `define SV_PACKET_SENDER

/*
 * 此定义只对FE端口的报文有效，GE不受影响
 * 发出GOOSE报文，用于压力提供，只SV_PACKET_SENDER的效后，此宏才有效
 * 报文长度534 + 4（FCS）,PPS = 16080, 线速度，71.168Mbps
*/

// `define GOOSE_PACKET_network_pressure_Sender	



`ifdef SV_PACKET_SENDER
	`ifdef GOOSE_PACKET_network_pressure_Sender
 		`define PACKET_LENGTH				11'd532		// 正常报文，不包括FCS
 	`else
 		`define PACKET_LENGTH				11'd326		// 正常报文，不包括FCS
 	`endif		
`else

	// 当选择RAM时，此值才有效，即selectRomDa = H时，发出1528 + 4（FCS）长的报文
	// 当选择 REG时，此时，发出60byte的报文，带时间戳，用于计时测试
	// 本报文手于测试 1514长度的数据，是否可能通过tfe端口
	
 	`define PACKET_LENGTH				11'd1498		// 正常报文，不包括FCS，不包括MAC、Type ,1498输出的报文长度为 1518 （包括FCS）
	
 `endif

// 发送模块使用
`include "../../comm_verilog/net/tx_ram_2_mii_fe.v" 	// 用于固定发送 FE 接口的报文
`include "../../comm_verilog/net/tx_ram_2_mii_ge.v" 	// 用于固定发送 GE 接口的报文
`include "../../core/xo2/rom/xo2_rom_1k_8bit.v"			// 包括  rom_1k_8bit.mem
`include "../../comm_verilog/misc/event_record.v"

// 公用
`include "../../comm_verilog/net/CRC_chk.v"
`include "../../comm_verilog/net/CRC_gen.v"
`include "../../comm_verilog/user_phy/mii_ctrl_8522.v"
`include "../../comm_verilog/misc/switch.v"
`include "../../comm_verilog/user_mib/mib_cnt.v"
`include "../../comm_verilog/rgmii_gmii/rgmii_gmii.v" 

`include "../impl1/source/sender_timer.v"	 	// 产生速率及秒信号
`include "../impl1/source/disp_88_sender.v"	 
`include "../impl1/source/rxm_v2.3_sender.v"	
`include "../impl1/source/tfe_rxm_sys_sender.v"	
`include "../impl1/source/chk_pkt_cnt.v"


// regmii interface       
module xo2_sender_top (        

input           nrst, 		// 系统复位，低有效         
input           sysclk,  	// 系统时钟 125Mhz 
input			exSync,		// 外部同步信号
  
// port A RGMII
input			RxClkA,  
input [3:0]		RxDA,			//     
input			RxCtlA,			// 
input			linkA,			//  
	 
output			TxClkA,  
output [3:0]	TxDA,
output 			TxCtlA,
input			linkB,  
// port B RGMII  
input			RxClkB,
input [3:0]		RxDB,			//  
input			RxCtlB,			// 
	 
output			TxClkB, 
output [3:0]	TxDB,
output 			TxCtlB,

// fe
input			fe_rxClk,
input			fe_rxDv,
input [3:0]		fe_rxDa,

input			fe_txClk,
output			fe_txEn,
output [3:0]	fe_txDa,

output			fe_rst,

// phy
output			mck,
output			mdio,
output			nphy_rst,		// oc output
output			phy_clk,		// diff clk

// indicator
output			led_run,	// H：亮，L：暗
output			led_alm,
output			sfp0_led,
output			sfp1_led,

output			led_d13,		//备用
output			led_d14,		// 报文内容显示，亮选择RAM
output			led_d15,		// 数码管内容选择，亮，显示接收到的全部报文

// key input
output			sw_load,
output			sw_clk,
input			sw_di,

// 数码管显示控制接口
output			tst_sdo, 		// output
output			tst_sclk,		// output 
output			tst_latch,		// output

output [5:0]	tsto
);

wire		gRxDvA;
wire		gRxErrA;
wire		gTxEnA;
wire [7:0]	gRxDA;
wire [7:0]	gTxDA;
wire		gTxClkA;
wire		gRxClkA;

wire [7:0]	gTxDB;
wire		gTxClkB;
wire		gTxEnB;

wire		clk2m;

wire		gRxDvB;
wire		gRxErrB;
wire		gRxClkB;
wire [7:0]	gRxDB;
wire 		one_second_l;
wire [15:0]	sw_data;
reg			spd_da;			// 发送报文的速率 
wire		spd_400;


assign phy_clk 	= sysclk;
assign fe_rst 	= nrst;

// H : 亮
assign led_d13	= sw_data[4];	// 端口选择，亮,选择GE，否则FE
assign led_d14	= sw_data[2];	// H：同步，L：不同步
assign led_d15	= sw_data[5];	// 数码管内容选择，亮，显示接收到的全部报文


rgmii2gmii rgmiiA (.nrst(nrst),
	
	// RGMII   input													output
	.RxClk(RxClkA),.RxD(RxDA),.RxCtl(RxCtlA),							.TxClk(TxClkA),.TxD(TxDA),.TxCtl(TxCtlA),	//output
	
	//GMII   output														input
	.gRxClk(gRxClkA),.gRxD(gRxDA),.gRxDv(gRxDvA),.gRxErr(gRxErrA),		.gTxClk(gTxClkA),.gTxD(gTxDA),.gTxEn(gTxEnA),.gTxErr(1'b0));
	

rgmii2gmii rgmiiB (.nrst(nrst),			// 用于接收来自大MAM的同步信号，同步本板卡，同时发出测试报文

	// RGMII   input												output
	.RxClk(RxClkB),.RxD(RxDB),.RxCtl(RxCtlB),						.TxClk(TxClkB),.TxD(TxDB),.TxCtl(TxCtlB),		
		
`ifdef SV_PACKET_SENDER

	//GMII   output														input
	.gRxClk(gRxClkB),.gRxD(gRxDB),.gRxDv(gRxDvB),.gRxErr(gRxErrB),	.gTxClk(gTxClkA),.gTxD(gTxDA),.gTxEn(gTxEnA),.gTxErr(1'b0));
`else
	.gRxClk(gRxClkB),.gRxD(gRxDB),.gRxDv(gRxDvB),.gRxErr(gRxErrB),	.gTxClk(gTxClkB),.gTxD(gTxDB),.gTxEn(gTxEnB),.gTxErr(1'b0));

`endif


/****************** TX module   ***********************************************
*
*		发出 60 或 1K 的 报文
*
*
**********************************************************************************/
wire [11:0]		ge_ram_addr;
wire [7:0]		ge_ram_da;
wire 			ge_ram_da_en;

wire [11:0]		fe_ram_addr;
wire [7:0]		fe_ram_da;
wire 			fe_ram_da_en;
wire [63:0]		evt_cnt;
wire [23:0]		free_cnt;
wire			spd_4k;
wire			spd_16k;
wire			spd_32k;
wire			spd_64k;

//		2、sw_data[15:8] 对应开关 sw5, sw_data[15] 对应PIN8，sw_data[8] 对应PIN1。

wire	selectRomDa	= sw_data[15];

wire [15:0]		pcnt;

wire tri_sv = sw_data[2] ? gRxDvA : spd_4k;		// 同步选择


`ifdef SV_PACKET_SENDER

wire g_ram_rom 		= 1'b0;				// H: RAM  L: Reg
wire g_ex_tri_en 	= 1'b1;				// H: 外部触发发送报文    		 L：自动发送报文 1000pps
wire g_ex_tri 		= tri_sv;			// 外部触发信号，高有电效，电平，上升沿触发

wire f_ram_rom 		= 1'b0;				// H: RAM  L: Reg
wire f_ex_tri_en 	= 1'b1;				// H: 外部触发发送报文    		 L：自动发送报文 1000pps

`ifdef GOOSE_PACKET_network_pressure_Sender
wire f_ex_tri 		= spd_16k;			// 外部触发信号，高有电效，电平，上升沿触发
`else
wire f_ex_tri 		= tri_sv;			// 外部触发信号，高有电效，电平，上升沿触发
`endif

`else

wire g_ram_rom 		= selectRomDa;		// H: RAM  L: Reg
wire g_ex_tri_en 	= 1'b1;				// H: 外部触发发送报文    		 L：自动发送报文 1000pps
wire g_ex_tri 		= spd_da;			// 外部触发信号，高有电效，电平，上升沿触发

wire f_ram_rom 		= selectRomDa;		// H: RAM  L: Reg
wire f_ex_tri_en 	= 1'b1;				// H: 外部触发发送报文    		 L：自动发送报文 1000pps
wire f_ex_tri 		= spd_da;			// 外部触发信号，高有电效，电平，上升沿触发

`endif


tx_ram_mii_ge tx_ge(  .nrst(nrst), .sclk(sysclk),	 
 .ram_or_reg	(g_ram_rom),	// H: RAM  L: Reg
 .disAutoTri	(g_ex_tri_en),	// H: 外部触发发送报文    		 L：自动发送报文 1000pps
 .exTri			(g_ex_tri),		// 外部触发信号，高有电效，电平，上升沿触发
 .pcnt			(pcnt),			// 发出的报文的计数器，通过tfe/tge发出的报文一致
 
 .fTxBus(),								// 定义 TX_RAM_IS_FE_TYPE 才支持 FE，否则支持 GE
 .gTxBus({gTxClkA,gTxEnA,gTxDA}),
 .ramAddr(ge_ram_addr),.enDataRam(ge_ram_da_en),.dataRamBus(ge_ram_da),		// RAM inf
 .mac1(evt_cnt),.mac2(64'h0),.tx_status({free_cnt,40'h0})					// input [63:0] status
, .mac_appid(sw_data[13:8])													// input [5:0]
 );					

  tx_ram_mii_fe tx_fe(  .nrst(nrst), .sclk(fe_txClk),	
 .ram_or_reg	(selectRomDa),	// H: RAM  L: Reg
 .disAutoTri	(f_ex_tri_en),	// H: 外部触发发送报文    		 L：自动发送报文 1000pps
 .exTri			(f_ex_tri),		// 外部触发信号，高有电效，电平，上升沿触发
 .pcnt			(pcnt),			// 发出的报文的计数器，通过tfe/tge发出的报文一致

 .fTxBus({fe_txEn,fe_txDa}),					// 定义 TX_RAM_IS_FE_TYPE 才支持 FE，否则支持 GE
 .gTxBus(),
 .ramAddr(fe_ram_addr),.enDataRam(fe_ram_da_en),.dataRamBus(fe_ram_da),		// RAM inf
 .mac1(evt_cnt),.mac2(64'h0),.tx_status({free_cnt,40'h0})					// input [63:0] status
, .mac_appid(sw_data[13:8])													// input [5:0]
 );					


// ROM 
xo2_rom_1k_8bit ge_rom (.Address(ge_ram_addr[9:0]), .OutClock(sysclk),  .OutClockEn(ge_ram_da_en), .Reset(~nrst),  .Q(ge_ram_da));
xo2_rom_1k_8bit fe_rom (.Address(fe_ram_addr[9:0]), .OutClock(fe_txClk),.OutClockEn(fe_ram_da_en), .Reset(~nrst),  .Q(fe_ram_da));

// 用于状态检测，每秒钟发出一个值
wire [11:0]		t_ram_addr;
wire [7:0]		t_ram_da;
wire 			t_ram_en;
wire			send_tst_pkt;

wire selectCont	= sw_data[14];
tx_ram_mii_ge tx_ge_o(  .nrst(nrst), .sclk(sysclk),	 
.ram_or_reg		(selectCont),	// H: RAM  L: Reg
.disAutoTri		(1'b1),			// H: 外部触发发送报文    		 L：自动发送报文 1000pps
.exTri			(send_tst_pkt),	// 外部触发信号，高有电效，电平，上升沿触发

.pcnt			(pcnt),			// 发出的报文的计数器，通过tfe/tge发出的报文一致

.fTxBus(),								// 定义 TX_RAM_IS_FE_TYPE 才支持 FE，否则支持 GE
.gTxBus({gTxClkB,gTxEnB,gTxDB}),
.ramAddr(t_ram_addr),.enDataRam(t_ram_en),.dataRamBus(t_ram_da),			// RAM inf
.mac1(evt_cnt),.mac2(64'h0),.tx_status({46'h0,linkB,linkA,sw_data})			// input [63:0] status
, .mac_appid(sw_data[13:8])													// input [5:0]

);			

/****************** RX module   ***********************************************
*
*		接收正确的报文计数
*  
*
**********************************************************************************/
wire [1:0]		ge_rx_mib;
wire [23:0]		grx_timestamp;		// ref gRxClkA
wire			glEn;				// ref gRxClkA
wire 			chk_pkt_cnt_tge;	// port A 接收到
wire [15:0]		pkt_cnt_tge;		// port A 接收到
wire [15:0]		sv_cnt_tge;			// port A 接收到

// 接收 PORT A的数据
 rxm_v23_sender ge_rxm(.nrst(nrst),.enLatch(1'b1),.rst_latch(1'b1),.rst_p(1'b0),.one_second_l(1'b0),

.gmiiClk(gRxClkA),.gmiiRxDv(gRxDvA),.gmiiRxD(gRxDA),.gmiiRxErr(1'b0),.gmiiTxEn(),.gmiiTxDa(),  // 使用gmiiRxClk时钟  
.rx_data(),.rx_end(),.rx_da_en(),.rxCrcErr(),

// ***************  ADD/DROP *********************************
.dropDaAddr(16'h0),.dropDaEn(),.dropDa(),  .addDaAddr(16'h0),.addDaEn(),.addDa(8'h0),
.dropStAddr(16'h0),.dropStEn(),.dropStDa(),.addStAddr(16'h0),.addStEn(),.addStDa(8'h0),

// ***************  ADD/DROP END  *********************************
// 以下信号是控制用
.findReset_l(),.findInit_l(),.findActive_l(),.findDevice_l(),.findDeviceEnd_l(),.findSync_l(),.findCtrl_l(),.findEctrl_l(),
.findEdata_l(),.findData_l(),.findStstus_l(),.findUnknow_l(),
.rxHostMac(),.rxLocalMac(),.rxLastCmd(),.rxCfgValue(),.rxRev(),.mfc(),.star_chNo(),.localMac(8'he),
.led_crc(),.rxPacketNum(),.rxPacketCrcErrNum(),.rxDnmNum(),.localIndexAviable(),

.rxSfd(),
.rxDelayTime(grx_timestamp),
.mib_da2(ge_rx_mib)		// output[1:0] {pkt_ok_reg,RxDv_reg}; 用于正确的报文、全部报文的计数处理
,.lts(glEn)				// 电平

,.chk_pkt_cnt_l(chk_pkt_cnt_tge)	// output 发现tfe发出的报文，CRC正确后有效，电平 
,.pkt_cnt(pkt_cnt_tge)				// output[15:0] 发现tfe发出的报文的计数值
,.sv_cnt(sv_cnt_tge)

);

wire [4:0]		fe_rx_mib;
wire [23:0]		frx_timestamp;		// ref sysclk
wire			flEn;				// ref sysclk
wire 			chk_pkt_cnt_tfe;	// port A 接收到
wire [15:0]		pkt_cnt_tfe;		// port A 接收到
wire [15:0]		sv_cnt_tfe;			// port A 接收到

// 接收FE接口的数据
tfe_rxm_sys_sender fe_rxm (.nrst(nrst),.sysclk(sysclk),.rxClk(fe_rxClk),.rxDv(fe_rxDv),.rxErr(1'b0),.rxDa(fe_rxDa),
.ramAddr(),.ramClkEn(),.ramWrDa(),.ramRdBlk(5'h0),.ramFull(),.ramEmpty(),.full(1'b0),.f_wrEn(),.f_wrDa(),.rxm_tst(),.dMAC(),.sMAC(),

.mib_da5	(fe_rx_mib)			// {full,ram_full_reg,drop_pkt_reg,pkt_ok_reg,sys_rxDv[0]};
,.rxtimestamp(frx_timestamp),.lts(flEn)

,.chk_pkt_cnt_l(chk_pkt_cnt_tfe)			// output 发现tge发出的报文，CRC正确后有效，电平 
,.pkt_cnt(pkt_cnt_tfe)						// output[15:0] 发现tge发出的报文的计数值
,.sv_cnt(sv_cnt_tfe)
);


wire [15:0]		err_cnt_tge;
wire [15:0]		err_cnt_tfe;


chk_pkt_cnt cpc  (.nrst(nrst),.sysclk(sysclk),.tx_tri(spd_da),.pcnt(pcnt),.sec_l(one_second_l),

.rst_err_cnt(sw_data[3]),

`ifdef SV_PACKET_SENDER
.cpcl_tge(gRxDvA),	.pkt_cnt_tge(sv_cnt_tge),		// input/input[15:0]
.cpcl_tfe(fe_rxDv),	.pkt_cnt_tfe(sv_cnt_tfe),		// input/input[15:0]

`else
.cpcl_tge(chk_pkt_cnt_tge),.pkt_cnt_tge(pkt_cnt_tge),		// input/input[15:0]
.cpcl_tfe(chk_pkt_cnt_tfe),.pkt_cnt_tfe(pkt_cnt_tfe),		// input/input[15:0]
`endif

.errCnt_tge(err_cnt_tge),.errCnt_tfe(err_cnt_tfe)	// output [15:0]/output [15:0]

);

/****************** 定时模块   ******************************************************
*
*		1、选择报文发出的速度
*		2、产生秒信号
*
**********************************************************************************/

// 输出的都是电平信号
// sec 秒信号占空比不是1：1

wire 		spd_6000;
wire 		spd_3000;
wire 		spd_1000;
wire 		sec_p;

sender_timer st (.nrst(nrst),.sysclk(sysclk),.exSync(exSync),

.tri_6k(spd_6000),.tri_3k(spd_3000),.tri_1k(spd_1000),.sec(one_second_l),.sec_p(sec_p),.spd_4k(spd_4k),.spd_16k(spd_16k),.spd_32k(spd_32k),.spd_64k(spd_64k),.spd_400(spd_400));

//		1、sw_data[7:0] 对应开关 sw1, sw_data[7] 对应PIN8，sw_data[0] 对应PIN1。

always @(*)
  case (sw_data[7:6])
    2'h1:	spd_da <= spd_6000;
    2'h2:	spd_da <= spd_3000;
    2'h3:	spd_da <= spd_400;
    default:spd_da <= spd_1000; 
  endcase


/****************** 数据统计     ******************************************************
*
*	1、ge-rx 接收到的正确的报文数量	
*	2、ge-rx 接收到的总报文数量
*	3、fe-rx 接收到的正确的报文数量	
*	4、fe-rx 接收到的总报文数量
*
**********************************************************************************/
 
mib_cnt mc_ge_ok   		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(ge_rx_mib[1]), .evtCnt(evt_cnt[63:48]));  
mib_cnt mc_ge_all 		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(gTxEnA), .evtCnt(evt_cnt[47:32]));  

mib_cnt mc_fe_ok   		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(fe_rx_mib[1]), .evtCnt(evt_cnt[31:16]));  
mib_cnt mc_fe_all 		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(fe_txEn), .evtCnt(evt_cnt[15:0])); 


/****************** 低速内部时钟     ***********************************************
*
*		利用内部的振荡器，产生低速管理用时钟
*
*
**********************************************************************************/

// 内部时钟 for XO2

defparam OSCH_inst.NOM_FREQ = "2.08";
OSCH OSCH_inst( .STDBY(1'b0),.OSC(clk2m),.SEDSTDBY());	// STDBY = 0 enable

mii_8522 mii_8522( .nrst(nrst),	.clk2m(clk2m),.mdio(mdio),.mck(mck),.nrstPhy(nphy_rst));


/****************** 2位数码管显示 & 键盘扫描           **************************************
*
*			全部的数码管同步显示
*
**********************************************************************************/
wire [2:0]		disp_cnt;
reg [7:0]		disp_da;

always @(*)
  case ({sw_data[5],disp_cnt[1:0]})

    	3'h0: disp_da <= evt_cnt[63:56];
  		3'h1: disp_da <= evt_cnt[55:48];
  		3'h2: disp_da <= evt_cnt[31:24];
  		3'h3: disp_da <= evt_cnt[23:16];
  		
`ifdef SV_PACKET_SENDER

  		//  for SV :           DMAC & APPID	           SMAC addr
  		3'h4: disp_da <= {1'b0,sw_data[13:11],1'b0,sw_data[10:8]};
  		3'h5: disp_da <= {1'b0,sw_data[13:11],1'b0,sw_data[10:8]};
  		3'h6: disp_da <= {1'b0,sw_data[13:11],1'b0,sw_data[10:8]};
  		3'h7: disp_da <= {1'b0,sw_data[13:11],1'b0,sw_data[10:8]};
  		
`else
  		// error packet
  		3'h4: disp_da <= err_cnt_tge[15:8];
  		3'h5: disp_da <= err_cnt_tge[7:0];
  		3'h6: disp_da <= err_cnt_tfe[15:8];
  		3'h7: disp_da <= err_cnt_tfe[7:0];
`endif
  		default: disp_da <= 0;
  		
  endcase

// dainA 是个位的数据，dainB是十位的数据
`ifdef SV_PACKET_SENDER
wire [1:0] dpoint = sw_data[5] ? 2'h0 : disp_cnt[1:0];		//关掉小数点的显示
`else
wire [1:0] dpoint = disp_cnt[1:0];

`endif

disp_88_sender d88(.nrst(nrst),.clk2m(clk2m),.dain(disp_da),.addr(disp_cnt),.sdo(tst_sdo),.sclk(tst_sclk),.latch(tst_latch),.scanp(),.dp(dpoint));

wire		sw_ld;
assign sw_load = ~sw_ld;		// L: load for 74hc165

//  swith 数据格式说明 【BOARD_IS_DNM_DEMO_XO2】
//		1、sw_data[7:0] 对应开关 sw1, sw_data[7] 对应PIN8，sw_data[0] 对应PIN1。
//		2、sw_data[15:8] 对应开关 sw5, sw_data[15] 对应PIN8，sw_data[8] 对应PIN1。

switch sw( .nrst(nrst),	.clk2m(clk2m),.load(sw_ld),.sdi(sw_di),.sclk(sw_clk),.sw(sw_data),.latch());

		   
// H： 亮 L：暗
assign 		led_run  	= one_second_l;
assign 		led_alm		= 0;
assign 		sfp0_led 	= linkA;
assign 		sfp1_led 	= linkB;

/****************** 传输时间统计   **********************************************************
*
*  对于具备时间戳的报文，可以统计出传输时间，并存贮在RAM内，可以由外部的模块读出并分析
*
******************************************************************************************/

//			D[4]		报文传输时延的数据源选择
//			  0			FE接收到的报文的延时数据
//			  1			GE接收到的报文的延时数据

wire		txTimeEn 	= sw_data[4] ? glEn : flEn;
wire [23:0]	txTime 		= sw_data[4] ? grx_timestamp : frx_timestamp;
wire		rxDv 		= sw_data[4] ? gRxDvA : fe_rxDv;

event_record er (.nrst(nrst),.sysclk(sysclk),
			.lts(spd_da),.tso(free_cnt),												// in/out[23:0]
			.txTimeEn(txTimeEn),.txTime(txTime),.rxDv(rxDv),							// in/in[23:0]/in  触发环境
			.rdRamAddr(t_ram_addr[9:0]),.rdRamClkEn(t_ram_en),.rdRamDa(t_ram_da),		// in[9:0]/in/out[7:0]  RAM inf
			.tri_l(send_tst_pkt),.tsto()												// out 上升沿，表示RAM已写满
			);
       
assign tsto = {spd_64k,spd_16k,spd_4k,spd_6000,spd_3000,spd_1000,gRxDvA,gTxEnA};

endmodule

