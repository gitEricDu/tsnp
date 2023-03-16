 // 2022��03��30 
 // ������SV����ʱ���ⲿ�˲�λ���ؾ���APPID������MAC��ֵ
 //
 // ������SVʱ���� D[13:8] D5/D2 �⣬����ѡ�񿪹أ�����Ч��ȫ���������ڶ�����SV����
 //
 // PORT A/B ���� POE���缰ͬ���������е�ģ���У���Ҫ��һ��MASTER����D2=0ѡ����������SLAVE��D2=1
 //
 // PORT C������ SV �źţ��ٶ�(PPS) 4K * 1.005 ������������� 0.5%
 // ���ĳ��� 332 ������FCS�����൱�� 10.88Mbps ������ͬ�������������ļ�϶�� 
 // ���ͱ���ʱ��27.20us(FE)��ͬ��Ϊ250us ռ��10.88%


// 2022��0128
// ÿ���ӷ��� 1k/3k/6k/ ������       
//  
//	1��sw_data[7:0] ��Ӧ���� sw1, sw_data[7] ��ӦPIN8��sw_data[0] ��ӦPIN1��
//
//			D[7:6]		���ķ����ٶ�ѡ��
//			  00  		1000pps
//			  01		6000pps  
//			  10		3000pps
//			  11		400pps
//
//			D[5]		�������ʾ����ѡ��
//			  0			���յ�����ȷ����	  
//			  1			���յ���ȫ������
//
//			D[4]		���Ĵ���ʱ�ӵ�����Դѡ��
//			  0			FE���յ��ı��ĵ���ʱ����
//			  1			GE���յ��ı��ĵ���ʱ����
//
//			D[3]		��λ��������� err_cnt_tge/err_cnt_tfe
//			  0->1		�����صĶ�������λ
//
//			D[2]		ͬ��ѡ�� 1��ͬ�� 0����ͬ��  only fot SV

// 
//	2��sw_data[15:8] ��Ӧ���� sw5, sw_data[15] ��ӦPIN8��sw_data[8] ��ӦPIN1��
//
//			 D[15]		REG/RAM ѡ��
//			  0			���ĳ��� 60 byte  �����ڲ��� ���� 64λ�ļĴ��������ݷ�������ʱ��������ڱ��ĵ�ʱ�Ӳ���
//			  1			���ĳ��� 1532(0x5fc) byte  ����ROM�е����ݷ�������ʱ������������ڱ���ʱ�Ӳ���
//
//			 D[14]		PORT B �������ѡ�񣬵�SV_PACKET_SENDER��Чʱ����ѡ����Ч
//			  0			���ĳ��� 60 byte  �����ڲ��� ���� 64λ�ļĴ��������ݷ��������ڼ��
//			  1			���ĳ��� 1024 byte  �������Ĵ�����ʱ���ݷ��������ڻ�ȡ����ʱ������
//
//			  
//			 D[13:11]		APPID (fe:400x  ge:401x) only for SV
//			  
//			 D[10:8]		SMAC (fe:120x  ge:130x) only for SV



/*
 * gTxA,gTxB ����ͬ����SV���ģ����ĳ���328 + 4��FCS��,PPS = 4020��, ���ٶȣ�11.320Mbps
 * �궨�壬һ���ŵ��ĵ��Ŀ�ʼ�����򣬻����û�����״̬����ʹ�����Ѷ��壬���ŵ�λ��������
 * SV�ڼ䣬geB����ͬ������������������
*/

// `define SV_PACKET_SENDER

/*
 * �˶���ֻ��FE�˿ڵı�����Ч��GE����Ӱ��
 * ����GOOSE���ģ�����ѹ���ṩ��ֻSV_PACKET_SENDER��Ч�󣬴˺����Ч
 * ���ĳ���534 + 4��FCS��,PPS = 16080, ���ٶȣ�71.168Mbps
*/

// `define GOOSE_PACKET_network_pressure_Sender	



`ifdef SV_PACKET_SENDER
	`ifdef GOOSE_PACKET_network_pressure_Sender
 		`define PACKET_LENGTH				11'd532		// �������ģ�������FCS
 	`else
 		`define PACKET_LENGTH				11'd326		// �������ģ�������FCS
 	`endif		
`else

	// ��ѡ��RAMʱ����ֵ����Ч����selectRomDa = Hʱ������1528 + 4��FCS�����ı���
	// ��ѡ�� REGʱ����ʱ������60byte�ı��ģ���ʱ��������ڼ�ʱ����
	// ���������ڲ��� 1514���ȵ����ݣ��Ƿ����ͨ��tfe�˿�
	
 	`define PACKET_LENGTH				11'd1498		// �������ģ�������FCS��������MAC��Type ,1498����ı��ĳ���Ϊ 1518 ������FCS��
	
 `endif

// ����ģ��ʹ��
`include "../../comm_verilog/net/tx_ram_2_mii_fe.v" 	// ���ڹ̶����� FE �ӿڵı���
`include "../../comm_verilog/net/tx_ram_2_mii_ge.v" 	// ���ڹ̶����� GE �ӿڵı���
`include "../../core/xo2/rom/xo2_rom_1k_8bit.v"			// ����  rom_1k_8bit.mem
`include "../../comm_verilog/misc/event_record.v"

// ����
`include "../../comm_verilog/net/CRC_chk.v"
`include "../../comm_verilog/net/CRC_gen.v"
`include "../../comm_verilog/user_phy/mii_ctrl_8522.v"
`include "../../comm_verilog/misc/switch.v"
`include "../../comm_verilog/user_mib/mib_cnt.v"
`include "../../comm_verilog/rgmii_gmii/rgmii_gmii.v" 

`include "../impl1/source/sender_timer.v"	 	// �������ʼ����ź�
`include "../impl1/source/disp_88_sender.v"	 
`include "../impl1/source/rxm_v2.3_sender.v"	
`include "../impl1/source/tfe_rxm_sys_sender.v"	
`include "../impl1/source/chk_pkt_cnt.v"


// regmii interface       
module xo2_sender_top (        

input           nrst, 		// ϵͳ��λ������Ч         
input           sysclk,  	// ϵͳʱ�� 125Mhz 
input			exSync,		// �ⲿͬ���ź�
  
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
output			led_run,	// H������L����
output			led_alm,
output			sfp0_led,
output			sfp1_led,

output			led_d13,		//����
output			led_d14,		// ����������ʾ����ѡ��RAM
output			led_d15,		// ���������ѡ��������ʾ���յ���ȫ������

// key input
output			sw_load,
output			sw_clk,
input			sw_di,

// �������ʾ���ƽӿ�
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
reg			spd_da;			// ���ͱ��ĵ����� 
wire		spd_400;


assign phy_clk 	= sysclk;
assign fe_rst 	= nrst;

// H : ��
assign led_d13	= sw_data[4];	// �˿�ѡ����,ѡ��GE������FE
assign led_d14	= sw_data[2];	// H��ͬ����L����ͬ��
assign led_d15	= sw_data[5];	// ���������ѡ��������ʾ���յ���ȫ������


rgmii2gmii rgmiiA (.nrst(nrst),
	
	// RGMII   input													output
	.RxClk(RxClkA),.RxD(RxDA),.RxCtl(RxCtlA),							.TxClk(TxClkA),.TxD(TxDA),.TxCtl(TxCtlA),	//output
	
	//GMII   output														input
	.gRxClk(gRxClkA),.gRxD(gRxDA),.gRxDv(gRxDvA),.gRxErr(gRxErrA),		.gTxClk(gTxClkA),.gTxD(gTxDA),.gTxEn(gTxEnA),.gTxErr(1'b0));
	

rgmii2gmii rgmiiB (.nrst(nrst),			// ���ڽ������Դ�MAM��ͬ���źţ�ͬ�����忨��ͬʱ�������Ա���

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
*		���� 60 �� 1K �� ����
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

//		2��sw_data[15:8] ��Ӧ���� sw5, sw_data[15] ��ӦPIN8��sw_data[8] ��ӦPIN1��

wire	selectRomDa	= sw_data[15];

wire [15:0]		pcnt;

wire tri_sv = sw_data[2] ? gRxDvA : spd_4k;		// ͬ��ѡ��


`ifdef SV_PACKET_SENDER

wire g_ram_rom 		= 1'b0;				// H: RAM  L: Reg
wire g_ex_tri_en 	= 1'b1;				// H: �ⲿ�������ͱ���    		 L���Զ����ͱ��� 1000pps
wire g_ex_tri 		= tri_sv;			// �ⲿ�����źţ����е�Ч����ƽ�������ش���

wire f_ram_rom 		= 1'b0;				// H: RAM  L: Reg
wire f_ex_tri_en 	= 1'b1;				// H: �ⲿ�������ͱ���    		 L���Զ����ͱ��� 1000pps

`ifdef GOOSE_PACKET_network_pressure_Sender
wire f_ex_tri 		= spd_16k;			// �ⲿ�����źţ����е�Ч����ƽ�������ش���
`else
wire f_ex_tri 		= tri_sv;			// �ⲿ�����źţ����е�Ч����ƽ�������ش���
`endif

`else

wire g_ram_rom 		= selectRomDa;		// H: RAM  L: Reg
wire g_ex_tri_en 	= 1'b1;				// H: �ⲿ�������ͱ���    		 L���Զ����ͱ��� 1000pps
wire g_ex_tri 		= spd_da;			// �ⲿ�����źţ����е�Ч����ƽ�������ش���

wire f_ram_rom 		= selectRomDa;		// H: RAM  L: Reg
wire f_ex_tri_en 	= 1'b1;				// H: �ⲿ�������ͱ���    		 L���Զ����ͱ��� 1000pps
wire f_ex_tri 		= spd_da;			// �ⲿ�����źţ����е�Ч����ƽ�������ش���

`endif


tx_ram_mii_ge tx_ge(  .nrst(nrst), .sclk(sysclk),	 
 .ram_or_reg	(g_ram_rom),	// H: RAM  L: Reg
 .disAutoTri	(g_ex_tri_en),	// H: �ⲿ�������ͱ���    		 L���Զ����ͱ��� 1000pps
 .exTri			(g_ex_tri),		// �ⲿ�����źţ����е�Ч����ƽ�������ش���
 .pcnt			(pcnt),			// �����ı��ĵļ�������ͨ��tfe/tge�����ı���һ��
 
 .fTxBus(),								// ���� TX_RAM_IS_FE_TYPE ��֧�� FE������֧�� GE
 .gTxBus({gTxClkA,gTxEnA,gTxDA}),
 .ramAddr(ge_ram_addr),.enDataRam(ge_ram_da_en),.dataRamBus(ge_ram_da),		// RAM inf
 .mac1(evt_cnt),.mac2(64'h0),.tx_status({free_cnt,40'h0})					// input [63:0] status
, .mac_appid(sw_data[13:8])													// input [5:0]
 );					

  tx_ram_mii_fe tx_fe(  .nrst(nrst), .sclk(fe_txClk),	
 .ram_or_reg	(selectRomDa),	// H: RAM  L: Reg
 .disAutoTri	(f_ex_tri_en),	// H: �ⲿ�������ͱ���    		 L���Զ����ͱ��� 1000pps
 .exTri			(f_ex_tri),		// �ⲿ�����źţ����е�Ч����ƽ�������ش���
 .pcnt			(pcnt),			// �����ı��ĵļ�������ͨ��tfe/tge�����ı���һ��

 .fTxBus({fe_txEn,fe_txDa}),					// ���� TX_RAM_IS_FE_TYPE ��֧�� FE������֧�� GE
 .gTxBus(),
 .ramAddr(fe_ram_addr),.enDataRam(fe_ram_da_en),.dataRamBus(fe_ram_da),		// RAM inf
 .mac1(evt_cnt),.mac2(64'h0),.tx_status({free_cnt,40'h0})					// input [63:0] status
, .mac_appid(sw_data[13:8])													// input [5:0]
 );					


// ROM 
xo2_rom_1k_8bit ge_rom (.Address(ge_ram_addr[9:0]), .OutClock(sysclk),  .OutClockEn(ge_ram_da_en), .Reset(~nrst),  .Q(ge_ram_da));
xo2_rom_1k_8bit fe_rom (.Address(fe_ram_addr[9:0]), .OutClock(fe_txClk),.OutClockEn(fe_ram_da_en), .Reset(~nrst),  .Q(fe_ram_da));

// ����״̬��⣬ÿ���ӷ���һ��ֵ
wire [11:0]		t_ram_addr;
wire [7:0]		t_ram_da;
wire 			t_ram_en;
wire			send_tst_pkt;

wire selectCont	= sw_data[14];
tx_ram_mii_ge tx_ge_o(  .nrst(nrst), .sclk(sysclk),	 
.ram_or_reg		(selectCont),	// H: RAM  L: Reg
.disAutoTri		(1'b1),			// H: �ⲿ�������ͱ���    		 L���Զ����ͱ��� 1000pps
.exTri			(send_tst_pkt),	// �ⲿ�����źţ����е�Ч����ƽ�������ش���

.pcnt			(pcnt),			// �����ı��ĵļ�������ͨ��tfe/tge�����ı���һ��

.fTxBus(),								// ���� TX_RAM_IS_FE_TYPE ��֧�� FE������֧�� GE
.gTxBus({gTxClkB,gTxEnB,gTxDB}),
.ramAddr(t_ram_addr),.enDataRam(t_ram_en),.dataRamBus(t_ram_da),			// RAM inf
.mac1(evt_cnt),.mac2(64'h0),.tx_status({46'h0,linkB,linkA,sw_data})			// input [63:0] status
, .mac_appid(sw_data[13:8])													// input [5:0]

);			

/****************** RX module   ***********************************************
*
*		������ȷ�ı��ļ���
*  
*
**********************************************************************************/
wire [1:0]		ge_rx_mib;
wire [23:0]		grx_timestamp;		// ref gRxClkA
wire			glEn;				// ref gRxClkA
wire 			chk_pkt_cnt_tge;	// port A ���յ�
wire [15:0]		pkt_cnt_tge;		// port A ���յ�
wire [15:0]		sv_cnt_tge;			// port A ���յ�

// ���� PORT A������
 rxm_v23_sender ge_rxm(.nrst(nrst),.enLatch(1'b1),.rst_latch(1'b1),.rst_p(1'b0),.one_second_l(1'b0),

.gmiiClk(gRxClkA),.gmiiRxDv(gRxDvA),.gmiiRxD(gRxDA),.gmiiRxErr(1'b0),.gmiiTxEn(),.gmiiTxDa(),  // ʹ��gmiiRxClkʱ��  
.rx_data(),.rx_end(),.rx_da_en(),.rxCrcErr(),

// ***************  ADD/DROP *********************************
.dropDaAddr(16'h0),.dropDaEn(),.dropDa(),  .addDaAddr(16'h0),.addDaEn(),.addDa(8'h0),
.dropStAddr(16'h0),.dropStEn(),.dropStDa(),.addStAddr(16'h0),.addStEn(),.addStDa(8'h0),

// ***************  ADD/DROP END  *********************************
// �����ź��ǿ�����
.findReset_l(),.findInit_l(),.findActive_l(),.findDevice_l(),.findDeviceEnd_l(),.findSync_l(),.findCtrl_l(),.findEctrl_l(),
.findEdata_l(),.findData_l(),.findStstus_l(),.findUnknow_l(),
.rxHostMac(),.rxLocalMac(),.rxLastCmd(),.rxCfgValue(),.rxRev(),.mfc(),.star_chNo(),.localMac(8'he),
.led_crc(),.rxPacketNum(),.rxPacketCrcErrNum(),.rxDnmNum(),.localIndexAviable(),

.rxSfd(),
.rxDelayTime(grx_timestamp),
.mib_da2(ge_rx_mib)		// output[1:0] {pkt_ok_reg,RxDv_reg}; ������ȷ�ı��ġ�ȫ�����ĵļ�������
,.lts(glEn)				// ��ƽ

,.chk_pkt_cnt_l(chk_pkt_cnt_tge)	// output ����tfe�����ı��ģ�CRC��ȷ����Ч����ƽ 
,.pkt_cnt(pkt_cnt_tge)				// output[15:0] ����tfe�����ı��ĵļ���ֵ
,.sv_cnt(sv_cnt_tge)

);

wire [4:0]		fe_rx_mib;
wire [23:0]		frx_timestamp;		// ref sysclk
wire			flEn;				// ref sysclk
wire 			chk_pkt_cnt_tfe;	// port A ���յ�
wire [15:0]		pkt_cnt_tfe;		// port A ���յ�
wire [15:0]		sv_cnt_tfe;			// port A ���յ�

// ����FE�ӿڵ�����
tfe_rxm_sys_sender fe_rxm (.nrst(nrst),.sysclk(sysclk),.rxClk(fe_rxClk),.rxDv(fe_rxDv),.rxErr(1'b0),.rxDa(fe_rxDa),
.ramAddr(),.ramClkEn(),.ramWrDa(),.ramRdBlk(5'h0),.ramFull(),.ramEmpty(),.full(1'b0),.f_wrEn(),.f_wrDa(),.rxm_tst(),.dMAC(),.sMAC(),

.mib_da5	(fe_rx_mib)			// {full,ram_full_reg,drop_pkt_reg,pkt_ok_reg,sys_rxDv[0]};
,.rxtimestamp(frx_timestamp),.lts(flEn)

,.chk_pkt_cnt_l(chk_pkt_cnt_tfe)			// output ����tge�����ı��ģ�CRC��ȷ����Ч����ƽ 
,.pkt_cnt(pkt_cnt_tfe)						// output[15:0] ����tge�����ı��ĵļ���ֵ
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

/****************** ��ʱģ��   ******************************************************
*
*		1��ѡ���ķ������ٶ�
*		2���������ź�
*
**********************************************************************************/

// ����Ķ��ǵ�ƽ�ź�
// sec ���ź�ռ�ձȲ���1��1

wire 		spd_6000;
wire 		spd_3000;
wire 		spd_1000;
wire 		sec_p;

sender_timer st (.nrst(nrst),.sysclk(sysclk),.exSync(exSync),

.tri_6k(spd_6000),.tri_3k(spd_3000),.tri_1k(spd_1000),.sec(one_second_l),.sec_p(sec_p),.spd_4k(spd_4k),.spd_16k(spd_16k),.spd_32k(spd_32k),.spd_64k(spd_64k),.spd_400(spd_400));

//		1��sw_data[7:0] ��Ӧ���� sw1, sw_data[7] ��ӦPIN8��sw_data[0] ��ӦPIN1��

always @(*)
  case (sw_data[7:6])
    2'h1:	spd_da <= spd_6000;
    2'h2:	spd_da <= spd_3000;
    2'h3:	spd_da <= spd_400;
    default:spd_da <= spd_1000; 
  endcase


/****************** ����ͳ��     ******************************************************
*
*	1��ge-rx ���յ�����ȷ�ı�������	
*	2��ge-rx ���յ����ܱ�������
*	3��fe-rx ���յ�����ȷ�ı�������	
*	4��fe-rx ���յ����ܱ�������
*
**********************************************************************************/
 
mib_cnt mc_ge_ok   		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(ge_rx_mib[1]), .evtCnt(evt_cnt[63:48]));  
mib_cnt mc_ge_all 		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(gTxEnA), .evtCnt(evt_cnt[47:32]));  

mib_cnt mc_fe_ok   		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(fe_rx_mib[1]), .evtCnt(evt_cnt[31:16]));  
mib_cnt mc_fe_all 		( .nrst(nrst),.sysclk(sysclk),.sec_p(sec_p),.enent(fe_txEn), .evtCnt(evt_cnt[15:0])); 


/****************** �����ڲ�ʱ��     ***********************************************
*
*		�����ڲ����������������ٹ�����ʱ��
*
*
**********************************************************************************/

// �ڲ�ʱ�� for XO2

defparam OSCH_inst.NOM_FREQ = "2.08";
OSCH OSCH_inst( .STDBY(1'b0),.OSC(clk2m),.SEDSTDBY());	// STDBY = 0 enable

mii_8522 mii_8522( .nrst(nrst),	.clk2m(clk2m),.mdio(mdio),.mck(mck),.nrstPhy(nphy_rst));


/****************** 2λ�������ʾ & ����ɨ��           **************************************
*
*			ȫ���������ͬ����ʾ
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

// dainA �Ǹ�λ�����ݣ�dainB��ʮλ������
`ifdef SV_PACKET_SENDER
wire [1:0] dpoint = sw_data[5] ? 2'h0 : disp_cnt[1:0];		//�ص�С�������ʾ
`else
wire [1:0] dpoint = disp_cnt[1:0];

`endif

disp_88_sender d88(.nrst(nrst),.clk2m(clk2m),.dain(disp_da),.addr(disp_cnt),.sdo(tst_sdo),.sclk(tst_sclk),.latch(tst_latch),.scanp(),.dp(dpoint));

wire		sw_ld;
assign sw_load = ~sw_ld;		// L: load for 74hc165

//  swith ���ݸ�ʽ˵�� ��BOARD_IS_DNM_DEMO_XO2��
//		1��sw_data[7:0] ��Ӧ���� sw1, sw_data[7] ��ӦPIN8��sw_data[0] ��ӦPIN1��
//		2��sw_data[15:8] ��Ӧ���� sw5, sw_data[15] ��ӦPIN8��sw_data[8] ��ӦPIN1��

switch sw( .nrst(nrst),	.clk2m(clk2m),.load(sw_ld),.sdi(sw_di),.sclk(sw_clk),.sw(sw_data),.latch());

		   
// H�� �� L����
assign 		led_run  	= one_second_l;
assign 		led_alm		= 0;
assign 		sfp0_led 	= linkA;
assign 		sfp1_led 	= linkB;

/****************** ����ʱ��ͳ��   **********************************************************
*
*  ���ھ߱�ʱ����ı��ģ�����ͳ�Ƴ�����ʱ�䣬��������RAM�ڣ��������ⲿ��ģ�����������
*
******************************************************************************************/

//			D[4]		���Ĵ���ʱ�ӵ�����Դѡ��
//			  0			FE���յ��ı��ĵ���ʱ����
//			  1			GE���յ��ı��ĵ���ʱ����

wire		txTimeEn 	= sw_data[4] ? glEn : flEn;
wire [23:0]	txTime 		= sw_data[4] ? grx_timestamp : frx_timestamp;
wire		rxDv 		= sw_data[4] ? gRxDvA : fe_rxDv;

event_record er (.nrst(nrst),.sysclk(sysclk),
			.lts(spd_da),.tso(free_cnt),												// in/out[23:0]
			.txTimeEn(txTimeEn),.txTime(txTime),.rxDv(rxDv),							// in/in[23:0]/in  ��������
			.rdRamAddr(t_ram_addr[9:0]),.rdRamClkEn(t_ram_en),.rdRamDa(t_ram_da),		// in[9:0]/in/out[7:0]  RAM inf
			.tri_l(send_tst_pkt),.tsto()												// out �����أ���ʾRAM��д��
			);
       
assign tsto = {spd_64k,spd_16k,spd_4k,spd_6000,spd_3000,spd_1000,gRxDvA,gTxEnA};

endmodule

