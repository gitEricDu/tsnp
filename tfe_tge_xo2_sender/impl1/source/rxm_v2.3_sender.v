// 2022.0215
// �����ⱨ�������ʱ���

// 2021.0208

// V 2.3
// ���� MASTER��ʼ��ʱ�Զ����� ͨ���ŵ��Ĺ���

// 2020.0701 

// �����˶Ա����Զ�����ĵ�ַ���ж����
// rxLocalMac : ��MASTER�Զ������DNM�ĵ�ַ
// rxDnmNum	:	MASTER����ʱ�����е��豸����λ�õĲ���ֵ����ֵӦ��rxLocalMac���
// rxLocalMac == rxDnmNum ��˵���Զ�����ĵ�ַ����ȷ�ģ�localIndexAviable = 1'b1;



// 2020.0609  :::: 20200609
// Ver 2.2
// ��Ϊ��SYNC����ȡconfig��ֵ�������пɿ��Դ���
// Ŀ�ģ�	
// 		1����·����
//		2��DNM�ն˼�⣨������⣩
//		3����type����ȡDNM��������only for MAM

// Ver 2.1 20191225
//  		�����˸�֡��ȡ����
// 20200428 ��֡��������8λ����Ϊ16λ

//
// Ver 2.0  20191016
// Դ�ļ������� gmii_rx_v1.1
// �Ż�������ٶȣ�������xo2-4000hc-4��оƬ���ٶ����Ϊ -6��

// rev 1.1  2018.12.04
//   		����ά��������״̬�Ĵ���

// 20180713�� ������eddst_rx�ļ�


module rxm_v23_sender (
input           nrst,         
input           enLatch,		// H ������ģ������뼤��״̬�󣬲��ٸ���local_mac/host_mac/delay_time/config_value��ֵ
input			rst_latch,		// H ��λlocal_mac/host_mac/delay_time/config_value��ֵ 
input			rst_p,			// H ���帴λ��2clk����λ��Щ����assign rxLocalMac/assign rxHostMac/assign rxLastCmd/assign rxDelayTime/assign rxCfgValue��slave ��Ч
input			one_second_l,	// H һ��2mʱ�ӿ�,����ͳ������

//GMII interface  
input			gmiiClk,
input           gmiiRxDv,                                       
input  [7:0]   	gmiiRxD,                                       
input           gmiiRxErr, 

// GMII output interface
output			gmiiTxEn,	// ����phy��enable
output [7:0]	gmiiTxDa,	// ������phy������ 

// ***************  Drop *********************************

output [7:0]	rx_data,		// ���ԼĴ��� dly_reg4���в������ݱ��ı䣬���������������Ӱ��
output			rx_end,			// �Ĵ���ֵ���ⲿû��
output			rx_da_en,		// �����Ĵ�����ȥ��MAC��FCS����û�����
output			rxCrcErr,		// ��ƽ


// �����ź���CRC��Ч�󣬲���Ч
output          findReset_l,
output          findDevice_l,		// ����������������MASTER
output          findDeviceEnd_l,
output			findInit_l,			// ���ڲ�ͬ��ʱ��
output			findActive_l,
output			findSync_l,			// ��ƽ�źţ�����ͬ���ⲿ���뱨��ͬ��

// �����źŽ��յ����ĺ���Ч
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
input [15:0]	dropDaAddr,			// {start,end} �������ߵ���ʼ��ַ����=0����ʾ�رղ��빦�ܣ����ݷ�Χ1-128��������ַ
output			dropDaEn,			// ������������
output [7:0]	dropDa,				// ���ź���dropDaEn ��ϣ���ֱ�ӽ����յ�������д��RAM��ȥ�����Ǳ��ؽ��յ�����

input [15:0]	addDaAddr,			// {start,end} �������ߵ���ʼ��ַ����=0����ʾ�رղ��빦�ܣ����ݷ�Χ1-128��������ַ
output			addDaEn,			// ���ݲ�������
input [7:0]		addDa,				// ��������ݱ��ĵ�����


// status ADD/DROP
input [15:0]	dropStAddr,		// {start,end} ״̬�����ߵ���ʼ��ַ����=0����ʾ�رղ��빦�ܣ����ݷ�Χ1-64��������ַ
output			dropStEn,			// ������������
output [7:0]	dropStDa,			// ���ź���dropStEn ��ϣ���ֱ�ӽ����յ�������д��RAM��ȥ�����Ǳ��ؽ��յ�����

input [15:0]	addStAddr,			// {start,end} ״̬�����ߵ���ʼ��ַ����=0����ʾ�رղ��빦�ܣ����ݷ�Χ1-128��������ַ
output			addStEn,			// ���ݲ�������
input [7:0]		addStDa,			// �����״̬���ĵ�����

// ***************  misc *********************************

// �������
output [15:0]	rxHostMac,		// only for slave init��������ȡ����ģ���ڲ�û��
output [7:0]	rxLocalMac,		// only for slave init��������ȡ��
output [23:0]	rxDelayTime,	// only for slave  ACTIVE/init ������ȡ��delay_time/timsstamp �������ݣ���ģ��û��
output [31:0]	rxCfgValue,		// only for slave ACTIVE������ȡ����ģ��û��
output [7:0]	rxLastCmd,		// master/slave ctrl/ectrl/edata ������������ȡ����ģ��û��
output [1:0]	mfc,			// only for slave sync��������ȡ�����ڸ�֡ͬ��,{H,L}
output [39:0]	rxRev,			// ���ڲ���
output [7:0]	star_chNo,		// �� master ���յ�����ʼ��ַ

input [7:0]		localMac,		// �ĸ����� ctrl/ectrl/edata/active ʹ�ô˵�ַ�ж�

// ��⼰����
output [15:0]	rxPacketNum,
output [15:0]	rxPacketCrcErrNum,
output [7:0]	rxDnmNum,			// ���DNM��������ֻ��MAMʹ��
output 			localIndexAviable,	// ���ص�ַУ����ȷ���ڲ���ʹ�ã�����ϵͳ����

// mib 
output [1:0]	mib_da2,
// test
output			led_crc,			// ÿ�����һ�Σ�����LEDָʾ�Ƿ���CRC����
output			rxSfd
, output		lts					// ��ƽ

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
reg             rx_data_en;	// ��ӦRxD_reg������
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
reg				insert_device_cnt;		// DNM ����
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


// �Ż�	
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
            State_idle:		if (RxDv_reg&&RxD_reg==8'h55)	Current_state <=State_preamble;  	// ����					
            State_preamble:	if (!RxDv_reg | RxErr_reg)		Current_state <=State_drop;   	//    
							//else if (RxErr_reg)			Current_state <=State_drop;        
							else if (RxD_reg==8'hd5)	Current_state <=State_SFD;                 
							else if (RxD_reg!=8'h55)	Current_state <=State_drop;  // pre״̬�µĴ���ͳ��   

						
            State_SFD:		if (!RxDv_reg | RxErr_reg)		Current_state <=State_drop;      
							//else if (RxErr_reg)	Current_state <=State_drop;        
							else				Current_state <=State_data;    
						
            State_data:		if (!RxDv_reg&&!Too_short) 		Current_state <=State_checkCRC;   
							else if (!RxDv_reg&&Too_short || RxErr_reg)	Current_state <=State_drop;		// С��64�ֽڵĶ̱���
						//	else if (RxErr_reg)				Current_state <=State_drop;        
						
            State_checkCRC:	if (crc_chk_err_level_reg)Current_state <=State_IFG;
							else         			Current_state <=State_OkEnd; 
						 
            State_drop:		if (!RxDv_reg)  		Current_state <=State_IFG;    // �ڽ��չ����У����� RxErr���ڴ���ͳ��  
   						
            State_OkEnd:  	Current_state <=State_IFG;         						// ����������������������ͳ��
       //     State_ErrEnd:	Current_state <=State_IFG;                        		// RxDv�������ڴ���ͳ��                                              
       //     State_CRCErrEnd:Current_state <=State_IFG;                          	// CRC�������ڴ���ͳ�� 
		//	State_err_pre:	Current_state <=State_IFG;                          	// ͬ����������⣬���ڴ���ͳ��
            State_IFG: 		if (ifgCnt==4'd8)	Current_state <=State_idle;   		//remove some additional time                                                     
            default:		Current_state <=State_idle;        
        endcase                                         
		end

//******************************************************************************
//  ��������
//  1��ifgCnt ���ڱ��ļ����ʱ
//  2��genCnt ����С��128����ǰ��ͷ�����ݶ�λ
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
//  ���ñ���
//  1��rx_data_en ����DMAC/SMAC��������Ч�źţ���RxD_reg���Ӧ
//  2��crc_chk_init_reg ���ڳ�ʼ��CRCģ�飬֮ǰʹ�õ�������źţ��ָ�Ϊreg�źţ��� RxD_reg ��ƥ��
//  3��too_short ���ڶ̱���ʱ�ĸ澯��ʾ
//****************************************************************************** 
// RxD_reg������ 
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) rx_data_en <= 1'b0;   
    else if ((Current_state==State_data||Current_state==State_SFD ||rx_sfd_reg ) && gmiiRxDv ) 	rx_data_en <= 1'b1;                              
    else rx_data_en <= 1'b0;
		
// ���� RxD_reg������
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) rx_sfd_reg <= 1'b0;   
    else if (Current_state==State_preamble && gmiiRxD==8'hd5 ) 	rx_sfd_reg <= 1'b1;                              
    else rx_sfd_reg <= 1'b0;


// �Ż�
always @ (negedge nrst or posedge Clk)                 
    if (~nrst) Too_short <= 1'b0;   
    else if (genCnt < 7'd63)  Too_short   <=1'b1;                            
    else  Too_short   <=1'b0;


  
//************************************************************************************************
//   1�����ݶ�λ����������ģ����Ӳ�ͬ�⣬�������ĵ�MACַ��ȫһ��������ֻ��Ҫ��λDMAC��ַ����
//   2�������ڲ�������ȡ�����CRC����
//************************************************************************************************

reg			posi_dmac_l;
reg			posi_type;
reg			posi_pkt_cnt;
reg			posi_sv_cnt;
reg [23:0]	gmacdata;
reg [7:0]	rx_tmp_da;
								
// ����λλ�ö�λ�����������RxD_reg�Ķ�λ
always @ (posedge Clk or negedge nrst)
    if (~nrst) 	begin 
    			posi_dmac_l <= 1'b0;
    			posi_type 	<= 1'b0;
    			posi_pkt_cnt<= 0;
    			posi_sv_cnt<= 0;
    			end
	else if(rx_data_en) begin
			posi_dmac_l 	<= (genCnt == 7'd4);		// ��ӦDMAS�����2λ
			posi_type 		<= (genCnt == 7'd12);		// ��Ӧtype������
			posi_pkt_cnt 	<=  (genCnt == 7'd23);		// ��Ӧtype������
			posi_sv_cnt 	<=  (genCnt == 7'd66);		// ��Ӧtype������
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
//   ���Ľ�������10�౨�ģ�ȫ�������ƽ�źţ����ڵ�ƽ�ź�û����CRC���
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


	// �˴�ʹ��16λ���ݣ���Ҫ������ٶ�
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

		
	// ��ƽ����ź�		
	assign findCtrl_l 	= ctrl_l_reg;
	assign findEctrl_l 	= ectrl_l_reg;
	assign findEdata_l 	= edata_l_reg;
	assign findData_l 	= data_l_reg;
	assign findStstus_l = status_l_reg;
	assign findUnknow_l	= unknow_l;
	


	
//******************************************************************************
//
//   ��������������������౨�ģ����Ϊ��ƽ�źţ����Ϊ3 cycle
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

// ��ƽ�źţ�3*cycle),���ź��ڱ��ķ�����ɺ���Ч,�������ֲ��������ģ�״̬���л���ɵģ�
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
// �������ݿ���
//
// 1. STATUS���ģ�
// 2. data���ģ�
//
//******************************************************************************

reg [5:0]   insert_cnt;			// ���ü�������������ʹ��
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


// �������ź������ڲ��л�����
reg		dly_add_data_en;
reg		dly_add_status_en;

// 5 bit counter
always @ (posedge Clk or negedge nrst)
    if (~nrst)          					insert_cnt        <= 0; 
    else if (Current_state==State_SFD ) 	insert_cnt        <= 0;
    else if (Current_state==State_data) 	insert_cnt        <= insert_cnt+ 1'b1;
    

// data oacket counter (64 byte per channel)  �������� RxD_reg �ϲ�
always @ (posedge Clk or negedge nrst)
    if (~nrst)   ch_cnt_64B <= 0;
    else if(data_l_reg )   begin 
    			if (insert_cnt == 6'ha) ch_cnt_64B <= ch_cnt_64B + 1'b1;
    			end
    else ch_cnt_64B <= 0;
		
// status packet counter (16 byte per channel)  �������� dly_reg0 �ϲ� 
always @ (posedge Clk or negedge nrst)
    if (~nrst)   ch_cnt_16B <= 0;
    else if(status_l_reg)   begin 
    			if (insert_cnt[3:0] == 4'hb)ch_cnt_16B <= ch_cnt_16B + 1'b1;
    			end
    else ch_cnt_16B <= 0;

// Ϊ������ٶȣ����ݵ���Ч���ж�Ԥ����
always @ (posedge Clk or negedge nrst)
    if (~nrst)    begin avaib_status_drop <= 0;avaib_status_add <= 0; avaib_data_drop <= 0;  avaib_data_add <= 0; end
    else begin 
    		avaib_status_drop 	<= (dropStAddr[15:8] != 0) && (dropStAddr[7:0]> dropStAddr[15:8]) && (dropStAddr[7:0] <= 8'd65);
    		avaib_status_add 	<= (addStAddr[15:8]  != 0) && (addStAddr[7:0] > addStAddr[15:8])  && (addStAddr[7:0] 	<= 8'd65);
    		avaib_data_drop 	<= (dropDaAddr[15:8] != 0) && (dropDaAddr[7:0]> dropDaAddr[15:8]) && (dropDaAddr[7:0] <= 8'd129);
    		avaib_data_add 		<= (addDaAddr[15:8]  != 0) && (addDaAddr[7:0] > addDaAddr[15:8])  && (addDaAddr[7:0]  <= 8'd129);
    	end	


// ��λstatus���ݵĲ���λ��	
always @ (posedge Clk or negedge nrst)
    if (~nrst)   drop_status_en <= 1'b0;
    else if((ch_cnt_16B == dropStAddr[15:8]) && avaib_status_drop && status_l_reg)   drop_status_en <= 1'b1;
		 else  if(ch_cnt_16B == dropStAddr[7:0])drop_status_en <= 1'b0;			 			 
always @ (posedge Clk or negedge nrst)
    if (~nrst)   add_status_en <= 1'b0;
    else if((ch_cnt_16B == addStAddr[15:8]) && avaib_status_add && status_l_reg)   add_status_en <= 1'b1;
		 else  if(ch_cnt_16B == addStAddr[7:0]) add_status_en <= 1'b0;


// ��λdata���ݵĲ���λ��	
always @ (posedge Clk or negedge nrst)
    if (~nrst)   drop_data_en <= 1'b0;
    else if((ch_cnt_64B == dropDaAddr[15:8]) && avaib_data_drop && data_l_reg)   drop_data_en <= 1'b1;
		 else  if(ch_cnt_64B == dropDaAddr[7:0]) drop_data_en <= 1'b0;	
always @ (posedge Clk or negedge nrst)
    if (~nrst)   add_data_en <= 1'b0;
    else if((ch_cnt_64B == addDaAddr[15:8]) && avaib_data_add && data_l_reg)   add_data_en <= 1'b1;
		 else  if(ch_cnt_64B == addDaAddr[7:0]) add_data_en <= 1'b0;	



// �������ź����ڶ�ȡ�ⲿRAM����ǰ��һ��cycle
assign dropDaEn 	= drop_data_en;						//RAM д��
assign dropDa	 	= gmiiRxD;//drop_data_en ? gmiiRxD : 8'h0;	// MUX ��Ҫ���ڲ��Է���
assign addDaEn 		= add_data_en;						// RAM ����

assign dropStEn		= drop_status_en;
assign dropStDa	 	= RxD_reg;//drop_status_en ? RxD_reg : 8'h0;	// MUX ��Ҫ���ڲ��Է���
assign addStEn		= add_status_en;



// �������ź����ڿ����ڲ�����������������ź��ͺ�һ��ʱ������
always @ (posedge Clk or negedge nrst)
    if (~nrst)    begin 
    				dly_add_data_en 	<= 1'b0; 
    				dly_add_status_en 	<= 1'b0; 
    				end
    else 		 begin 
    				dly_add_data_en 	<= add_data_en;
    				dly_add_status_en 	<= add_status_en;
    			end
    

// ���ݲ��뼰ʱ��������                                                            

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
				dly_reg2 	<= insert_device_cnt ? (rev_reg[7:0] + 1'b1) : dly_reg1;		// �˴���1��ÿͨ��һ��DNM������1
				dly_reg3 	<= crc_gen_enable ? crc_gen_daout : dly_reg2;					// ����CRC��ֵ  
				
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
		
		
// ȥ��MAC��TYPE������ݣ�����14yte)�����źŵ��������������������д�뵽FIFO�Ŀ����źŵ���ʼ
always @ (posedge Clk or negedge nrst)
    if (~nrst) rx_da_en_reg <= 1'b0;
	else  if(genCnt == 7'h10) rx_da_en_reg <= 1'b1;
		  else if(Current_state==State_IFG) rx_da_en_reg <= 1'b0;

		
assign rx_data 	= rx_tmp_da; 
assign rx_da_en	= RxDv_reg & rx_da_en_reg;	// ȥ��CRC/MAC/TYPE��ģ̣����û�����
assign rx_end	= rx_end_reg;


assign gmiiTxEn		= tx_en_reg3;
assign gmiiTxDa 	= dly_reg3;


//******************************************************************************
// �����������������Ϣ
//
// 1. init��������ȡ local mac / host_mac_reg ��ֵ
// 2. ctrl/ectrl/edata ��ȡ last_cmd��ֵ 
// 3. active ������ȡ delay/config
//
//******************************************************************************

// INIT ������ȡlocal mac / host_mac_reg


// �˴���Ҫ�ر�˵��
// 1���������е�������CRC����Ч�����Ե�һ��init���ģ���rst_latch��Чʱ�����ġ�
// 2������enLatch�¼�⣬����Զ�޷�ͬ��
// 3������λ���յ���λ����ʱ��λ

always @ (posedge Clk or negedge nrst)
    if (~nrst)   begin local_mac_reg <=0; host_mac_reg <= 0; end
	else if(rst_p) begin local_mac_reg <=0; host_mac_reg <= 0; end
    else if(init_l_reg && rst_latch && genCnt == 7'hd)begin
			 local_mac_reg 	<= gmiiRxD;
			 host_mac_reg 	<= {rx_tmp_da,gmacdata[23:16]};
			end
			
// ctrl/ectrl/edata ��ȡ last_cmd��ֵ 
always @ (posedge Clk or negedge nrst)
    if (~nrst)   last_cmd_reg <=0; 
    else if(rst_p) last_cmd_reg <=0;
    else if(ctrl_l_reg|ectrl_l_reg|edata_l_reg)begin
			 if(genCnt == 7'h6) last_cmd_reg <= rx_tmp_da;
			end

// ACTIVE������ȡ��delay_reg / start_dn_ch_num;  2021.02.08 ����
// ����λ���յ���λ����ʱ��λ
//reg [7:0]	start_dn_ch_num;
/*
always @ (posedge Clk or negedge nrst)
    if (~nrst)  	begin delay_reg <=16'h0; start_dn_ch_num <= 0; end
	else if(rst_p) 	begin delay_reg <=16'h0; start_dn_ch_num <= 0; end
    else if(active_l_reg & (enLatch))begin
			if(genCnt == 7'h11) delay_reg 		<= gmacdata[15:0];
			if(genCnt == 7'h14) start_dn_ch_num <= gmacdata[23:16];		//2021.02.08 ����
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
   
			
// SYNC ������ȡ��conf_reg   20200609
// ����λ���յ���λ����ʱ��λ
reg [1:0]	conf_cnt_reg;		// ����������¼���Σ������ݶ���ȷ�����沢���
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

// 20200609 ���ڸı�SYNC�����е�TTL
always @ (posedge Clk or negedge nrst)
    if (~nrst) insert_device_cnt <= 0;
    else insert_device_cnt <= sync_l_reg & (genCnt == 7'h1a);

// 20200609 ��ȡDNM���ݣ�only for DNM
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


// 20200428 �޸�
// 1���ĳ���16λ��ԭ8λ
// 2�����������־��mfc_l/mfc_h;

 reg mfCntl_reg ;
 reg mfCnth_reg ;
always @ (posedge Clk or negedge nrst)
    if (~nrst) begin mfCntl_reg <= 0; mfCnth_reg <= 0; end
    // mfc��λ������ǰ����λ�����ں�
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
//   CRC ���� ,����TxDa/TxEn�����ݵ�FCS                                                         
//******************************************************************************
 reg [2:0]	crc_en_cnt;
 wire		crc_gen_en;
 
// ����CRC_genģ���������
always @ (posedge Clk or negedge nrst)
    if (~nrst) crc_en_cnt <= 3'h7;
	else  if(crc_gen_en) crc_en_cnt <= 3'h0;
		  else if(!crc_en_cnt[2]) crc_en_cnt <= crc_en_cnt + 1'b1;

assign  crc_gen_enable 	= ~crc_en_cnt[2] & ~crc_gen_en;		// CRC �����������
assign 	crc_gen_en 		= dly_en_reg2 & gmiiRxDv;			// ���ź���dly_reg2����

CRC_gen rxm_crc_gen(
.Reset                    (~nrst),
.Clk                      (Clk),
.Init                     (rx_sfd_reg),	// �˶�Ӧ dly_reg0��SFD
.Frame_data               (dly_reg2),   		// ʹ�úϳɺ������
.Data_en                  (crc_gen_en), 
.CRC_rd                   (crc_gen_enable),
.CRC_out                  (crc_gen_daout),		// output  [7:0]
.CRC_end                  ()					//��������ʱû�� 
);
	
//******************************************************************************
//   CRC ��飬�������RxD/RxDv�����ݵ���Ч��                                                           
//******************************************************************************

wire			crc_chk_err;
	
CRC_chk rxm_crc_chk(
.Reset                      (~nrst), 
.Clk                        (Clk),
.CRC_init                   (rx_sfd_reg),	// �˶�Ӧ RxD_reg��SFD
.CRC_data                   (RxD_reg),			// ʹ�������ԭʼ����
.CRC_en                     (rx_data_en),		// ��ӦRxD_reg��L2���� 

 //From CPU  
.CRC_chk_en                 (crc_chk_eof),		// �ڴ˼������ CRC_err �ź�
.CRC_err                    (crc_chk_err),		// �߼��������ǰû��
.CRC_err_l					(crc_chk_err_l)		// One clock delay compared to CRC_err
);   


// �����ź���Ϊ������ٶȶ����ӵ�reg���������ź�Ϊ��ƽ�ź�
always @ (posedge Clk or negedge nrst)
    if (~nrst) crc_chk_err_level_reg <= 0;
	else if(crc_chk_err_l) crc_chk_err_level_reg <= 1'b1;
	     else if(Current_state==State_idle) crc_chk_err_level_reg <= 1'b0;
			 
//******************************************************************************
//
//   ά������ģ��
//
//	1. latch_packet_all_cnt 	���յ��ı�������������
//	2. latch_packet_all_crc_err_cnt 	���յ��ı���C����RC����������
//   
//******************************************************************************	


reg [15:0]		packet_all_cnt_reg;
reg [15:0]		latch_packet_all_cnt;
reg	[1:0]		one_second_edge;

always @ (posedge Clk or negedge nrst)
    if (!nrst) one_second_edge <= 0;
    else one_second_edge <= {one_second_edge[0],one_second_l};

//wire one_second_e = ~one_second_edge[0] & one_second_edge[1];	// �½���

reg 	one_second_e;
always @ (posedge Clk or negedge nrst)
    if (!nrst) one_second_e <= 0;
    else one_second_e <=  ~one_second_edge[0] & one_second_edge[1];	


// ���յ�ȫ�����ļ���
always @ (posedge Clk or negedge nrst)
    if (~nrst) packet_all_cnt_reg <= 0;
    else if(one_second_e) packet_all_cnt_reg <= 0;
    else if(Current_state==State_SFD) packet_all_cnt_reg <= packet_all_cnt_reg + 1'b1;

always @ (posedge Clk or negedge nrst)
    if (~nrst) latch_packet_all_cnt <= 0;
    else if(one_second_e) latch_packet_all_cnt <= packet_all_cnt_reg;


// ���յ�ȫ������CRCУ��

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
//   1��rxDv 			����ͳ���յ���ȫ����������
//   2��pkt_ok_reg		����ͳ����ȷ�ı��ĵ�����,��������FCS��RAM��FIFO�����ɵĶ�����
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
                                                
