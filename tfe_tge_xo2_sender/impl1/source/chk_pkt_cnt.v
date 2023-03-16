// 2022.0325
// 检查从tge/tfe接收到的pkt_cnt值的连续性，若连续则正常

module chk_pkt_cnt  (
input           nrst,  
input           sysclk,
input			tx_tri,			// 报文触发
output [15:0]	pcnt,			// 输出的序列号
input			rst_err_cnt,	// 用于清零错误计数器
input			sec_l,			// 秒信号

input			cpcl_tge,		// tge检查点
input [15:0]	pkt_cnt_tge,	// 接收到的tge的序列号

input			cpcl_tfe,		// tfe检查点
input [15:0]	pkt_cnt_tfe,	// 接收到的tfe的序列号

output [15:0]	errCnt_tge,
output [15:0]	errCnt_tfe

);

reg	[1:0]		eg_tri_reg;
reg	[1:0]		eg_cpcl_tge;
reg	[1:0]		eg_cpcl_tfe;
reg	[1:0]		eg_rst_err_cnt;
reg	[1:0]		eg_sec_l;

always @ (negedge nrst or posedge sysclk)                 
    if (~nrst) begin  
            eg_tri_reg		<=0;
            eg_cpcl_tge		<=0;                                                            
            eg_cpcl_tfe		<=0;
            eg_rst_err_cnt	<=0;
            eg_sec_l		<=0;
        end
    else begin  
            eg_tri_reg		<= {eg_tri_reg[0],tx_tri};
            eg_cpcl_tge		<= {eg_cpcl_tge[0],cpcl_tge};                                                            
            eg_cpcl_tfe		<= {eg_cpcl_tfe[0],cpcl_tfe}; 
            eg_rst_err_cnt	<= {eg_rst_err_cnt[0],rst_err_cnt}; 
            eg_sec_l		<= {eg_sec_l[0],sec_l};
        end


wire	egr_tx_tri 		= ~eg_tri_reg[1]  & eg_tri_reg[0];
wire	egr_cpcl_tge 	= ~eg_cpcl_tge[1] & eg_cpcl_tge[0];
wire	egr_cpcl_tfe 	= ~eg_cpcl_tfe[1] & eg_cpcl_tfe[0];
wire	egr_rst_err_cnt = ~eg_rst_err_cnt[1] & eg_rst_err_cnt[0];
wire	egr_sec_l 		= ~eg_sec_l[1] & eg_sec_l[0];

// ****************  pkt_cnt **********************************

`ifdef SV_PACKET_SENDER

reg [17:0]		pcnt_reg;
always @ (negedge nrst or posedge sysclk)                 
  if (~nrst) pcnt_reg <= 0;
  else if (egr_tx_tri) pcnt_reg <= pcnt_reg + 1'b1;

assign pcnt = pcnt_reg[17:2];

`else

reg [15:0]		pcnt_reg;

always @ (negedge nrst or posedge sysclk)                 
  if (~nrst) pcnt_reg <= 0;
  else if (egr_tx_tri) pcnt_reg <= pcnt_reg + 1'b1;

assign pcnt = pcnt_reg;

`endif


// ****************  latch pkt_cnt **********************************

reg [15:0]		lat_pc_tge;
reg [15:0]		lat_pc_tfe;

reg [15:0]		diff_pc_tge;	// 差值  	
reg [15:0]		diff_pc_tfe;	// 差值  
  
always @ (negedge nrst or posedge sysclk)                 
  if (~nrst) begin 
  			lat_pc_tge <= 0;
  			diff_pc_tge <= 0;
  		end
  else if (egr_cpcl_tge) begin
  			lat_pc_tge	<= pkt_cnt_tge;
  			diff_pc_tge <= (pkt_cnt_tge - lat_pc_tge);
  		end

always @ (negedge nrst or posedge sysclk)                 
  if (~nrst) begin 
  			lat_pc_tfe <= 0;
  			diff_pc_tfe <= 0;
  		end
  else if (egr_cpcl_tfe) begin
  			lat_pc_tfe	<=  pkt_cnt_tfe;
  			diff_pc_tfe <= (pkt_cnt_tfe - lat_pc_tfe);
  		end

// ****************  判断   ********************************** 

reg			dly_egr_cpcl_tge;
reg			dly_egr_cpcl_tfe;

reg [15:0]	err_cnt_tge;
reg [15:0]	err_cnt_tfe;

always @ (negedge nrst or posedge sysclk)                 
  if (~nrst) begin 
  			dly_egr_cpcl_tge 	<= 0;
  			dly_egr_cpcl_tfe 	<= 0;
  			err_cnt_tge 		<= 0;
  			err_cnt_tfe 		<= 0;
  		end
  else  begin
  			dly_egr_cpcl_tge 	<= egr_cpcl_tge;
  			dly_egr_cpcl_tfe 	<= egr_cpcl_tfe;
  			if (egr_rst_err_cnt) begin
  			  		err_cnt_tge 		<= 0;
  					err_cnt_tfe 		<= 0;
  			 	  end
  			else begin
  				if(dly_egr_cpcl_tge) begin 
  					if (diff_pc_tge > 16'h1) err_cnt_tge	<= err_cnt_tge + diff_pc_tge;
  					else if (diff_pc_tge == 16'h0) err_cnt_tge	<= err_cnt_tge + 1'b1;
  					end
  				if(dly_egr_cpcl_tfe) begin 
  					if (diff_pc_tfe > 16'h1) err_cnt_tfe	<= err_cnt_tfe + diff_pc_tfe;
  					else if (diff_pc_tfe == 16'h0) err_cnt_tfe	<= err_cnt_tfe + 1'b1;
  					end
  				end
  		end




// ****************  超时处理   **********************************
// 连续4秒没有收到报文，则显示 0e0e
reg [3:0]		ot_cnt_tge;
reg [3:0]		ot_cnt_tfe;

always @ (negedge nrst or posedge sysclk)                 
  if (~nrst) begin
  			ot_cnt_tge <= 0;
  			ot_cnt_tfe <= 0;
  		  end
  else begin
  		if(egr_cpcl_tge) ot_cnt_tge <= 0;
  		else if (egr_sec_l) ot_cnt_tge <= {ot_cnt_tge[2:0],1'b1};
  		
  		if(egr_cpcl_tfe) ot_cnt_tfe <= 0;
  		else if (egr_sec_l) ot_cnt_tfe <= {ot_cnt_tfe[2:0],1'b1};
	end


assign errCnt_tge = ot_cnt_tge[3] ? 16'h0E0E : err_cnt_tge;
assign errCnt_tfe = ot_cnt_tfe[3] ? 16'h0E0E : err_cnt_tfe;

	
endmodule            
                                                
