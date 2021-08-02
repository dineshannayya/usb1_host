/**********************************************************************
*  Ported to USB2UART project
*  Author:  Dinesh Annayya
*           Email:- dinesha@opencores.org
*
*     Date: 4th Feb 2013
*     Changes:
*     A. Warning Clean Up
*
**********************************************************************/
/////////////////////////////////////////////////////////////////////
////                                                             ////
////  Internal Setup Engine                                      ////
////                                                             ////
////                                                             ////
////  Author: Rudolf Usselmann                                   ////
////          rudi@asics.ws                                      ////
////                                                             ////
////                                                             ////
////  Downloaded from: http://www.opencores.org/cores/usb1_funct/////
////                                                             ////
/////////////////////////////////////////////////////////////////////
////                                                             ////
//// Copyright (C) 2000-2002 Rudolf Usselmann                    ////
////                         www.asics.ws                        ////
////                         rudi@asics.ws                       ////
////                                                             ////
//// This source file may be used and distributed without        ////
//// restriction provided that this copyright statement is not   ////
//// removed from the file and that any derivative work contains ////
//// the original copyright notice and the associated disclaimer.////
////                                                             ////
////     THIS SOFTWARE IS PROVIDED ``AS IS'' AND WITHOUT ANY     ////
//// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   ////
//// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   ////
//// FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL THE AUTHOR      ////
//// OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,         ////
//// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES    ////
//// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE   ////
//// GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR        ////
//// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF  ////
//// LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT  ////
//// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  ////
//// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE         ////
//// POSSIBILITY OF SUCH DAMAGE.                                 ////
////                                                             ////
/////////////////////////////////////////////////////////////////////

//  CVS Log
//
//  $Id: usb1_ctrl.v,v 1.2 2002-09-25 06:06:49 rudi Exp $
//
//  $Date: 2002-09-25 06:06:49 $
//  $Revision: 1.2 $
//  $Author: rudi $
//  $Locker:  $
//  $State: Exp $
//
// Change History:
//               $Log: not supported by cvs2svn $
//               Revision 1.1.1.1  2002/09/19 12:07:09  rudi
//               Initial Checkin
//
//
//
//
//
//

`include "usb1_defines.v"

module usb1_ctrl(	clk, rst,

			rom_adr, rom_data,

			ctrl_setup, ctrl_in, ctrl_out,

			rx_ctrl_data, rx_ctrl_dvalid,rx_ctrl_ddone,

			ep0_din, ep0_dout, ep0_re, ep0_we, ep0_stat,
			ep0_size,

			send_stall, frame_no,
			funct_adr, configured, halt,

			v_set_int, v_set_feature, wValue, wIndex, vendor_data,

	       // Register Interface
		reg_addr,
		reg_rdwrn,
		reg_req,
		reg_wdata,
		reg_rdata,
		reg_ack


		);

input		clk, rst;

output	[6:0]	rom_adr;
input	[7:0]	rom_data;

input		ctrl_setup;
input		ctrl_in;
input		ctrl_out;


input [7:0]     rx_ctrl_data; 
input           rx_ctrl_dvalid;
input           rx_ctrl_ddone;

input	[7:0]	ep0_din;
output	[7:0]	ep0_dout;
output		ep0_re, ep0_we;
input	[3:0]	ep0_stat;
output	[7:0]	ep0_size;

output		send_stall;
input	[10:0]	frame_no;
output	[6:0]	funct_adr;
output		configured, halt;

output		v_set_int;
output		v_set_feature;
output	[15:0]	wValue;
output	[15:0]	wIndex;
input	[15:0]	vendor_data;

//-----------------------------------
// Register Interface
// ----------------------------------
output [31:0]   reg_addr;   // Register Address
output		reg_rdwrn;  // 0 -> write, 1-> read
output		reg_req;    //  Register Req
output [31:0]	reg_wdata;  // Register write data
input  [31:0]	reg_rdata;  // Register Read Data
input		reg_ack;    // Register Ack



///////////////////////////////////////////////////////////////////
//
// Local Wires and Registers
//

parameter	IDLE			=	20'b0000_0000_0000_0000_0001,
		GET_HDR			=	20'b0000_0000_0000_0000_0010,
		GET_STATUS_S		=	20'b0000_0000_0000_0000_0100,
		CLEAR_FEATURE_S		=	20'b0000_0000_0000_0000_1000,
		SET_FEATURE_S		=	20'b0000_0000_0000_0001_0000,
		SET_ADDRESS_S		=	20'b0000_0000_0000_0010_0000,
		GET_DESCRIPTOR_S	=	20'b0000_0000_0000_0100_0000,
		SET_DESCRIPTOR_S	=	20'b0000_0000_0000_1000_0000,
		GET_CONFIG_S		=	20'b0000_0000_0001_0000_0000,
		SET_CONFIG_S		=	20'b0000_0000_0010_0000_0000,
		GET_INTERFACE_S		=	20'b0000_0000_0100_0000_0000,
		SET_INTERFACE_S		=	20'b0000_0000_1000_0000_0000,
		SYNCH_FRAME_S		=	20'b0000_0001_0000_0000_0000,
		WAIT_IN_DATA		=	20'b0000_0010_0000_0000_0000,
		STATUS_IN		=	20'b0000_0100_0000_0000_0000,
		STATUS_OUT		=	20'b0000_1000_0000_0000_0000,
		V_SET_INT_S		=	20'b0001_0000_0000_0000_0000,
		V_GET_STATUS_S		=	20'b0010_0000_0000_0000_0000,
		V_GET_REG_RDATA_S	=	20'b0100_0000_0000_0000_0000,
		V_WAIT_RDATA_DONE_S     =	20'b1000_0000_0000_0000_0000;


wire	[7:0]	bmReqType, bRequest;
wire	[15:0]	wValue, wIndex, wLength;
wire		bm_req_dir;
wire	[1:0]	bm_req_type;
wire	[4:0]	bm_req_recp;

reg		get_status, clear_feature, set_feature, set_address;
reg		get_descriptor, set_descriptor, get_config, set_config;
reg		get_interface, set_interface, synch_frame;
reg		hdr_done_r, config_err;
reg		v_set_int, v_set_feature, v_get_status;

reg             v_set_reg_waddr; // Set the Reg Bus Address
reg             v_set_reg_raddr; // Set the Reg Bus Address

wire		fifo_re1, fifo_full, fifo_empty;
reg		fifo_we_d;
reg	[5:0]	data_sel;
reg		ep0_we;
reg	[7:0]	ep0_dout;
reg	[7:0]	ep0_size;
reg		send_stall;
reg	[19:0]	state, next_state;
reg		get_hdr;
reg	[7:0]	le;
wire		hdr_done;
reg		adv;
reg	[7:0]	hdr0, hdr1, hdr2, hdr3, hdr4, hdr5, hdr6, hdr7;
reg	[6:0]	funct_adr;
reg		set_adr_pending;
reg	[6:0]	funct_adr_tmp;

reg		in_size_0;
reg		in_size_1;
reg		in_size_2;
reg		in_size_4;
wire		high_sel;
reg		write_done;

//----------------------------
// Register Interface
// ----------------------------
reg   [31:0]    reg_addr;
reg   [31:0]    reg_wdata;
reg   [31:0]    reg_rdata_r;
reg             reg_req;
reg             reg_rdwrn; // 0 - write, 1 -> read
reg             reg_wphase; // register write phase
reg             reg_rphase; // register read phase
reg [3:0]       tx_bcnt; // transmit byte count
///////////////////////////////////////////////////////////////////
//
// FIFO interface
//

assign ep0_re = fifo_re1;
assign fifo_empty = ep0_stat[1];
assign fifo_full = ep0_stat[2];

///////////////////////////////////////////////////////////////////
//
// Current States
//
reg	addressed;
reg	configured;
reg	halt;
wire	clr_halt;
wire	set_halt=0;	// FIX_ME

// For this implementation we do not implement HALT for the
// device nor for any of the endpoints. This is useless for
// this device, but can be added here later ...
// FYI, we report device/endpoint errors via interrupts,
// instead of halting the entire or part of the device, much
// nicer for non-critical errors.

assign clr_halt = ctrl_setup;

always @(posedge clk)
	if(!rst)	addressed <= #1 1'b0;
	else
	if(set_address)	addressed <= #1 1'b1;

always @(posedge clk)
	if(!rst)	configured <= #1 1'b0;
	else
	if(set_config)	configured <= #1 1'b1;

always @(posedge clk)
	if(!rst)	halt <= #1 1'b0;
	else
	if(clr_halt)	halt <= #1 1'b0;
	else
	if(set_halt)	halt <= #1 1'b1;

///////////////////////////////////////////////////////////////////
//
// Descriptor ROM
//
reg	[6:0]	rom_adr;
reg		rom_sel, rom_sel_r;
wire		rom_done;
reg	[6:0]	rom_size;
reg		fifo_we_rom_r;
reg		fifo_we_rom_r2;
wire		fifo_we_rom;
reg	[7:0]	rom_start_d;
reg	[6:0]	rom_size_dd;
wire	[6:0]	rom_size_d;

always @(wValue)
	case(wValue[11:8])		// synopsys full_case parallel_case
	   4'h1:	rom_start_d = `ROM_START0;
	   4'h2:	rom_start_d = `ROM_START1;
	   4'h3:
		case(wValue[3:0])	// synopsys full_case parallel_case
		   4'h0:	rom_start_d = `ROM_START2A;
		   4'h1:	rom_start_d = `ROM_START2B;
		   4'h2:	rom_start_d = `ROM_START2C;
		   4'h3:	rom_start_d = `ROM_START2D;
		   default:	rom_start_d = `ROM_START2A;
		endcase
	   default:	rom_start_d = 7'h00;
	endcase

always @(wValue)
	case(wValue[11:8])		// synopsys full_case parallel_case
	   4'h1:	rom_size_dd = `ROM_SIZE0;
	   4'h2:	rom_size_dd = `ROM_SIZE1;
	   4'h3:
		case(wValue[3:0])	// synopsys full_case parallel_case
		   4'h0:	rom_size_dd = `ROM_SIZE2A;
		   4'h1:	rom_size_dd = `ROM_SIZE2B;
		   4'h2:	rom_size_dd = `ROM_SIZE2C;
		   4'h3:	rom_size_dd = `ROM_SIZE2D;
		   default:	rom_size_dd = `ROM_SIZE2A;
		endcase
	   default:	rom_size_dd = 7'h01;
	endcase

assign rom_size_d = (rom_size_dd > wLength[6:0]) ? wLength[6:0] : rom_size_dd;

always @(posedge clk)
	rom_sel_r <= #1 rom_sel;

always @(posedge clk)
	if(!rst)			rom_adr <= #1 7'h0;
	else
	if(rom_sel & !rom_sel_r)	rom_adr <= #1 rom_start_d;
	else
	if(rom_sel & !fifo_full)	rom_adr <= #1 rom_adr + 7'h1;

always @(posedge clk)
	if(!rst)			rom_size <= #1 7'h0;
	else
	if(rom_sel & !rom_sel_r)	rom_size <= #1 rom_size_d;
	else
	if(rom_sel & !fifo_full)	rom_size <= #1 rom_size - 7'h01;

always @(posedge clk)
	fifo_we_rom_r <= #1 rom_sel;

always @(posedge clk)
	fifo_we_rom_r2 <= #1 fifo_we_rom_r;

assign fifo_we_rom = rom_sel & fifo_we_rom_r2;

assign rom_done = (rom_size == 7'h0) & !(rom_sel & !rom_sel_r);

///////////////////////////////////////////////////////////////////
//
// Get Header
//

assign fifo_re1 = (get_hdr | reg_wphase) & !fifo_empty;

always @(posedge clk)
	adv <= #1 get_hdr & !fifo_empty & !adv;

always @(posedge clk)
	if(!rst)	le <= #1 8'h0;
	else
	if(!get_hdr)	le <= #1 8'h0;
	else
	if(!(|le))	le <= #1 8'h1;
	else
	if(fifo_re1 && get_hdr)	le <= #1 {le[6:0], 1'b0};

always @(posedge clk)
	if(le[0])	hdr0 <= #1 ep0_din;

always @(posedge clk)
	if(le[1])	hdr1 <= #1 ep0_din;

always @(posedge clk)
	if(le[2])	hdr2 <= #1 ep0_din;

always @(posedge clk)
	if(le[3])	hdr3 <= #1 ep0_din;

always @(posedge clk)
	if(le[4])	hdr4 <= #1 ep0_din;

always @(posedge clk)
	if(le[5])	hdr5 <= #1 ep0_din;

always @(posedge clk)
	if(le[6])	hdr6 <= #1 ep0_din;

always @(posedge clk)
	if(le[7])	hdr7 <= #1 ep0_din;

assign hdr_done = le[7] & fifo_re1 & get_hdr;

///////////////////////////////////////////////////////////////////
//
// Send Data to Host
//
parameter	ZERO_DATA	=	6'b000001,
		ZERO_ONE_DATA	=	6'b000010,
		CONFIG_DATA	=	6'b000100,
		SYNC_FRAME_DATA	=	6'b001000,
		VEND_DATA	=	6'b010000,
		REG_RDATA	=	6'b100000;

assign high_sel = write_done;

always @(posedge clk)
	case(data_sel)		// synopsys full_case parallel_case
	   ZERO_DATA:		ep0_dout <= #1 rom_sel ? rom_data : 8'h0;
	   ZERO_ONE_DATA:	ep0_dout <= #1 high_sel ? 8'h1 : 8'h0;
	   CONFIG_DATA:		ep0_dout <= #1 {7'h0, configured};	// return configuration
	   SYNC_FRAME_DATA:	ep0_dout <= #1 high_sel ? {5'h0, frame_no[10:8]} : frame_no[7:0];
	   VEND_DATA:		ep0_dout <= #1 high_sel ? vendor_data[15:8] : vendor_data[7:0];
	   REG_RDATA:		ep0_dout <= #1 (tx_bcnt==0) ? reg_rdata_r[31:24] : 
		                               (tx_bcnt==1) ? reg_rdata_r[23:16] :
		                               (tx_bcnt==2) ? reg_rdata_r[15:8] : reg_rdata_r[7:0];
	endcase

always @(posedge clk)
	ep0_we <= #1 (fifo_we_d & !write_done) | fifo_we_rom;

always @(posedge clk)
	if(in_size_0)	        ep0_size <= #1 8'h0;
	else if(in_size_1)	ep0_size <= #1 8'h1;
	else if(in_size_2)	ep0_size <= #1 8'h2;
	else if(in_size_4)	ep0_size <= #1 8'h4;
	else
	if(rom_sel)	ep0_size <= #1 {1'b0, rom_size_d};


always @(posedge clk) begin
   if(!rst) begin
       tx_bcnt    <= 0;
       write_done <= 0;
    end else begin
       if(state == IDLE)  begin
	  tx_bcnt <= 0;
	  write_done <= 0; 
      end else if((ep0_size == (tx_bcnt+1))  && (!fifo_full && fifo_we_d))
	  write_done <= 1;
      else if(!fifo_full && fifo_we_d ) 
	  tx_bcnt <= tx_bcnt+1;
      else 
	  write_done <= 0;
   end
end

///////////////////////////////////////////////////////////////////
//
// Decode Header
//

// Valid bRequest Codes
parameter	GET_STATUS	=	8'h00,
		CLEAR_FEATURE	=	8'h01,
		SET_FEATURE	=	8'h03,
		SET_ADDRESS	=	8'h05,
		GET_DESCRIPTOR	=	8'h06,
		SET_DESCRIPTOR	=	8'h07,
		GET_CONFIG	=	8'h08,
		SET_CONFIG	=	8'h09,
		GET_INTERFACE	=	8'h0a,
		SET_INTERFACE	=	8'h0b,
		SYNCH_FRAME	=	8'h0c,
		CUSTOM_REG_WADDR=	8'h10, // Added by Dinesh-A, 19th Feb 2013
		CUSTOM_REG_RADDR=	8'h11; // Added by Dinesh-A, 19th Feb 2013

parameter	V_SET_INT	=	8'h0f;

/*************************************************
*  Author: Dinesh-A: 18th Feb 2013  
*  Setup Byte Details
Byte	Field	Description
0	bmRequest Type	 
           Bit 7: Request direction (0=Host to device � Out, 1=Device to host � In).
           Bits 5-6: Request type (0=standard, 1=class, 2=vendor, 3=reserved).
           Bits 0-4: Recipient (0=device, 1=interface, 2=endpoint,3=other).
1	bRequest	 The actual request (see the Standard Device Request Codes table [9.2.1.5].
2	wValueL	 A word-size value that varies according to the request. For example, 
        in the CLEAR_FEATURE request the value is used to select the feature, 
	in the GET_DESCRIPTOR request the value indicates the descriptor type and in the 
	SET_ADDRESS request the value contains the device address.
3	wValueH	The upper byte of the Value word.
4	wIndexL	 A word-size value that varies according to the request. 
        The index is generally used to specify an endpoint or an interface.
5	wIndexH	The upper byte of the Index word.
6	wLengthL  A word-size value that indicates the number of bytes to be transferred if there is a data stage.
7	wLengthH  The upper byte of the Length word.
**************************************************/
/*******
bRequest	               Value
GET_STATUS	               0
CLEAR_FEATURE	               1
Reserved for future use	       2
SET_FEATURE	               3
Reserved for future use	       4
SET_ADDRESS	               5
GET_DESCRIPTOR	               6
SET_DESCRIPTOR	               7
GET_CONFIGURATION	       8
SET_CONFIGURATION	       9
GET_INTERFACE	               10
SET_INTERFACE	               11
SYNCH_FRAME	               12

*******************************/

assign bmReqType = hdr0;
assign bm_req_dir = bmReqType[7];	// 0-Host to device; 1-device to host 
assign bm_req_type = bmReqType[6:5];	// 0-standard; 1-class; 2-vendor; 3-RESERVED
assign bm_req_recp = bmReqType[4:0];	// 0-device; 1-interface; 2-endpoint; 3-other
					// 4..31-reserved
assign bRequest =  hdr1;
assign wValue   = {hdr3, hdr2};
assign wIndex   = {hdr5, hdr4};
assign wLength  = {hdr7, hdr6};

always @(posedge clk)
	hdr_done_r <= #1 hdr_done;

// Standard commands that MUST support
always @(posedge clk)
	get_status <= #1	hdr_done & (bRequest == GET_STATUS) & (bm_req_type==2'h0);

always @(posedge clk)
	clear_feature <= #1	hdr_done & (bRequest == CLEAR_FEATURE) & (bm_req_type==2'h0);

always @(posedge clk)
	set_feature <= #1	hdr_done & (bRequest == SET_FEATURE) & (bm_req_type==2'h0);

always @(posedge clk)
	set_address <= #1	hdr_done & (bRequest == SET_ADDRESS) & (bm_req_type==2'h0);

always @(posedge clk)
	get_descriptor <= #1	hdr_done & (bRequest == GET_DESCRIPTOR) & (bm_req_type==2'h0);

always @(posedge clk)
	set_descriptor <= #1	hdr_done & (bRequest == SET_DESCRIPTOR) & (bm_req_type==2'h0);

always @(posedge clk)
	get_config <= #1	hdr_done & (bRequest == GET_CONFIG) & (bm_req_type==2'h0);

always @(posedge clk)
	set_config <= #1	hdr_done & (bRequest == SET_CONFIG) & (bm_req_type==2'h0);

always @(posedge clk)
	get_interface <= #1	hdr_done & (bRequest == GET_INTERFACE) & (bm_req_type==2'h0);

always @(posedge clk)
	set_interface <= #1	hdr_done & (bRequest == SET_INTERFACE) & (bm_req_type==2'h0);

always @(posedge clk)
	synch_frame <= #1	hdr_done & (bRequest == SYNCH_FRAME) & (bm_req_type==2'h0);

always @(posedge clk)
	v_set_int <= #1		hdr_done & (bRequest == V_SET_INT) & (bm_req_type==2'h2);

always @(posedge clk)
	v_set_feature <= #1	hdr_done & (bRequest == SET_FEATURE) & (bm_req_type==2'h2);

always @(posedge clk)
	v_get_status <= #1	hdr_done & (bRequest == GET_STATUS) & (bm_req_type==2'h2);
always @(posedge clk)
	v_set_reg_waddr <= #1	hdr_done & (bRequest == CUSTOM_REG_WADDR) & (bm_req_type==2'h2);
always @(posedge clk)
	v_set_reg_raddr <= #1	hdr_done & (bRequest == CUSTOM_REG_RADDR) & (bm_req_type==2'h2);

always @(posedge clk)
	if(v_set_reg_waddr || v_set_reg_raddr) reg_addr  <= {hdr2,hdr3,hdr4,hdr5}; 

reg [1:0] reg_byte_cnt;
always @(posedge clk) begin
   if(!rst) begin
       reg_byte_cnt <= 2'b0;
   end else begin
      if(v_set_reg_waddr) begin 
	  reg_wphase      <= 1; 
	  reg_byte_cnt    <= 0;
      end else if(reg_byte_cnt == 2'b11 && fifo_re1) begin
	  reg_wphase  <= 0; 
	  reg_byte_cnt    <= 0;
      end else if(reg_wphase && fifo_re1) begin
	 reg_byte_cnt <= reg_byte_cnt+1;
      end
   end
end


always @(posedge clk)
      if(reg_wphase && fifo_re1) reg_wdata  <= {reg_wdata[23:0],ep0_din[7:0]}; 

always @(posedge clk)
      if(reg_rdwrn && reg_ack) reg_rdata_r  <= {reg_rdata}; 

always @(posedge clk) begin
   if(!rst) begin
      reg_req  <= 0; 
   end else begin
      if(fifo_re1 && reg_wphase && reg_byte_cnt== 2'b11) reg_req  <= 1; 
      else if(v_set_reg_raddr)         reg_req  <= 1; 
      else reg_req <= 0;
   end
end

always @(posedge clk)
	if(v_set_reg_raddr)      reg_rdwrn <= 1'b1 ;
	else if(v_set_reg_waddr) reg_rdwrn <= 1'b0 ;

// A config err must cause the device to send a STALL for an ACK
always @(posedge clk)
	config_err <= #1 hdr_done_r & !(get_status | clear_feature |
			set_feature | set_address | get_descriptor |
			set_descriptor | get_config | set_config |
			get_interface | set_interface | synch_frame |
			v_set_int | v_set_feature | v_get_status | v_set_reg_waddr | v_set_reg_raddr);

always @(posedge clk)
	send_stall <= #1 config_err;

///////////////////////////////////////////////////////////////////
//
// Set address
//

always @(posedge clk)
	if(!rst)				set_adr_pending <= #1 1'b0;
	else
	if(ctrl_in | ctrl_out | ctrl_setup)	set_adr_pending <= #1 1'b0;
	else
	if(set_address)				set_adr_pending <= #1 1'b1;

always @(posedge clk)
	if(!rst)			funct_adr_tmp <= #1 7'h0;
	else
	if(set_address)			funct_adr_tmp <= #1 wValue[6:0];

always @(posedge clk)
	if(!rst)			funct_adr <= #1 7'h0;
	else
	if(set_adr_pending & ctrl_in)	funct_adr <= #1 funct_adr_tmp;

///////////////////////////////////////////////////////////////////
//
// Main FSM
//

always @(posedge clk)
	if(!rst)	state <= #1 IDLE;
	else		state <= next_state;

always @(state or ctrl_setup or ctrl_in or ctrl_out or hdr_done or
	fifo_full or rom_done or write_done or wValue or bm_req_recp or
	get_status or clear_feature or set_feature or set_address or
	get_descriptor or set_descriptor or get_config or set_config or
	get_interface or set_interface or synch_frame or v_set_int or
	v_set_feature or v_get_status or v_set_reg_waddr or v_set_reg_raddr, reg_ack
	)
   begin
	next_state = state;
	get_hdr  = 1'b0;
	data_sel = ZERO_DATA;
	fifo_we_d = 1'b0;
	in_size_0 = 1'b0;
	in_size_1 = 1'b0;
	in_size_2 = 1'b0;
	in_size_4 = 1'b0;
	rom_sel = 1'b0;

	case(state)	// synopsys full_case parallel_case

		// Wait for Setup token
	   IDLE:
		   begin
			if(ctrl_setup)		next_state = GET_HDR;
			if(get_status)		next_state = GET_STATUS_S;
			if(clear_feature)	next_state = CLEAR_FEATURE_S;
			if(set_feature)		next_state = SET_FEATURE_S;
			if(set_address)		next_state = SET_ADDRESS_S;
			if(get_descriptor)	next_state = GET_DESCRIPTOR_S;
			if(set_descriptor)	next_state = SET_DESCRIPTOR_S;
			if(get_config)		next_state = GET_CONFIG_S;
			if(set_config)		next_state = SET_CONFIG_S;
			if(get_interface)	next_state = GET_INTERFACE_S;
			if(set_interface)	next_state = SET_INTERFACE_S;
			if(synch_frame)		next_state = SYNCH_FRAME_S;
			if(v_set_int)		next_state = V_SET_INT_S;
			if(v_set_feature)	next_state = V_SET_INT_S;
			if(v_get_status)	next_state = V_GET_STATUS_S;
			if(v_set_reg_waddr)	next_state = STATUS_IN;
			if(v_set_reg_raddr)	next_state = V_WAIT_RDATA_DONE_S;
		   end

		// Retrieve Setup Header
	   GET_HDR:
		   begin
			get_hdr = 1'b1;
			if(hdr_done)	next_state = IDLE;
		   end


		// Actions for supported commands
	   GET_STATUS_S:
		   begin
			// Returns to host
			// 16'h0001 for device
			// 16'h0000 for interface
			// 16'h0000 for endpoint
			if(bm_req_recp == 5'h00)	data_sel = ZERO_ONE_DATA;
			else				data_sel = ZERO_DATA;

			in_size_2 = 1'b1;
			if(!fifo_full)
			   begin
				fifo_we_d = 1'b1;
				if(write_done)	next_state = WAIT_IN_DATA;
			   end

		   end
	   V_GET_STATUS_S:
		   begin
			data_sel = VEND_DATA;
			in_size_2 = 1'b1;
			if(!fifo_full)
			   begin
				fifo_we_d = 1'b1;
				if(write_done)	next_state = WAIT_IN_DATA;
			   end
		   end

	V_WAIT_RDATA_DONE_S: begin // Wait for Register Read Access Completion
		     if(reg_ack)
			next_state = V_GET_REG_RDATA_S;
		   end

	   V_GET_REG_RDATA_S: // Register Access Read Data
		   begin
			data_sel = REG_RDATA;
			in_size_4 = 1'b1;
			if(!fifo_full)
			   begin
				fifo_we_d = 1'b1;
				if(write_done)	next_state = WAIT_IN_DATA;
			   end
		   end		   
	   CLEAR_FEATURE_S:
		   begin
			// just ignore this for now
			next_state = STATUS_IN;
		   end

	   SET_FEATURE_S:
		   begin
			// just ignore this for now
			next_state = STATUS_IN;
		   end

	   SET_ADDRESS_S:
		   begin
			// done elsewhere ....
			next_state = STATUS_IN;
		   end

	   GET_DESCRIPTOR_S:
		   begin
			if(	wValue[15:8] == 8'h01 |
				wValue[15:8] == 8'h02 | 
				wValue[15:8] == 8'h03	)
				rom_sel = 1'b1;
			else
				next_state = IDLE;

			if(rom_done)
				next_state = IDLE;
		   end

	   SET_DESCRIPTOR_S:
		   begin
			// This doesn't do anything since we do not support
			// setting the descriptor
			next_state = IDLE;
		   end

	   GET_CONFIG_S:
		   begin
			// Send one byte back that indicates current status
			in_size_1 = 1'b1;
			data_sel = CONFIG_DATA;
			if(!fifo_full)
			   begin
				fifo_we_d = 1'b1;
				next_state = WAIT_IN_DATA;
			   end
		   end

	   SET_CONFIG_S:
		   begin
			// done elsewhere ....
			next_state = STATUS_IN;
		   end

	   GET_INTERFACE_S:
		   begin
			// Return interface '0'
			in_size_1 = 1'b1;
			if(!fifo_full)
			   begin
				fifo_we_d = 1'b1;
				next_state = WAIT_IN_DATA;
			   end
		   end

	   SET_INTERFACE_S:
		   begin
			// just ignore this for now
			next_state = STATUS_IN;
		   end

	   SYNCH_FRAME_S:
		   begin
			// Return Frame current frame number
			data_sel = SYNC_FRAME_DATA;
			in_size_2 = 1'b1;
			if(!fifo_full)
			   begin
				fifo_we_d = 1'b1;
				if(write_done)	next_state = WAIT_IN_DATA;
			   end
		   end

	   V_SET_INT_S:
		   begin
			// done elsewhere ....
			next_state = STATUS_IN;
		   end

	   WAIT_IN_DATA:
		   begin
			if(ctrl_in)	next_state = STATUS_OUT;
		   end

	   STATUS_IN:
		   begin
			in_size_0 = 1'b1;
			if(ctrl_in)	next_state = IDLE;
		   end

	   STATUS_OUT:
		   begin
			if(ctrl_out)	next_state = IDLE;
		   end
	endcase
   end

endmodule

