////////////////////////////////////////////////////////////////////////////
// SPDX-FileCopyrightText:  2021 , Dinesh Annayya
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0
// SPDX-FileContributor: Modified by Dinesh Annayya <dinesha@opencores.org>
//////////////////////////////////////////////////////////////////////
////                                                              ////
////                                                              ////
////  To Do:                                                      ////
////    nothing                                                   ////
////                                                              ////
////  Author(s):                                                  ////
////      - Dinesh Annayya, dinesha@opencores.org                 ////
////                                                              ////
////  Revision :                                                  ////
////    0.1 - 16th Feb 2021, Dinesh A                             ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Authors and OPENCORES.ORG                 ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////

`default_nettype none

 
`include "usb1_defines.v"
//`include "usb_test1.v"
//`include "usb_test2.v"
`include "usb_test3.v"


module tb_top;

parameter  WB_BP_PER = 2.5;       
parameter  USB_BP_PER = 10.4167;

reg usb_48mhz_clk;
reg        wb_clk;
reg        wb_rst_i;

reg        wbd_ext_cyc_i;  // strobe/request
reg        wbd_ext_stb_i;  // strobe/request
reg [31:0] wbd_ext_adr_i;  // address
reg        wbd_ext_we_i;  // write
reg [31:0] wbd_ext_dat_i;  // data output
reg [3:0]  wbd_ext_sel_i;  // byte enable

wire [31:0] wbd_ext_dat_o;  // data input
wire        wbd_ext_ack_o;  // acknowlegement
wire        wbd_ext_err_o;  // error

reg         test_fail;
reg [31:0]  read_data;
wire        uart_txd;
wire        uart_rxd;



always begin
     #WB_BP_PER     wb_clk <= 1'b0;
     #WB_BP_PER     wb_clk <= 1'b1;
end
 
always begin
     #USB_BP_PER     usb_48mhz_clk <= 1'b0;
     #USB_BP_PER     usb_48mhz_clk <= 1'b1;
end


	initial begin
                wbd_ext_cyc_i ='h0;  // strobe/request
                wbd_ext_stb_i ='h0;  // strobe/request
                wbd_ext_adr_i ='h0;  // address
                wbd_ext_we_i  ='h0;  // write
                wbd_ext_dat_i ='h0;  // data output
                wbd_ext_sel_i ='h0;  // byte enable
	end

	`ifdef WFDUMP
	   initial begin
	   	$dumpfile("tb_top.vcd");
	   	$dumpvars(0, tb_top);

	   end
       `endif

	initial begin
		test_fail = 0;
		wb_rst_i = 1'b1;
		#100;
		wb_rst_i = 1'b0;	    	// Release reset

	        repeat (200) @(posedge wb_clk);
			// $display("+1000 cycles");
			//
		//usb_test1;
	        //usb_test2;
	        usb_test3;


          	if(test_fail == 0) begin
		   `ifdef GL
	    	       $display("Monitor: SPI Master Mode (GL) Passed");
		   `else
		       $display("Monitor: SPI Master Mode (RTL) Passed");
		   `endif
	        end else begin
		    `ifdef GL
	    	        $display("Monitor: SPI Master Mode (GL) Failed");
		    `else
		        $display("Monitor: SPI Master Mode (RTL) Failed");
		    `endif
		 end
	    	$display("###################################################");
	    $finish;
	end


wire  usbd_txoen,usbd_txdp,usbd_txdn;
wire  usbh_txdp,usbh_txdn,usbh_txoen;

wire dpls    = (usbd_txoen == 1'b0) ? usbd_txdp : 
	       (usbh_txoen == 1'b0) ? usbh_txdp :  1'bz;
wire dmns    = (usbd_txoen == 1'b0) ? usbd_txdn : 
	       (usbh_txoen == 1'b0) ? usbh_txdn :  1'bz;

wire usb_rxd = ((dpls == 1) && (dmns == 0)) ? 1'b1:
	       ((dpls == 0) && (dmns == 1)) ? 1'b0: 1'b0;
 

pullup(dpls); // Full Speed Device Indication
//pulldown(dmns);

usb1_host u_usb_host(
    .usb_clk_i   (usb_48mhz_clk  ),
    .usb_rstn_i  (!wb_rst_i      ),

    // USB D+/D-
    .in_dp        (dpls           ),
    .in_dn        (dmns           ),

    .out_dp       (usbh_txdp    ),
    .out_dn       (usbh_txdn    ),
    .out_tx_oen   (usbh_txoen   ),

    // Master Port
    .wbm_rst_n   (!wb_rst_i      ), // Regular Reset signal
    .wbm_clk_i   (wb_clk         ), // System clock
    .wbm_stb_i   (wbd_ext_stb_i  ), // strobe/request
    .wbm_adr_i   (wbd_ext_adr_i[5:0]  ), // address
    .wbm_we_i    (wbd_ext_we_i   ), // write
    .wbm_dat_i   (wbd_ext_dat_i  ), // data output
    .wbm_sel_i   (wbd_ext_sel_i  ), // byte enable
    .wbm_dat_o   (wbd_ext_dat_o  ), // data input
    .wbm_ack_o   (wbd_ext_ack_o  ), // acknowlegement
    .wbm_err_o   (),  // error

    // Outputs
    .usb_intr_o ()


    );


usb2uart u_usb2uart(

	.clk_i      (usb_48mhz_clk), 
	.rstn_i     (!wb_rst_i),
 
		// USB PHY Interface
	.usb_txdp   (usbd_txdp), 
	.usb_txdn   (usbd_txdn), 
	.usb_txoe   (usbd_txoen),
	.usb_rxd    (usb_rxd), 
	.usb_rxdp   (dpls), 
	.usb_rxdn   (dmns),
 
	// USB Misc
	.phy_tx_mode(1'b1), 
        .usb_rst(),
 
	// Interrupts
	.dropped_frame(), 
	.misaligned_frame(),
	.crc16_err(),
 
	// Vendor Features
	.v_set_int(), 
	.v_set_feature(), 
	.wValue(),
	.wIndex(), 
	.vendor_data(),
 
	// USB Status
	.usb_busy(), 
	.ep_sel(),
 
	// End point 1 configuration
	.ep1_cfg(	`ISO  | `IN  | 14'd0256		),
	// End point 1 'OUT' FIFO i/f
	.ep1_dout(					),
	.ep1_we(					),
	.ep1_full(		1'b0			),
	// End point 1 'IN' FIFO i/f
	.ep1_din(		8'h0		        ),
	.ep1_re(		   		        ),
	.ep1_empty(		1'b0     		),
	.ep1_bf_en(		1'b0			),
	.ep1_bf_size(		7'h0			),
 
	// End point 2 configuration
	.ep2_cfg(	`ISO  | `OUT | 14'd0256		),
	// End point 2 'OUT' FIFO i/f
	.ep2_dout(				        ),
	.ep2_we(				        ),
	.ep2_full(		1'b0     		),
	// End point 2 'IN' FIFO i/f
	.ep2_din(		8'h0			),
	.ep2_re(					),
	.ep2_empty(		1'b0			),
	.ep2_bf_en(		1'b0			),
	.ep2_bf_size(		7'h0			),
 
	// End point 3 configuration
	.ep3_cfg(	`BULK | `IN  | 14'd064		),
	// End point 3 'OUT' FIFO i/f
	.ep3_dout(					),
	.ep3_we(					),
	.ep3_full(		1'b0			),
	// End point 3 'IN' FIFO i/f
	.ep3_din(		8'h0      		),
	.ep3_re(		        		),
	.ep3_empty(		1'b0    		),
	.ep3_bf_en(		1'b0			),
	.ep3_bf_size(		7'h0			),
 
	// End point 4 configuration
	.ep4_cfg(	`BULK | `OUT | 14'd064		),
	// End point 4 'OUT' FIFO i/f
	.ep4_dout(		        		),
	.ep4_we(		        		),
	.ep4_full(		1'b0     		),
	// End point 4 'IN' FIFO i/f
	.ep4_din(		8'h0			),
	.ep4_re(					),
	.ep4_empty(		1'b0			),
	.ep4_bf_en(		1'b0			),
	.ep4_bf_size(		7'h0			),
 
	// End point 5 configuration
	.ep5_cfg(	`INT  | `IN  | 14'd064		),
	// End point 5 'OUT' FIFO i/f
	.ep5_dout(					),
	.ep5_we(					),
	.ep5_full(		1'b0			),
	// End point 5 'IN' FIFO i/f
	.ep5_din(		8'h0     		),
	.ep5_re(				        ),
	.ep5_empty(		1'b0     		),
	.ep5_bf_en(		1'b0			),
	.ep5_bf_size(		7'h0			),
 
	// End point 6 configuration
	.ep6_cfg(		14'h00			),
	// End point 6 'OUT' FIFO i/f
	.ep6_dout(					),
	.ep6_we(					),
	.ep6_full(		1'b0			),
	// End point 6 'IN' FIFO i/f
	.ep6_din(		8'h0			),
	.ep6_re(					),
	.ep6_empty(		1'b0			),
	.ep6_bf_en(		1'b0			),
	.ep6_bf_size(		7'h0			),
 
	// End point 7 configuration
	.ep7_cfg(		14'h00			),
	// End point 7 'OUT' FIFO i/f
	.ep7_dout(					),
	.ep7_we(					),
	.ep7_full(		1'b0			),
	// End point 7 'IN' FIFO i/f
	.ep7_din(		8'h0			),
	.ep7_re(					),
	.ep7_empty(		1'b0			),
	.ep7_bf_en(		1'b0			),
	.ep7_bf_size(		7'h0			),
 
        // Uart Line Interface
	.uart_txd     (uart_txd),
	.uart_rxd    (uart_rxd)
 
	);

uart_agent u_uart_agent(
	.test_clk (usb_48mhz_clk),
	.sin     (uart_rxd),
	.sout    (uart_txd)
     );


usb_agent u_usb_agent();

test_control test_control();


task wb_user_core_write;
input [31:0] address;
input [31:0] data;
begin
  repeat (1) @(posedge wb_clk);
  #1;
  wbd_ext_adr_i =address;  // address
  wbd_ext_we_i  ='h1;  // write
  wbd_ext_dat_i =data;  // data output
  wbd_ext_sel_i ='hF;  // byte enable
  wbd_ext_cyc_i ='h1;  // strobe/request
  wbd_ext_stb_i ='h1;  // strobe/request
  wait(wbd_ext_ack_o == 1);
  repeat (1) @(posedge wb_clk);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  //$display("STATUS: WB USER ACCESS WRITE Address : 0x%x, Data : 0x%x",address,data);
  repeat (2) @(posedge wb_clk);
end
endtask

task  wb_user_core_read;
input [31:0] address;
output [31:0] data;
reg    [31:0] data;
begin
  repeat (1) @(posedge wb_clk);
  #1;
  wbd_ext_adr_i =address;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='0;  // data output
  wbd_ext_sel_i ='hF;  // byte enable
  wbd_ext_cyc_i ='h1;  // strobe/request
  wbd_ext_stb_i ='h1;  // strobe/request
  wait(wbd_ext_ack_o == 1);
  data  = wbd_ext_dat_o;  
  repeat (1) @(posedge wb_clk);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  //$display("STATUS: WB USER ACCESS READ  Address : 0x%x, Data : 0x%x",address,data);
  repeat (2) @(posedge wb_clk);
end
endtask

task  wb_user_core_read_check;
input [31:0] address;
output [31:0] data;
input [31:0] cmp_data;
reg    [31:0] data;
begin
  repeat (1) @(posedge wb_clk);
  #1;
  wbd_ext_adr_i =address;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='0;  // data output
  wbd_ext_sel_i ='hF;  // byte enable
  wbd_ext_cyc_i ='h1;  // strobe/request
  wbd_ext_stb_i ='h1;  // strobe/request
  wait(wbd_ext_ack_o == 1);
  data  = wbd_ext_dat_o;  
  repeat (1) @(posedge wb_clk);
  #1;
  wbd_ext_cyc_i ='h0;  // strobe/request
  wbd_ext_stb_i ='h0;  // strobe/request
  wbd_ext_adr_i ='h0;  // address
  wbd_ext_we_i  ='h0;  // write
  wbd_ext_dat_i ='h0;  // data output
  wbd_ext_sel_i ='h0;  // byte enable
  if(data !== cmp_data) begin
     $display("ERROR : WB USER ACCESS READ  Address : 0x%x, Exd: 0x%x Rxd: 0x%x ",address,cmp_data,data);
     tb_top.test_fail = 1;
  end else begin
     $display("STATUS: WB USER ACCESS READ  Address : 0x%x, Data : 0x%x",address,data);
  end
  repeat (2) @(posedge wb_clk);
end
endtask



endmodule


`default_nettype wire
