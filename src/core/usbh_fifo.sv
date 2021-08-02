//////////////////////////////////////////////////////////////////////////////
// SPDX-FileCopyrightText: 2021 , Dinesh Annayya                          
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
// SPDX-FileContributor: Created by Dinesh Annayya <dinesha@opencores.org>
//
//-----------------------------------------------------------------
//                     USB Full Speed Host
//                           V0.6
//                     Ultra-Embedded.com
//                     Copyright 2015-2020
//
//                 Email: admin@ultra-embedded.com
//
//                         License: GPL
// If you would like a version with a more permissive license for
// use in closed source commercial applications please contact me
// for details.
//-----------------------------------------------------------------
//
// This file is open source HDL; you can redistribute it and/or 
// modify it under the terms of the GNU General Public License as 
// published by the Free Software Foundation; either version 2 of 
// the License, or (at your option) any later version.
//
// This file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public 
// License along with this file; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
// USA
//-----------------------------------------------------------------

//-----------------------------------------------------------------
//                          Generated File
//-----------------------------------------------------------------

module usbh_fifo
(
    // Inputs
     input           clk_i
    ,input           rstn_i
    ,input  [  7:0]  data_i
    ,input           push_i
    ,input           pop_i
    ,input           flush_i

    // Outputs
    ,output          full_o
    ,output          empty_o
    ,output [  7:0]  data_o
);



parameter WIDTH   = 8;
parameter DEPTH   = 64;
parameter ADDR_W  = 6;

//-----------------------------------------------------------------
// Local Params
//-----------------------------------------------------------------
localparam COUNT_W = ADDR_W + 1;

//-----------------------------------------------------------------
// Registers
//-----------------------------------------------------------------
reg [WIDTH-1:0]         ram [DEPTH-1:0];
reg [ADDR_W-1:0]        rd_ptr;
reg [ADDR_W-1:0]        wr_ptr;
reg [COUNT_W-1:0]       count;

//-----------------------------------------------------------------
// Sequential
//-----------------------------------------------------------------
always @ (posedge clk_i or negedge rstn_i)
if (!rstn_i)
begin
    count   <= {(COUNT_W) {1'b0}};
    rd_ptr  <= {(ADDR_W) {1'b0}};
    wr_ptr  <= {(ADDR_W) {1'b0}};
end
else
begin

    if (flush_i)
    begin
        count   <= {(COUNT_W) {1'b0}};
        rd_ptr  <= {(ADDR_W) {1'b0}};
        wr_ptr  <= {(ADDR_W) {1'b0}};
    end

    // Push
    if (push_i & ~full_o)
    begin
        ram[wr_ptr] <= data_i;
        wr_ptr      <= wr_ptr + 1;
    end

    // Pop
    if (pop_i & ~empty_o)
    begin
        rd_ptr      <= rd_ptr + 1;
    end

    // Count up
    if ((push_i & ~full_o) & ~(pop_i & ~empty_o))
    begin
        count <= count + 1;
    end
    // Count down
    else if (~(push_i & ~full_o) & (pop_i & ~empty_o))
    begin
        count <= count - 1;
    end
end

//-------------------------------------------------------------------
// Combinatorial
//-------------------------------------------------------------------
/* verilator lint_off WIDTH */
assign full_o    = (count == DEPTH);
assign empty_o   = (count == 0);
/* verilator lint_on WIDTH */

assign data_o    = ram[rd_ptr];


endmodule
