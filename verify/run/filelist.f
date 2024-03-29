+define+USBF_DEBUG \
-sv  \
+defin+USB_VERBOSE_DEBUG  \
+incdir+../usb2uart/usb1_core \
+incdir+../tests \
+incdir+../usb2uart/lib \
+incdir+../../src/includes \
timescale.v \
../tb/tb_top.v                \
../tb/test_control.v      \
../agents/uart/uart_agent.v \
../agents/usb/usb_agents.v \
../usb2uart/usb1_core/usb1_core.v \
../usb2uart/usb1_core/usb1_crc16.v \
../usb2uart/usb1_core/usb1_crc5.v \
../usb2uart/usb1_core/usb1_ctrl.v \
../usb2uart/usb1_core/usb1_fifo2.v \
../usb2uart/usb1_core/usb1_idma.v \
../usb2uart/usb1_core/usb1_pa.v \
../usb2uart/usb1_core/usb1_pd.v \
../usb2uart/usb1_core/usb1_pe.v \
../usb2uart/usb1_core/usb1_pl.v \
../usb2uart/usb1_core/usb1_rom1.v \
../usb2uart/usb1_core/usb1_utmi_if.v  \
../usb2uart/usb1_phy/usb_phy.v \
../usb2uart/usb1_phy/usb_rx_phy.v \
../usb2uart/usb1_phy/usb_tx_phy.v \
../usb2uart/lib/generic_fifo_sc_a.v \
../usb2uart/lib/generic_dpram.v \
../usb2uart/lib/sync_fifo.v \
../usb2uart/uart_core/uart_core.v \
../usb2uart/uart_core/uart_txfsm.v \
../usb2uart/uart_core/uart_rxfsm.v \
../usb2uart/uart_core/uart_cfg.v \
../usb2uart/lib/clk_ctl.v \
../usb2uart/lib/double_sync_high.v \
../usb2uart/lib/double_sync_low.v \
../usb2uart/lib/registers.v  \
../usb2uart/core/usb2uart.v  \
../../src/core/usbh_core.sv  \
../../src/core/usbh_crc16.sv \
../../src/core/usbh_crc5.sv  \
../../src/core/usbh_fifo.sv  \
../../src/core/usbh_sie.sv   \
../../src/phy/usb_fs_phy.v   \
../../src/phy/usb_transceiver.v \
../../src/lib/async_wb.sv    \
../../src/lib/async_fifo.sv  \
../../src/top/usb1_host.sv 
