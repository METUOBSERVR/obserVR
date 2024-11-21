create_wave_config  wave_config.wcfg
add_wave_group rx_management
add_wave_group rx
add_wave_group buffer
add_wave -into rx_management    /tb_rx_management/rx_management_inst_0/*
add_wave -into rx               /tb_rx_management/rx_management_inst_0/rx_inst_0/*
add_wave -into buffer           /tb_rx_management/rx_management_inst_0/uart_buffer_inst_0/*