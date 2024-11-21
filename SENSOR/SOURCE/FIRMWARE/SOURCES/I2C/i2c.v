module i2c (
    input   A,
    output  B
);

i2c_master_driver your_instance_name(
    .I_CLK(I_CLK_i), //input I_CLK
    .I_RESETN(I_RESETN_i), //input I_RESETN
    .I_TX_EN(I_TX_EN_i), //input I_TX_EN
    .I_WADDR(I_WADDR_i), //input [2:0] I_WADDR
    .I_WDATA(I_WDATA_i), //input [7:0] I_WDATA
    .I_RX_EN(I_RX_EN_i), //input I_RX_EN
    .I_RADDR(I_RADDR_i), //input [2:0] I_RADDR
    .O_RDATA(O_RDATA_o), //output [7:0] O_RDATA
    .O_IIC_INT(O_IIC_INT_o), //output O_IIC_INT
    .SCL(SCL_io), //inout SCL
    .SDA(SDA_io) //inout SDA
);
endmodule 