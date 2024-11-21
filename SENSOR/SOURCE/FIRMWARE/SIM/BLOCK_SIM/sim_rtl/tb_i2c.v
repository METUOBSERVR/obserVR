// ENG: IBRAHIM METE BINGOL 
// MODULE NAME: CLOCK MANAGEMENT SIMULATION
// MODULE DESCRIPTION: Testing clock management
// Release: 1.00
`timescale 1ns / 100ps
module tb_i2c ();
    

reg A = 0;
wire B;

initial 
begin
    # 500;
    A = 1;    
end

i2c UUT(
    .A(A),
    .B(B)
);

endmodule