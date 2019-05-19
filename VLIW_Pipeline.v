module D_ff_reg(input clk, input reset, input regWrite, input decOut1b , input d, output reg q);
    always @ (negedge clk)
    begin
    if(reset==1)
        q=0;
    else
        if(regWrite == 1 && decOut1b==1)
        begin
            q=d;
        end
    end
endmodule

//*********** Registers start



module mux32to1_DM_16bit(input [15:0] outR0, input [15:0] outR1, input [15:0] outR2, input [15:0] outR3,
    input [15:0] outR4, input [15:0] outR5, input [15:0] outR6, input [15:0] outR7,
    input [15:0] outR8, input [15:0] outR9, input [15:0] outR10, input [15:0] outR11,
    input [15:0] outR12, input [15:0] outR13, input [15:0] outR14, input [15:0] outR15,input [15:0] outR16,input [15:0] outR17,input [15:0] outR18,
    input [15:0] outR19,input [15:0] outR20,input [15:0] outR21,input [15:0] outR22,input [15:0] outR23,input [15:0] outR24,input [15:0] outR25,
    input [15:0] outR26,input [15:0] outR27,input [15:0] outR28,input [15:0] outR29,input [15:0] outR30,input [15:0] outR31,
    input [4:0] Sel, output reg [15:0] outBus);

    always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,outR16,
    outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,Sel)
        case(Sel)
            5'd0:outBus=outR0;
            5'd1:outBus=outR1;
            5'd2:outBus=outR2;
            5'd3:outBus=outR3;
            5'd4:outBus=outR4;
            5'd5:outBus=outR5;
            5'd6:outBus=outR6;
            5'd7:outBus=outR7;
            5'd8:outBus=outR8;
            5'd9:outBus=outR9;
            5'd10:outBus=outR10;
            5'd11:outBus=outR11;
            5'd12:outBus=outR12;
            5'd13:outBus=outR13;
            5'd14:outBus=outR14;
            5'd15:outBus=outR15;
            
            5'd16:outBus=outR16;
            5'd17:outBus=outR17;
            5'd18:outBus=outR18;
            5'd19:outBus=outR19;
            5'd20:outBus=outR20;
            5'd21:outBus=outR21;
            5'd22:outBus=outR22;
            5'd23:outBus=outR23;
            5'd24:outBus=outR24;
            5'd25:outBus=outR25;
            5'd26:outBus=outR26;
            5'd27:outBus=outR27;
            5'd28:outBus=outR28;
            5'd29:outBus=outR29;
            5'd30:outBus=outR30;
            5'd31:outBus=outR31;
        endcase
endmodule

module mux2to1_16bits(input [15:0] in0, input [15:0] in1, input sel, output reg [15:0] muxOut);
    always@(in0, in1, sel)
        begin
            case(sel)
                1'b0: muxOut=in0;
                1'b1: muxOut=in1;
            endcase
        end
endmodule

// Data Memory Starts

module DM(input clk, input reset, input memRead,input memWrite, input [4:0] addread_5bits, input [4:0] addwrite_5bits, input [31:0] memDatain, output [15:0] memDataout);


    wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
                    Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,
                    Qout16, Qout17, Qout18, Qout19, Qout20, Qout21, Qout22, Qout23,
                    Qout24, Qout25, Qout26, Qout27, Qout28, Qout29, Qout30, Qout31;


    wire [31:0] decOut;
    decoder5to32dm decdm(addwrite_5bits,decOut);
                    
    register16bit rIM0 (clk, reset,memWrite, decOut[0], memDatain[15:0], Qout0[15:0]); 
    register16bit rIM1 (clk, reset, memWrite, decOut[1], memDatain[31:16], Qout1[15:0]); 
    register16bit rIM2 (clk, reset, memWrite, decOut[2], memDatain[15:0], Qout2[15:0]);  
    register16bit rIM3 (clk, reset, memWrite, decOut[3], memDatain[31:16], Qout3[15:0]); 
    register16bit rIM4 (clk, reset, memWrite, decOut[4], memDatain[15:0], Qout4[15:0]);
    register16bit rIM5 (clk, reset, memWrite, decOut[5], memDatain[31:16], Qout5[15:0]); 
    register16bit rIM6 (clk, reset, memWrite, decOut[6], memDatain[15:0], Qout6[15:0]); 
    register16bit rIM7 (clk, reset, memWrite, decOut[7], memDatain[31:16], Qout7[15:0]); 
    register16bit rIM8 (clk, reset, memWrite, decOut[8], memDatain[15:0], Qout8[15:0]); 
    register16bit rIM9 (clk, reset, memWrite, decOut[9], memDatain[31:16], Qout9[15:0]); 
    register16bit rIM10 (clk, reset, memWrite, decOut[10], memDatain[15:0], Qout10[15:0]);   
    register16bit rIM11 (clk, reset, memWrite, decOut[11], memDatain[31:16], Qout11[15:0]); 
    register16bit rIM12 (clk, reset, memWrite, decOut[12], memDatain[15:0], Qout12[15:0]); 
    register16bit rIM13 (clk, reset, memWrite, decOut[13], memDatain[31:16], Qout13[15:0]); 
    register16bit rIM14 (clk, reset, memWrite, decOut[14], memDatain[15:0], Qout14[15:0]);   
    register16bit rIM15 (clk, reset, memWrite, decOut[15], memDatain[31:16], Qout15[15:0]);
    register16bit rIM16 (clk, reset, memWrite, decOut[16], memDatain[15:0], Qout16[15:0]);
    register16bit rIM17 (clk, reset, memWrite, decOut[17], memDatain[31:16], Qout17[15:0]);
    register16bit rIM18 (clk, reset, memWrite, decOut[18], memDatain[15:0], Qout18[15:0]);
    register16bit rIM19 (clk, reset, memWrite, decOut[19], memDatain[31:16], Qout19[15:0]);
    register16bit rIM20 (clk, reset, memWrite, decOut[20], memDatain[15:0], Qout20[15:0]);
    register16bit rIM21 (clk, reset, memWrite, decOut[21], memDatain[31:16], Qout21[15:0]);
    register16bit rIM22 (clk, reset, memWrite, decOut[22], memDatain[15:0], Qout22[15:0]);
    register16bit rIM23 (clk, reset, memWrite, decOut[23], memDatain[31:16], Qout23[15:0]);
    register16bit rIM24 (clk, reset, memWrite, decOut[24], memDatain[15:0], Qout24[15:0]);
    register16bit rIM25 (clk, reset,memWrite,  decOut[25], memDatain[31:16], Qout25[15:0]);       
    register16bit rIM26 (clk, reset, memWrite, decOut[26], memDatain[15:0], Qout26[15:0]);   
    register16bit rIM27 (clk, reset, memWrite, decOut[27], memDatain[31:16], Qout27[15:0]);   
    register16bit rIM28 (clk, reset, memWrite, decOut[28], memDatain[15:0], Qout28[15:0]); 
    register16bit rIM29 (clk, reset, memWrite, decOut[29], memDatain[31:16], Qout29[15:0]); 
    register16bit rIM30 (clk, reset,memWrite,  decOut[30], memDatain[15:0], Qout30[15:0]);   
    register16bit rIM31 (clk, reset, memWrite, decOut[31], memDatain[31:16], Qout31[15:0]);       
    
    wire [15:0] muxinput1;
    mux32to1_DM_16bit mDM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,
    Qout16,Qout17,Qout18,Qout19,Qout20,Qout21,Qout22,Qout23,Qout24,Qout25,Qout26,Qout27,Qout28,Qout29,Qout30,Qout31,
    addread_5bits[4:0],muxinput1);
        
    
    mux2to1_16bits mux1(16'd0,muxinput1,memRead,memDataout);
    
endmodule

module register32bit(input clk, input reset, input writeEn, input decOut1b, input [31:0] writeData, output  [31:0] outR);

    D_ff_reg d0(clk, reset, writeEn, decOut1b, writeData[0], outR[0]);
    D_ff_reg d1(clk, reset, writeEn, decOut1b, writeData[1], outR[1]);
    D_ff_reg d2(clk, reset, writeEn, decOut1b, writeData[2], outR[2]);
    D_ff_reg d3(clk, reset, writeEn, decOut1b, writeData[3], outR[3]);
    D_ff_reg d4(clk, reset, writeEn, decOut1b, writeData[4], outR[4]);
    D_ff_reg d5(clk, reset, writeEn, decOut1b, writeData[5], outR[5]);
    D_ff_reg d6(clk, reset, writeEn, decOut1b, writeData[6], outR[6]);
    D_ff_reg d7(clk, reset, writeEn, decOut1b, writeData[7], outR[7]);
    D_ff_reg d8(clk, reset, writeEn, decOut1b, writeData[8], outR[8]);
    D_ff_reg d9(clk, reset, writeEn, decOut1b, writeData[9], outR[9]);
    D_ff_reg d10(clk, reset, writeEn, decOut1b, writeData[10], outR[10]);
    D_ff_reg d11(clk, reset, writeEn, decOut1b, writeData[11], outR[11]);
    D_ff_reg d12(clk, reset, writeEn, decOut1b, writeData[12], outR[12]);
    D_ff_reg d13(clk, reset, writeEn, decOut1b, writeData[13], outR[13]);
    D_ff_reg d14(clk, reset, writeEn, decOut1b, writeData[14], outR[14]);
    D_ff_reg d15(clk, reset, writeEn, decOut1b, writeData[15], outR[15]);
    D_ff_reg d16(clk, reset, writeEn, decOut1b, writeData[16], outR[16]);
    D_ff_reg d17(clk, reset, writeEn, decOut1b, writeData[17], outR[17]);
    D_ff_reg d18(clk, reset, writeEn, decOut1b, writeData[18], outR[18]);
    D_ff_reg d19(clk, reset, writeEn, decOut1b, writeData[19], outR[19]);
    D_ff_reg d20(clk, reset, writeEn, decOut1b, writeData[20], outR[20]);
    D_ff_reg d21(clk, reset, writeEn, decOut1b, writeData[21], outR[21]);
    D_ff_reg d22(clk, reset, writeEn, decOut1b, writeData[22], outR[22]);
    D_ff_reg d23(clk, reset, writeEn, decOut1b, writeData[23], outR[23]);
    D_ff_reg d24(clk, reset, writeEn, decOut1b, writeData[24], outR[24]);
    D_ff_reg d25(clk, reset, writeEn, decOut1b, writeData[25], outR[25]);
    D_ff_reg d26(clk, reset, writeEn, decOut1b, writeData[26], outR[26]);
    D_ff_reg d27(clk, reset, writeEn, decOut1b, writeData[27], outR[27]);
    D_ff_reg d28(clk, reset, writeEn, decOut1b, writeData[28], outR[28]);
    D_ff_reg d29(clk, reset, writeEn, decOut1b, writeData[29], outR[29]);
    D_ff_reg d30(clk, reset, writeEn, decOut1b, writeData[30], outR[30]);
    D_ff_reg d31(clk, reset, writeEn, decOut1b, writeData[31], outR[31]);
    
endmodule

module register16bit(input clk, input reset, input writeEn, input decOut1b, input [15:0] writeData, output  [15:0] outR);

    D_ff_reg d0(clk, reset, writeEn, decOut1b, writeData[0], outR[0]);
    D_ff_reg d1(clk, reset, writeEn, decOut1b, writeData[1], outR[1]);
    D_ff_reg d2(clk, reset, writeEn, decOut1b, writeData[2], outR[2]);
    D_ff_reg d3(clk, reset, writeEn, decOut1b, writeData[3], outR[3]);
    D_ff_reg d4(clk, reset, writeEn, decOut1b, writeData[4], outR[4]);
    D_ff_reg d5(clk, reset, writeEn, decOut1b, writeData[5], outR[5]);
    D_ff_reg d6(clk, reset, writeEn, decOut1b, writeData[6], outR[6]);
    D_ff_reg d7(clk, reset, writeEn, decOut1b, writeData[7], outR[7]);
    D_ff_reg d8(clk, reset, writeEn, decOut1b, writeData[8], outR[8]);
    D_ff_reg d9(clk, reset, writeEn, decOut1b, writeData[9], outR[9]);
    D_ff_reg d10(clk, reset, writeEn, decOut1b, writeData[10], outR[10]);
    D_ff_reg d11(clk, reset, writeEn, decOut1b, writeData[11], outR[11]);
    D_ff_reg d12(clk, reset, writeEn, decOut1b, writeData[12], outR[12]);
    D_ff_reg d13(clk, reset, writeEn, decOut1b, writeData[13], outR[13]);
    D_ff_reg d14(clk, reset, writeEn, decOut1b, writeData[14], outR[14]);
    D_ff_reg d15(clk, reset, writeEn, decOut1b, writeData[15], outR[15]);


endmodule


module register2bit(input clk, input reset, input writeEn, input decOut1b, input [1:0] writeData, output  [1:0] outR);

    D_ff_reg d0(clk, reset, writeEn, decOut1b, writeData[0], outR[0]);
    D_ff_reg d1(clk, reset, writeEn, decOut1b, writeData[1], outR[1]);
    
endmodule

module register3bit(input clk, input reset, input writeEn, input decOut1b, input [2:0] writeData, output  [2:0] outR);

    D_ff_reg d0(clk, reset, writeEn, decOut1b, writeData[0], outR[0]);
    D_ff_reg d1(clk, reset, writeEn, decOut1b, writeData[1], outR[1]);
    D_ff_reg d2(clk, reset, writeEn, decOut1b, writeData[2], outR[2]);
    
    
endmodule


module register5bit(input clk, input reset, input writeEn, input decOut1b, input [4:0] writeData, output  [4:0] outR);

    D_ff_reg d0(clk, reset, writeEn, decOut1b, writeData[0], outR[0]);
    D_ff_reg d1(clk, reset, writeEn, decOut1b, writeData[1], outR[1]);
    D_ff_reg d2(clk, reset, writeEn, decOut1b, writeData[2], outR[2]);
    D_ff_reg d3(clk, reset, writeEn, decOut1b, writeData[3], outR[3]);
    D_ff_reg d4(clk, reset, writeEn, decOut1b, writeData[4], outR[4]);
    
endmodule

//*******************888888  Registers end

// Instruction Memory Design

module D_ff_IM(input clk, input reset, input d, output reg q);
    always@(reset or posedge clk)
    if(reset)
        q=d;
endmodule

module register_IM(input clk, input reset, input [15:0] d_in, output [15:0] q_out);
    D_ff_IM dIM0 (clk, reset, d_in[0], q_out[0]);
    D_ff_IM dIM1 (clk, reset, d_in[1], q_out[1]);
    D_ff_IM dIM2 (clk, reset, d_in[2], q_out[2]);
    D_ff_IM dIM3 (clk, reset, d_in[3], q_out[3]);
    D_ff_IM dIM4 (clk, reset, d_in[4], q_out[4]);
    D_ff_IM dIM5 (clk, reset, d_in[5], q_out[5]);
    D_ff_IM dIM6 (clk, reset, d_in[6], q_out[6]);
    D_ff_IM dIM7 (clk, reset, d_in[7], q_out[7]);
    D_ff_IM dIM8 (clk, reset, d_in[8], q_out[8]);
    D_ff_IM dIM9 (clk, reset, d_in[9], q_out[9]);
    D_ff_IM dIM10 (clk, reset, d_in[10], q_out[10]);
    D_ff_IM dIM11 (clk, reset, d_in[11], q_out[11]);
    D_ff_IM dIM12 (clk, reset, d_in[12], q_out[12]);
    D_ff_IM dIM13 (clk, reset, d_in[13], q_out[13]);
    D_ff_IM dIM14 (clk, reset, d_in[14], q_out[14]);
    D_ff_IM dIM15 (clk, reset, d_in[15], q_out[15]);
endmodule


module mux32to1_IM(input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
    outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
    input [4:0] Sel, output reg [63:0] outBus );
    
    always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,
        outR16,outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,
        Sel)
        case (Sel)
            5'd0: outBus = {outR3,outR2,outR1,outR0};
            5'd1: outBus = {outR4,outR3,outR2,outR1};
            5'd2: outBus = {outR5,outR4,outR3,outR2};
            5'd3: outBus = {outR6,outR5,outR4,outR3};
            5'd4: outBus = {outR7,outR6,outR5,outR4};
            5'd5: outBus = {outR8,outR7,outR6,outR5};
            5'd6: outBus = {outR9,outR8,outR7,outR6};
            5'd7: outBus = {outR10,outR9,outR8,outR7};
            5'd8: outBus = {outR11,outR10,outR9,outR8};
            5'd9: outBus = {outR12,outR11,outR10,outR9};
            5'd10: outBus = {outR13,outR12,outR11,outR10};
            5'd11: outBus = {outR14,outR13,outR12,outR11};
            5'd12: outBus = {outR15,outR14,outR13,outR12};
            5'd13: outBus = {outR16,outR15,outR14,outR13};
            5'd14: outBus = {outR17,outR16,outR15,outR14};
            5'd15: outBus = {outR18,outR17,outR16,outR15};
            5'd16: outBus = {outR19,outR18,outR17,outR16};
            5'd17: outBus = {outR20,outR19,outR18,outR17};
            5'd18: outBus = {outR21,outR20,outR19,outR18};
            5'd19: outBus = {outR22,outR21,outR20,outR19};
            5'd20: outBus = {outR23,outR22,outR21,outR20};
            5'd21: outBus = {outR24,outR23,outR22,outR21};
            5'd22: outBus = {outR25,outR24,outR23,outR22};
            5'd23: outBus = {outR26,outR25,outR24,outR23};
            5'd24: outBus = {outR27,outR26,outR25,outR24};
            5'd25: outBus = {outR28,outR27,outR26,outR25};
            5'd26: outBus = {outR29,outR28,outR27,outR26};
            5'd27: outBus = {outR30,outR29,outR28,outR27};
            5'd28: outBus = {outR31,outR30,outR29,outR28};
            5'd29: outBus = {32'b0,outR31,outR30,outR29};
            5'd30: outBus = {32'b0,32'b0,outR31,outR30};
            5'd31: outBus = {32'b0,32'b0,32'b0,outR31};
        endcase
endmodule

module IM(input clk, input reset, input [4:0] pc_5bits, output [31:0] IR, output [15:0] IRcomp);
    wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
                    Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,
                    Qout16, Qout17, Qout18, Qout19, Qout20, Qout21, Qout22, Qout23,
                    Qout24, Qout25, Qout26, Qout27, Qout28, Qout29, Qout30, Qout31;
    wire [63:0] IRtemp;
     register_IM rIM0 (clk, reset, 16'hF0B7, Qout0); // lui $1, F
    register_IM rIM1 (clk, reset, 16'h0000, Qout1); 
    register_IM rIM2 (clk, reset, 16'h0000, Qout2); 
    register_IM rIM3 (clk, reset, 16'h0000, Qout3); 
    register_IM rIM4 (clk, reset, 16'h8113, Qout4); //addi $2, $1, 3 
    register_IM rIM5 (clk, reset, 16'h0030, Qout5); 
    register_IM rIM6 (clk, reset, 16'h8186, Qout6);  //c.mv $3,$1
    register_IM rIM7 (clk, reset, 16'h0000, Qout7); 
    register_IM rIM8 (clk, reset, 16'h0113, Qout8); //addi $2, $2, 3 
    register_IM rIM9 (clk, reset, 16'h0031, Qout9); 
    register_IM rIM10 (clk, reset, 16'h8186, Qout10);//c.mv $3, $1
    register_IM rIM11 (clk, reset, 16'h0000, Qout11); 
    register_IM rIM12 (clk, reset, 16'hB5B3, Qout12); //sltu $11, $3, $4 
    register_IM rIM13 (clk, reset, 16'h0000, Qout13); 
    register_IM rIM14 (clk, reset, 16'hC208, Qout14); //sw $4, $2,0
    register_IM rIM15 (clk, reset, 16'h0000, Qout15);
    register_IM rIM16 (clk, reset, 16'h1083, Qout16);//lw $1, $4
    register_IM rIM17 (clk, reset, 16'h0002, Qout17);
    register_IM rIM18 (clk, reset, 16'h8186, Qout18); //c.mv $3, $1
    register_IM rIM19 (clk, reset, 16'h0000, Qout19);
    register_IM rIM20 (clk, reset, 16'h0113, Qout20); //addi $2, $2, 3
    register_IM rIM21 (clk, reset, 16'h0031, Qout21);
    register_IM rIM22 (clk, reset, 16'h848E, Qout22); //c.mv $9, $3
    register_IM rIM23 (clk, reset, 16'h0000, Qout23);
    register_IM rIM24 (clk, reset, 16'hB5B3, Qout24);  // sltu $11, $3, $4 
    register_IM rIM25 (clk, reset, 16'h0000, Qout25);       
    register_IM rIM26 (clk, reset, 16'h3F91, Qout26);  
    register_IM rIM27 (clk, reset, 16'h0000, Qout27);   
    register_IM rIM28 (clk, reset, 16'h8186, Qout28); // sltu $11, $3, $4 
    register_IM rIM29 (clk, reset, 16'h0000, Qout29); 
    register_IM rIM30 (clk, reset, 16'h3F91, Qout30);     //jalr -28d
    register_IM rIM31 (clk, reset, 16'h0000, Qout31);      
    mux32to1_IM mIM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,
        Qout16,Qout17,Qout18,Qout19,Qout20,Qout21,Qout22,Qout23,Qout24,Qout25,Qout26,Qout27,Qout28,Qout29,Qout30,Qout31,
        pc_5bits[4:0],IRtemp);
    assign IR = IRtemp[31:0];
    assign IRcomp = IRtemp[47:32];
endmodule

//Instruction Memory Design Ends

// PC ADDER start


module pc_adder(input [31:0] in1, input [31:0] in2, output reg [31:0] adder_out);
    always@(in1 or in2)
    begin
        adder_out = in1 + in2;
    end
endmodule

// PC ADDER end

module Dffvliw(input clk, input reset,input writeEn, input writeEnc, input decOut, input decOutc, input d, input dc, output reg q );

    always@(negedge clk)
        begin
            if(reset==1)
                q = 1'b0;
            else if(writeEn&decOut==1'b1)
                q = d;
            else if(writeEnc&decOutc==1'b1)
                q = dc;

    end


endmodule


module registervliw(input clk, input reset, input writeEn, input writeEnc, input decOut, input decOutc, input [31:0] writeData, input [31:0] writeDatac, output [31:0] outdata);

     Dffvliw d0(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[0], writeDatac[0], outdata[0]);
    Dffvliw d1(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[1], writeDatac[1], outdata[1]);
    Dffvliw d2(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[2], writeDatac[2], outdata[2]);
    Dffvliw d3(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[3], writeDatac[3], outdata[3]);
    Dffvliw d4(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[4], writeDatac[4], outdata[4]);
    Dffvliw d5(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[5], writeDatac[5], outdata[5]);
    Dffvliw d6(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[6], writeDatac[6], outdata[6]);
    Dffvliw d7(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[7], writeDatac[7], outdata[7]);
    Dffvliw d8(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[8], writeDatac[8], outdata[8]);
    Dffvliw d9(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[9], writeDatac[9], outdata[9]);
    Dffvliw d10(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[10], writeDatac[10], outdata[10]);
    Dffvliw d11(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[11], writeDatac[11], outdata[11]);
    Dffvliw d12(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[12], writeDatac[12], outdata[12]);
    Dffvliw d13(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[13], writeDatac[13], outdata[13]);
    Dffvliw d14(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[14], writeDatac[14], outdata[14]);
    Dffvliw d15(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[15], writeDatac[15], outdata[15]);
    Dffvliw d16(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[16], writeDatac[16], outdata[16]);
    Dffvliw d17(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[17], writeDatac[17], outdata[17]);
    Dffvliw d18(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[18], writeDatac[18], outdata[18]);
    Dffvliw d19(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[19], writeDatac[19], outdata[19]);
    Dffvliw d20(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[20], writeDatac[20], outdata[20]);
    Dffvliw d21(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[21], writeDatac[21], outdata[21]);
    Dffvliw d22(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[22], writeDatac[22], outdata[22]);
    Dffvliw d23(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[23], writeDatac[23], outdata[23]);
    Dffvliw d24(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[24], writeDatac[24], outdata[24]);
    Dffvliw d25(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[25], writeDatac[25], outdata[25]);
    Dffvliw d26(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[26], writeDatac[26], outdata[26]);
    Dffvliw d27(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[27], writeDatac[27], outdata[27]);
    Dffvliw d28(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[28], writeDatac[28], outdata[28]);
    Dffvliw d29(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[29], writeDatac[29], outdata[29]);
    Dffvliw d30(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[30], writeDatac[30], outdata[30]);
    Dffvliw d31(clk, reset, writeEn, writeEnc, decOut, decOutc, writeData[31], writeDatac[31], outdata[31]);    
    
    

endmodule


module registerSet(input clk, input reset, input writeEn,input writeEnc, input [31:0] decOut, input [31:0] decOutc, input [31:0] writeData, input [31:0] writeDatac,
    output [31:0] outR0, output [31:0] outR1, output [31:0] outR2, output [31:0] outR3,
    output [31:0] outR4, output [31:0] outR5, output [31:0] outR6, output [31:0] outR7,
    output [31:0] outR8, output [31:0] outR9, output [31:0] outR10, output [31:0] outR11,
    output [31:0] outR12, output [31:0] outR13, output [31:0] outR14, output [31:0] outR15,
    output [31:0] outR16,output [31:0] outR17,output [31:0] outR18,output [31:0] outR19,output [31:0] outR20,output [31:0] outR21,
    output [31:0] outR22,output [31:0] outR23,output [31:0] outR24,output [31:0] outR25,
    output [31:0] outR26,output [31:0] outR27,output [31:0] outR28,output [31:0] outR29,
    output [31:0] outR30,output [31:0] outR31);


   registervliw   register0(clk, reset, writeEn, writeEnc, decOut[0], decOutc[0],writeData, writeDatac,  outR0);
    registervliw register1(clk, reset, writeEn, writeEnc, decOut[1], decOutc[1],writeData, writeDatac,  outR1);
    registervliw register2(clk, reset, writeEn, writeEnc, decOut[2], decOutc[2],writeData, writeDatac,  outR2);
    registervliw register3(clk, reset, writeEn, writeEnc, decOut[3], decOutc[3],writeData, writeDatac,  outR3);
    registervliw register4(clk, reset, writeEn, writeEnc, decOut[4], decOutc[4],writeData, writeDatac,  outR4);
    registervliw register5(clk, reset, writeEn, writeEnc, decOut[5], decOutc[5],writeData, writeDatac,  outR5);
    registervliw register6(clk, reset, writeEn, writeEnc, decOut[6], decOutc[6],writeData, writeDatac,  outR6);
    registervliw register7(clk, reset, writeEn, writeEnc, decOut[7], decOutc[7],writeData, writeDatac,  outR7);
    registervliw register8(clk, reset, writeEn, writeEnc, decOut[8], decOutc[8],writeData, writeDatac,  outR8);
    registervliw register9(clk, reset, writeEn, writeEnc, decOut[9], decOutc[9],writeData, writeDatac,  outR9);
    registervliw register10(clk, reset, writeEn, writeEnc, decOut[10], decOutc[10],writeData, writeDatac,  outR10);
    registervliw register11(clk, reset, writeEn, writeEnc, decOut[11], decOutc[11],writeData, writeDatac,  outR11);
    registervliw register12(clk, reset, writeEn, writeEnc, decOut[12], decOutc[12],writeData, writeDatac,  outR12);
    registervliw register13(clk, reset, writeEn, writeEnc, decOut[13], decOutc[13],writeData, writeDatac,  outR13);
    registervliw register14(clk, reset, writeEn, writeEnc, decOut[14], decOutc[14],writeData, writeDatac,  outR14);
    registervliw register15(clk, reset, writeEn, writeEnc, decOut[15], decOutc[15],writeData, writeDatac,  outR15);
    
    registervliw register16(clk, reset, writeEn, writeEnc, decOut[16], decOutc[16],writeData, writeDatac,  outR16);
    registervliw register17(clk, reset, writeEn, writeEnc, decOut[17], decOutc[17],writeData, writeDatac,  outR17);
    registervliw register18(clk, reset, writeEn, writeEnc, decOut[18], decOutc[18],writeData, writeDatac,  outR18);
    registervliw register19(clk, reset, writeEn, writeEnc, decOut[19], decOutc[19],writeData, writeDatac,  outR19);
    registervliw register20(clk, reset, writeEn, writeEnc, decOut[20], decOutc[20],writeData, writeDatac,  outR20);
    registervliw register21(clk, reset, writeEn, writeEnc, decOut[21], decOutc[21],writeData, writeDatac,  outR21);
    registervliw register22(clk, reset, writeEn, writeEnc, decOut[22], decOutc[22],writeData, writeDatac,  outR22);
    registervliw register23(clk, reset, writeEn, writeEnc, decOut[23], decOutc[23],writeData, writeDatac,  outR23);
    registervliw register24(clk, reset, writeEn, writeEnc, decOut[24], decOutc[24],writeData, writeDatac,  outR24);
    registervliw register25(clk, reset, writeEn, writeEnc, decOut[25], decOutc[25],writeData, writeDatac,  outR25);
    registervliw register26(clk, reset, writeEn, writeEnc, decOut[26], decOutc[26],writeData, writeDatac,  outR26);
    registervliw register27(clk, reset, writeEn, writeEnc, decOut[27], decOutc[27],writeData, writeDatac,  outR27);
    registervliw register28(clk, reset, writeEn, writeEnc, decOut[28], decOutc[28],writeData, writeDatac,  outR28);
    registervliw register29(clk, reset, writeEn, writeEnc, decOut[29], decOutc[29],writeData, writeDatac,  outR29);
    registervliw register30(clk, reset, writeEn, writeEnc, decOut[30], decOutc[30],writeData, writeDatac,  outR30);
    registervliw register31(clk, reset, writeEn, writeEnc, decOut[31], decOutc[31],writeData, writeDatac,  outR31);

 
    
    
endmodule


module decoder5to32(input [4:0] destReg, output reg [31:0] decOut);
    always @(destReg)
        begin
            decOut = 32'd0;
            case(destReg)
                5'd0: decOut[0] = 1'b1;                                                                  
                5'd1: decOut[1] = 1'b1;                                                                                       
                5'd2: decOut[2] = 1'b1;                                                                                       
                5'd3: decOut[3] = 1'b1;                                                                                       
                5'd4: decOut[4] = 1'b1;                                                                                       
                5'd5: decOut[5] = 1'b1;                                                                                       
                5'd6: decOut[6] = 1'b1;                                                                                       
                5'd7: decOut[7] = 1'b1;                                                                                       
                5'd8: decOut[8] = 1'b1;                                                                                       
                5'd9: decOut[9] = 1'b1;                                                                                       
                5'd10: decOut[10] = 1'b1;                                                                                       
                5'd11: decOut[11] = 1'b1;                                                                                       
                5'd12: decOut[12] = 1'b1;                                                                                       
                5'd13: decOut[13] = 1'b1;                                                                                       
                5'd14: decOut[14] = 1'b1;                                                                                       
                5'd15: decOut[15] = 1'b1;
                5'd16: decOut[16] = 1'b1;                                                                  
                5'd17: decOut[17] = 1'b1;                                                                                       
                5'd18: decOut[18] = 1'b1;                                                                                       
                5'd19: decOut[19] = 1'b1;                                                                                       
                5'd20: decOut[20] = 1'b1;                                                                                       
                5'd21: decOut[21] = 1'b1;                                                                                       
                5'd22: decOut[22] = 1'b1;                                                                                       
                5'd23: decOut[23] = 1'b1;                                                                                       
                5'd24: decOut[24] = 1'b1;                                                                                       
                5'd25: decOut[25] = 1'b1;                                                                                       
                5'd26: decOut[26] = 1'b1;                                                                                       
                5'd27: decOut[27] = 1'b1;                                                                                       
                5'd28: decOut[28] = 1'b1;                                                                                       
                5'd29: decOut[29] = 1'b1;                                                                                       
                5'd30: decOut[30] = 1'b1;                                                                                       
                5'd31: decOut[31] = 1'b1;                                                                                        
            endcase
        end
endmodule


module decoder5to32dm(input [4:0] destReg, output reg [31:0] decOut);
    always @(destReg)
        begin
            decOut = 32'd0;
            case(destReg)
                5'd0: decOut[1:0] = 2'd3;                                                                  
                5'd1: decOut[2:1] = 2'd3;                                                                                       
                5'd2: decOut[3:2] = 2'd3;                                                                                       
                5'd3: decOut[4:3] = 2'd3;                                                                                       
                5'd4: decOut[5:4] = 2'd3;                                                                                       
                5'd5: decOut[6:5] = 2'd3;                                                                                       
                5'd6: decOut[7:6] = 2'd3;                                                                                       
                5'd7: decOut[8:7] = 2'd3;                                                                                       
                5'd8: decOut[9:8] = 2'd3;                                                                                       
                5'd9: decOut[10:9] = 2'd3;                                                                                       
                5'd10: decOut[11:10] = 2'd3;                                                                                       
                5'd11: decOut[12:11] = 2'd3;                                                                                       
                5'd12: decOut[13:12] = 2'd3;                                                                                       
                5'd13: decOut[14:13] = 2'd3;                                                                                       
                5'd14: decOut[15:14] = 2'd3;                                                                                       
                5'd15: decOut[16:15] = 2'd3;
                5'd16: decOut[17:16] = 2'd3;                                                                  
                5'd17: decOut[18:17] = 2'd3;                                                                                       
                5'd18: decOut[19:18] = 2'd3;                                                                                       
                5'd19: decOut[20:19] = 2'd3;                                                                                       
                5'd20: decOut[21:20] = 2'd3;                                                                                       
                5'd21: decOut[22:21] = 2'd3;                                                                                       
                5'd22: decOut[23:22] = 2'd3;                                                                                       
                5'd23: decOut[24:23] = 2'd3;                                                                                       
                5'd24: decOut[25:24] = 2'd3;                                                                                       
                5'd25: decOut[26:25] = 2'd3;                                                                                       
                5'd26: decOut[27:26] = 2'd3;                                                                                       
                5'd27: decOut[28:27] = 2'd3;                                                                                       
                5'd28: decOut[29:28] = 2'd3;                                                                                       
                5'd29: decOut[30:29] = 2'd3;                                                                                       
                5'd30: decOut[31:30] = 2'd3;                                                                                       
                5'd31: decOut[31] = 1'b1;                                                                                        
            endcase
        end
endmodule


// ****************************** MUX START

module mux32to1(input [31:0] outR0, input [31:0] outR1, input [31:0] outR2, input [31:0] outR3,
    input [31:0] outR4, input [31:0] outR5, input [31:0] outR6, input [31:0] outR7,
    input [31:0] outR8, input [31:0] outR9, input [31:0] outR10, input [31:0] outR11,
    input [31:0] outR12, input [31:0] outR13, input [31:0] outR14, input [31:0] outR15,input [31:0] outR16,input [31:0] outR17,input [31:0] outR18,
    input [31:0] outR19,input [31:0] outR20,input [31:0] outR21,input [31:0] outR22,input [31:0] outR23,input [31:0] outR24,input [31:0] outR25,
    input [31:0] outR26,input [31:0] outR27,input [31:0] outR28,input [31:0] outR29,input [31:0] outR30,input [31:0] outR31,
    input [4:0] Sel, output reg [31:0] outBus);

    always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,outR16,
    outR17,outR18,outR19,outR20,outR21,outR22,outR23,outR24,outR25,outR26,outR27,outR28,outR29,outR30,outR31,Sel)
        case(Sel)
            5'd0:outBus=outR0;
            5'd1:outBus=outR1;
            5'd2:outBus=outR2;
            5'd3:outBus=outR3;
            5'd4:outBus=outR4;
            5'd5:outBus=outR5;
            5'd6:outBus=outR6;
            5'd7:outBus=outR7;
            5'd8:outBus=outR8;
            5'd9:outBus=outR9;
            5'd10:outBus=outR10;
            5'd11:outBus=outR11;
            5'd12:outBus=outR12;
            5'd13:outBus=outR13;
            5'd14:outBus=outR14;
            5'd15:outBus=outR15;
            
            5'd16:outBus=outR16;
            5'd17:outBus=outR17;
            5'd18:outBus=outR18;
            5'd19:outBus=outR19;
            5'd20:outBus=outR20;
            5'd21:outBus=outR21;
            5'd22:outBus=outR22;
            5'd23:outBus=outR23;
            5'd24:outBus=outR24;
            5'd25:outBus=outR25;
            5'd26:outBus=outR26;
            5'd27:outBus=outR27;
            5'd28:outBus=outR28;
            5'd29:outBus=outR29;
            5'd30:outBus=outR30;
            5'd31:outBus=outR31;
        endcase
endmodule


module mux16to1(input [31:0] outR0, input [31:0] outR1, input [31:0] outR2, input [31:0] outR3,
    input [31:0] outR4, input [31:0] outR5, input [31:0] outR6, input [31:0] outR7,
    input [31:0] outR8, input [31:0] outR9, input [31:0] outR10, input [31:0] outR11,
    input [31:0] outR12, input [31:0] outR13, input [31:0] outR14, input [31:0] outR15,
    input [3:0] Sel, output reg [31:0] outBus);

    always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15,Sel)
        case(Sel)
            4'd0:outBus=outR0;
            4'd1:outBus=outR1;
            4'd2:outBus=outR2;
            4'd3:outBus=outR3;
            4'd4:outBus=outR4;
            4'd5:outBus=outR5;
            4'd6:outBus=outR6;
            4'd7:outBus=outR7;
            4'd8:outBus=outR8;
            4'd9:outBus=outR9;
            4'd10:outBus=outR10;
            4'd11:outBus=outR11;
            4'd12:outBus=outR12;
            4'd13:outBus=outR13;
            4'd14:outBus=outR14;
            4'd15:outBus=outR15;
        endcase
endmodule


module mux8to1_32bits(input [31:0] outR0, input [31:0] outR1, input [31:0] outR2, input [31:0] outR3,
    input [31:0] outR4, input [31:0] outR5, input [31:0] outR6, input [31:0] outR7,
    input [2:0] Sel, output reg [31:0] outBus);

    always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,Sel)
        case(Sel)
            3'd0:outBus=outR0;
            3'd1:outBus=outR1;
            3'd2:outBus=outR2;
            3'd3:outBus=outR3;
            3'd4:outBus=outR4;
            3'd5:outBus=outR5;
            3'd6:outBus=outR6;
            3'd7:outBus=outR7;
        endcase
endmodule

module mux4to1_32bits(input [31:0] in0, input [31:0] in1, input [31:0] in2, input [31:0] in3, input [1:0] sel, output reg [31:0] muxOut);
    always@(in0, in1, in2, in3, sel)
        begin
            case(sel)
                2'd0: muxOut=in0;
                2'd1: muxOut=in1;
                2'd2: muxOut=in2;
                2'd3: muxOut=in3;
            endcase
        end
endmodule



module mux2to1_32bits(input [31:0] in0, input [31:0] in1, input sel, output reg [31:0] muxOut);
    always@(in0, in1, sel)
        begin
            case(sel)
                1'b0: muxOut=in0;
                1'b1: muxOut=in1;
            endcase
        end
endmodule


module mux2to1_2bits(input [1:0] in0, input [1:0] in1, input sel, output reg [1:0] muxOut);
    always@(in0, in1, sel)
        begin
            case(sel)
                1'b0: muxOut=in0;
                1'b1: muxOut=in1;
            endcase
        end
endmodule

module mux2to1_1bit(input  in0, input  in1, input sel, output reg  muxOut);
    always@(in0, in1, sel)
        begin
            case(sel)
                1'b0: muxOut=in0;
                1'b1: muxOut=in1;
            endcase
        end
endmodule

module mux2to1_5bits(input [4:0] in0, input [4:0] in1, input sel, output reg [4:0] muxOut);
    always@(in0, in1, sel)
        begin
            case(sel)
                1'b0: muxOut=in0;
                1'b1: muxOut=in1;
            endcase
        end
endmodule


// *************************************** MUX ENDD
module registerFile(input clk, input reset, input regWrite,input regWritec, input [4:0] rs, input [4:0] rt,input [4:0] rd, 
input [4:0] rsc, input [4:0] rtc, input [4:0] rdc, input [31:0] writeData, input [31:0] writeDatac, output [31:0] rsOut, output [31:0] rtOut,
output [31:0] rsOutc, output [31:0] rtOutc);


    wire [31:0] decOut, decOutc;
    wire [31:0] outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15,
     outR16, outR17, outR18, outR19, outR20, outR21, outR22, outR23, outR24, outR25,
     outR26, outR27, outR28, outR29, outR30, outR31;

    
    decoder5to32 d1(rd,decOut);
    decoder5to32 dc(rdc,decOutc);


    registerSet rSet(clk, reset, regWrite,regWritec, decOut, decOutc, writeData,writeDatac,outR0, outR1, outR2, outR3, outR4, outR5, outR6,
     outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15, outR16, outR17,  outR18,  outR19, outR20, 
     outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31);
        
    mux32to1 m0(outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15, outR16, outR17, outR18,
     outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31, rs, rsOut);
    
    mux32to1 m1(outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15,  outR16, outR17, outR18,
     outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31, rt, rtOut);
    


    mux32to1 m2(outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15,  outR16, outR17, outR18,
     outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31, rsc, rsOutc);


    mux32to1 m3(outR0, outR1, outR2, outR3, outR4, outR5, outR6, outR7, outR8, outR9, outR10, outR11, outR12, outR13, outR14, outR15,  outR16, outR17, outR18,
     outR19, outR20, outR21, outR22, outR23, outR24, outR25, outR26, outR27, outR28, outR29, outR30, outR31, rtc, rtOutc);



        
endmodule


// Modules for extensions
module zeroExt5to32(input [4:0] in, output reg [31:0] zeroExtin);
    always@(in)
        zeroExtin={27'b0,in};
endmodule

module signExt12to32(input [11:0] in, output reg [31:0] signExtin);
    always@(in)
        signExtin={{20{in[11]}},in};
endmodule

module signExt16to32(input [15:0] in, output reg [31:0] signExtin);
    always@(in)
        signExtin={{16{in[15]}},in};
endmodule

// Modules ended for Exntension



// ************************ Pipelines start *********************************

module pipelineIFID(input clk, input reset,input ifidWrite, input [31:0] irIn, output [31:0] irOut);
    
	 
    register32bit IR(clk, reset, ifidWrite,1, irIn, irOut);

endmodule


module pipelineIFIDC(input clk, input reset, input ifidWritec, input [15:0] irInc, input [31:0] pcinc_ifid, output [15:0] irOutc, output [31:0] pcoutc_ifid );

        register16bit IRc(clk, reset, ifidWritec,1, irInc, irOutc);
        register32bit PCc(clk, reset, ifidWritec,1, pcinc_ifid, pcoutc_ifid);


endmodule

module pipelineIDEX(input clk, input reset,input [31:0] regrsin, input [31:0] regrtin, input [1:0] aluOpin, input [1:0] aluSrcin, input memReadin,
 input regWritein, input  memtoRegin, input [31:0] shamtin,
            input [4:0] rdin, input [31:0] sextin, input [31:0] luin,input [4:0] rsin, input [4:0] rtin,input luiselectin, output [31:0] regrsout, output [31:0] regrtout,
            output [1:0] aluOpout, output [1:0] aluSrcout, output memReadout, output regWriteout, output  memtoRegout, output [31:0] muxin1, output [31:0] muxin2,
            output [31:0] luiout,  output [4:0] rdout, output [4:0] rsout, output [4:0] rtout, output luiselectout_idex);
            
            
        register32bit regrs(clk, reset, 1,1, regrsin, regrsout);
        register32bit regrt(clk, reset, 1,1, regrtin, regrtout);
        register2bit aluop(clk, reset, 1,1, aluOpin, aluOpout);
        register2bit alusrc(clk,reset,1,1,aluSrcin,aluSrcout);
        D_ff_reg memread(clk,reset,1,1,memReadin,memReadout);
    
        D_ff_reg regwrite(clk,reset,1,1,regWritein,regWriteout);
        D_ff_reg memtoreg(clk,reset,1,1,memtoRegin,memtoRegout);
        
        register32bit shamt(clk, reset, 1,1, shamtin,muxin1);
        register32bit SExt(clk, reset, 1,1, sextin, muxin2);
        
        register5bit rd(clk, reset, 1,1, rdin, rdout);
        register5bit rs(clk,reset,1,1,rsin,rsout);
        register5bit rt(clk,reset,1,1,rtin,rtout);

        register32bit luii(clk, reset, 1,1, luin, luiout);      
        D_ff_reg regwrite1(clk,reset,1,1,luiselectin,luiselectout_idex); 

endmodule

module pipelineIDEXC(input clk, input reset, input regWritec, input memWrite, input [4:0] rsin_idex, input [4:0] rtin_idex, input [31:0] rsOutc,
        input [31:0] rtOutc, input [31:0] swin, input [4:0] rdinc_idex, output regWriteoutc_idex,  output memWriteout_idex, 
		  output [4:0] rs_idex, output [4:0] rt_idex, output [31:0] rsOutc_idex, 
		  output [31:0] rtOutc_idex, output [31:0] swout_idex, output [4:0] rdoutc_idex);

        D_ff_reg d1(clk,reset,1,1,regWritec,regWriteoutc_idex);
        
        D_ff_reg d3(clk,reset,1,1,memWrite,memWriteout_idex);
     

        
        register32bit rsoutc(clk,reset,1,1,rsOutc, rsOutc_idex);
        register32bit rtoutc(clk,reset,1,1,rtOutc, rtOutc_idex);
        register32bit swinc(clk,reset,1,1,swin,swout_idex);

        register5bit rdin(clk,reset,1,1,rdinc_idex,rdoutc_idex);
        register5bit t1(clk,reset,1,1,rsin_idex,rs_idex);    
		 register5bit t2(clk,reset,1,1,rtin_idex,rt_idex);	

endmodule 

module pipelineEXMEM(input clk, input reset, input memReadin, input regWritein,
 input  memtoRegin, input lessflagin, input [31:0] aluOutin,
             input [4:0] rdin, output memReadout,  output regWriteout,
            output  memtoRegout, output lessflagout, output [31:0] aluOutout,  output [4:0] rdout);
            
    D_ff_reg memread(clk,reset,1,1,memReadin,memReadout);
    D_ff_reg regwrite(clk,reset,1,1,regWritein,regWriteout);
    D_ff_reg memtoreg(clk,reset,1,1,memtoRegin,memtoRegout);    
    
    
    register5bit rd(clk, reset, 1,1, rdin, rdout);
    
    D_ff_reg lessflag(clk,reset,1,1,lessflagin,lessflagout);
    register32bit aluout(clk, reset, 1,1, aluOutin, aluOutout);
            

endmodule

module pipelineEXMEMC(input clk, input reset, input regWriteoutc_idex,  input memWriteout_idex, 
 input [31:0] rtOutc_idex, input [31:0] adderoutc, input [4:0] rdfinalin_exmem, output regWriteoutc_exmem,  output memWriteout_exmem,
  output [31:0] rtOutc_exmem, output [31:0] adderoutc_exmem, output [4:0] rdfinalout_exmem);

    D_ff_reg reg1(clk,reset, 1,1,regWriteoutc_idex,regWriteoutc_exmem);
 
    D_ff_reg reg3(clk,reset, 1,1,memWriteout_idex,memWriteout_exmem);

   
    register32bit reg5(clk,reset,1,1,rtOutc_idex,rtOutc_exmem);
    register32bit reg6(clk,reset,1,1,adderoutc,adderoutc_exmem);

    register5bit reg7(clk,reset,1,1,rdfinalin_exmem,rdfinalout_exmem);


endmodule

module pipelineMEMWB(input clk, input reset, input regWritein, input  memtoRegin, input [4:0] rdin, input [15:0] memDatain, input [31:0] aluOutin,

                    output regWriteout, output  memtoRegout, output [4:0] rdout, output [15:0] memDataout, output [31:0] aluOutout);
                    
                    
    D_ff_reg regwrite(clk,reset,1,1,regWritein,regWriteout);
    D_ff_reg memtoreg(clk,reset,1,1,memtoRegin,memtoRegout);    
    
    register5bit rd(clk, reset, 1,1, rdin, rdout);  
    register32bit aluout(clk, reset, 1,1, aluOutin, aluOutout);
    
    register16bit memdata(clk, reset, 1,1, memDatain, memDataout);
                    


endmodule

module pipelineMEMWBC(input clk, input reset, input regWriteoutc_exmem, input [31:0] rtOutc_exmem,
    input [4:0] rdfinalout_exmem, output regWriteoutc_memwb, output [31:0] rtOutc_memwb,
    output [4:0] rdfinalout_memwb);

    D_ff_reg reg1(clk,reset,1,1,regWriteoutc_exmem,regWriteoutc_memwb);


    register32bit r2(clk,reset,1,1,rtOutc_exmem, rtOutc_memwb);

    register5bit r3(clk,reset,1,1,rdfinalout_exmem,rdfinalout_memwb);


    endmodule

// ************************ Pipelines end *************************************

// Control Circuit ******************************

module ctrlCkt(input [6:0] opcode, input [2:0] funct3_32,
    output reg [1:0] aluOp, output reg [1:0] aluSrc, output reg memRead, output reg branch, output reg regWrite, output reg  memtoReg);
    
    
        always @ (opcode, funct3_32)
            begin
                aluOp = 2'd0;
                aluSrc = 2'd0;
                memRead = 1'b0;
                branch = 1'b0;
                regWrite = 1'b0;
                memtoReg = 1'd0;
                
                case(opcode)
                    7'b0010011: if(funct3_32 == 3'b001)
                            begin
                                aluOp = 2'd0;
                                aluSrc = 2'd1;
                                memRead = 1'b0;
                                branch = 1'b0;
                                regWrite = 1'b1;
                                memtoReg = 1'd0; // aluOut
                            end
                           else if(funct3_32 == 3'b000)
                            begin
                                aluOp = 2'd1;
                                aluSrc = 2'd2;
                                memRead = 1'b0;
                                branch = 1'b0;
                                regWrite = 1'b1;
                                memtoReg = 1'd0; // aluOut
                            end
                    7'b0110011: begin
                            aluOp = 2'd2;
                            aluSrc = 2'd0;
                            memRead = 1'b0;
                            branch = 1'b0;
                            regWrite = 1'b1;
                            memtoReg = 1'd0; // sltu
                        end
                    7'b0000011: begin
                            aluOp = 2'd1;
                            aluSrc = 2'd2;
                            memRead = 1'b1;
                            branch = 1'b0;
                            regWrite = 1'b1;
                            memtoReg = 1'd1; // load half word
                        end
                    7'b0110111: begin   
                            aluOp = 2'd0;
                            aluSrc = 2'd1;
                            memRead = 1'b0;
                            branch = 1'b0;
                            regWrite = 1'b1;
                            memtoReg = 1'd0; // lui
                end
                    7'b1100011: begin
                            aluOp = 2'd2;
                            aluSrc = 2'd0;
                            memRead = 1'b0;
                            branch = 1'b1;
                            regWrite = 1'b0;
                            memtoReg = 1'd0; // branch
                        end
                endcase
            end

    
endmodule

module ctrlCktcomp(input [1:0] opcode, input [2:0] funct3_32, output reg memWrite,output reg regWritec, output reg jump, output reg memtoregc);
    always@(opcode,funct3_32)
        begin
            memWrite=1'b0;
            regWritec=1'b0;
            jump=1'b0;
            memtoregc=1'b0;
           

            case(opcode)
                2'd2: //mv
                    begin 
                        memWrite=1'b0;
                        regWritec=1'b1;
                        jump=1'b0;
                        memtoregc=1'b0;
                       
                    end
                2'd0: if(funct3_32==3'd6)
                    begin
                        memWrite = 1'b1;
                        regWritec=1'b0;
                        jump=1'b0;
                        memtoregc=1'b0;
                        
                    end
                2'd1: begin
                        memWrite = 1'b0;
                        regWritec = 1'b1;
                        jump = 1'b1;
                        memtoregc = 1'b1;
                        
                        end

            endcase

        end


endmodule


// ****************888 Control circuit ends ***********************


//****************** ALU ***************************


module ALU(input signed [31:0] aluIn1, input signed [31:0] aluIn2, input [1:0] aluOp, output reg [31:0] aluOut,
 output reg lessflag);
    always@(aluIn1 or aluIn2 or aluOp)
    begin
        if({1'd0,aluIn1} < {1'd0,aluIn2})
            lessflag = 1'b1;
        else
            lessflag = 1'b0;
        case(aluOp)
            3'd0:
                    aluOut = aluIn1 << aluIn2;
            3'd1:   aluOut = aluIn1 + aluIn2;
            3'd2: if({1'd0,aluIn1} < {1'd0,aluIn2})
                begin
                lessflag = 1'b1;
                aluOut = 32'd1;
                end
                  else
                  begin
                    aluOut = 32'd0;
                    lessflag = 1'b0;
                  end
        endcase
    end
endmodule

module comparatorlessthan(input signed [31:0]  in0, input signed [31:0] in1, output reg slt);
    always@(in0,in1)
        if({1'd0,in0}<{1'd0,in1})
            slt = 1'b1;
        else
            slt = 1'b0;
    
endmodule


// ****************** ALU Ends *******************

  

module fwdcct(input [4:0] rsout_idex, input [4:0] rtout_idex, input regWriteout_exmem, input [4:0] rdout_exmem, input [4:0] rdout_idex, input regWriteout_memwb, 
input [4:0] rdout_memwb, input [4:0] rsout_ifid, input [4:0] rtout_ifid, input regWriteoutc_idex, input regWriteoutc_exmem, input regWriteoutc_memwb,
input [4:0] rdoutc_idex, input [4:0] rdfinalout_exmem, input [4:0] rdfinalout_memwb,
 output reg [2:0] fwda, output reg [2:0] fwdb,output reg [2:0] latchsrcina, output reg [2:0] latchsrcinb, output reg [2:0] latchsrcina_c, output reg [2:0] latchsrcinb_c );

    always@(rsout_idex, rtout_idex, regWriteout_exmem, rdout_exmem, regWriteout_memwb, rdout_memwb,
	 rsout_ifid, rtout_ifid)
        begin
            fwda = 3'd3;
            fwdb = 3'd3;
            latchsrcina = 3'd7;
			latchsrcinb = 3'd7;
			latchsrcina_c= 3'd0;
			latchsrcinb_c = 3'd0;
			
            if((regWriteout_exmem==1 && rdout_exmem == rsout_idex)|| (regWriteoutc_exmem==1'b1 && rdfinalout_exmem == rsout_idex) )
				begin
					if(regWriteout_exmem==1 && rdout_exmem == rsout_idex)	
						fwda = 3'd1;
					else
						fwda = 3'd4;
				end
            else if((regWriteout_memwb==1 && rdout_memwb == rsout_idex) || (regWriteoutc_memwb==1'b1 && rdfinalout_memwb == rsout_idex))
				begin
					if(regWriteout_memwb==1 && rdout_memwb == rsout_idex)
						fwda = 3'd2;
					else
						fwda = 3'd5;
				end
			
			if((regWriteout_memwb==1 && rdout_memwb == rsout_ifid && rdout_exmem!=rsout_ifid && rdout_idex!=rsout_ifid)
						&& (regWriteoutc_memwb==1'b1 && rdfinalout_memwb == rsout_ifid && rdfinalout_exmem!=rsout_ifid && rdoutc_idex!=rsout_ifid))
				begin	
					if(regWriteout_memwb==1 && rdout_memwb == rsout_ifid && rdout_exmem!=rsout_ifid && rdout_idex!=rsout_ifid)
						latchsrcina = 3'd0;
					else
						latchsrcina_c = 3'd7;
				end
			
			
			
			 if((regWriteout_exmem==1 && rdout_exmem == rtout_idex)|| (regWriteoutc_exmem==1'b1 && rdfinalout_exmem == rtout_idex) )
				begin
					if(regWriteout_exmem==1 && rdout_exmem == rtout_idex)	
						fwdb = 3'd1;
					else
						fwdb = 3'd4;
				end
            else if((regWriteout_memwb==1 && rdout_memwb == rtout_idex) || (regWriteoutc_memwb==1'b1 && rdfinalout_memwb == rtout_idex))
				begin
					if(regWriteout_memwb==1 && rdout_memwb == rtout_idex)
						fwdb = 3'd2;
					else
						fwdb = 3'd5;
				end
			if((regWriteout_memwb==1 && rdout_memwb == rtout_ifid && rdout_exmem!=rtout_ifid && rdout_idex!=rtout_ifid)
						&& (regWriteoutc_memwb==1'b1 && rdfinalout_memwb == rtout_ifid && rdfinalout_exmem!=rtout_ifid && rdoutc_idex!=rtout_ifid))
				begin	
					if(regWriteout_memwb==1 && rdout_memwb == rtout_ifid && rdout_exmem!=rtout_ifid && rdout_idex!=rtout_ifid)
						latchsrcinb = 3'd0;
					else
						latchsrcinb_c = 3'd7;
				end
			                                          
        end

endmodule
//loadusehazard
module loadusehazard(input memReadout_idex, input [4:0] rdout_idex, input [4:0] irOut_rs, input [4:0] irOut_rt,input [4:0] irOutc_rs,
			irOutc_rt, output reg pcWrite,
	output reg ifidWrite, output reg idex_controlmux);
	
	always@(memReadout_idex, rdout_idex, irOut_rs, irOut_rt, irOutc_rs, irOutc_rt)
		begin

			if(memReadout_idex==1'b1 && (rdout_idex==irOut_rs || rdout_idex == irOut_rt || rdout_idex == irOutc_rs || rdout_idex == irOutc_rt))
				begin
					pcWrite = 1'b0;
					ifidWrite = 1'b0;
					idex_controlmux = 1'b1;
				end
			else
				begin
					pcWrite = 1'b1;
					ifidWrite = 1'b1;
					idex_controlmux = 1'b0;
				end
		end
	
endmodule

module fwdcctc(input [4:0] idex_rs, input [4:0] idex_rt, input [4:0] ifid_rs, input [4:0] ifid_rt, input [4:0] rdoutc_idex, input [4:0] rdfinalout_exmem, input [4:0] rdfinalout_memwb, input regWriteoutc_idex,
				input regWriteoutc_exmem, input regWriteoutc_memwb, input [4:0] rdout_idex, input [4:0] rdout_exmem,  input [4:0] rdout_memwb, input regWriteout_idex,
				input regWriteout_exmem, input regWriteout_memwb, output reg[2:0] fwdrs, output reg[2:0] fwdrt, 
				output reg[2:0] latchsrcinrs, output reg[2:0] latchsrcinrt,
				output reg[2:0] latchsrcinrsc, output reg[2:0] latchsrcinrtc );
			
			
			always@(idex_rs, idex_rt, ifid_rs, ifid_rt, rdoutc_idex, rdfinalout_exmem,rdfinalout_memwb, regWriteout_idex, regWriteoutc_exmem, regWriteoutc_memwb, rdout_idex,
						rdout_exmem, rdout_memwb, regWriteout_idex, regWriteout_exmem, regWriteout_memwb)
						
				begin
				
					fwdrs = 3'd3;
					fwdrt = 3'd3;
					latchsrcinrs=3'd0;
					latchsrcinrt=3'd0;
					latchsrcinrsc=3'd7;
					latchsrcinrtc=3'd7;
					
					if ((regWriteoutc_exmem ==1'b1 && (idex_rs == rdfinalout_exmem)) 
								|| (regWriteout_exmem==1'b1 && (idex_rs == rdout_exmem)))
						begin
					
									if(idex_rs == rdfinalout_exmem)
									begin
										
										fwdrs = 3'd1;
									end
									else
										fwdrs = 3'd4;

						end
					
						
					else if((regWriteoutc_memwb==1'b1 &&(idex_rs == rdfinalout_memwb ))
							||	(regWriteout_memwb==1'b1 && (idex_rs == rdout_memwb)) )
						begin	
							

							
									if(idex_rs == rdfinalout_memwb)
										fwdrs = 3'd2;
									else
										fwdrs = 3'd5;
						end
						
					if( ((regWriteoutc_memwb == 1'b1 && (ifid_rs == rdfinalout_memwb ))
							|| (regWriteout_memwb == 1'b1 && (ifid_rs == rdout_memwb ))) && 
							(regWriteoutc_exmem == 1'b1 && (ifid_rs != rdfinalout_exmem )) &&
							(regWriteoutc_idex == 1'b1 && (ifid_rs != rdoutc_idex)) &&
							(regWriteout_exmem == 1'b1 && (ifid_rs != rdout_exmem )) &&
							(regWriteout_idex == 1'b1 && (ifid_rs != rdout_idex )) )
							
						begin
							if(ifid_rs == rdfinalout_memwb)
										latchsrcinrsc=3'd0;
									else
										latchsrcinrs = 3'd7;
							end
					
					//for rt :
						
					if ((regWriteoutc_exmem ==1'b1 && (idex_rt == rdfinalout_exmem)) 
								|| (regWriteout_exmem==1'b1 && (idex_rt == rdout_exmem)))
						begin
					
									if(idex_rt == rdfinalout_exmem)
									begin
										
										fwdrt = 3'd1;
									end
									else
										fwdrt = 3'd4;

						end
					
						
					else if((regWriteoutc_memwb==1'b1 &&(idex_rt == rdfinalout_memwb ))
							||	(regWriteout_memwb==1'b1 && (idex_rt == rdout_memwb)) )
						begin	
							

							
									if(idex_rt == rdfinalout_memwb)
										fwdrt = 3'd2;
									else
										fwdrt = 3'd5;
						end
						
					if( ((regWriteoutc_memwb == 1'b1 && (ifid_rt == rdfinalout_memwb ))
							|| (regWriteout_memwb == 1'b1 && (ifid_rt == rdout_memwb ))) && 
							(regWriteoutc_exmem == 1'b1 && (ifid_rt != rdfinalout_exmem )) &&
							(regWriteoutc_idex == 1'b1 && (ifid_rt != rdoutc_idex)) &&
							(regWriteout_exmem == 1'b1 && (ifid_rt != rdout_exmem )) &&
							(regWriteout_idex == 1'b1 && (ifid_rt != rdout_idex )) )
							
						begin
							if(ifid_rs == rdfinalout_memwb)
										latchsrcinrtc=3'd0;
									else
										latchsrcinrt = 3'd7;
							end
							
					
				end
endmodule


module fwdcctcompare(input [4:0] rsout_ifid, input [4:0] rtout_ifid, input [4:0] rdout_idex, input [4:0] rdout_exmem, input [4:0] rdout_memwb,
						input [4:0] rdoutc_idex, input [4:0] rdfinalout_exmem, input [4:0] rdfinalout_memwb, input regWriteoutc_idex, input memReadout_exmem,
						input regWriteoutc_exmem, input regWriteoutc_memwb,	input regWriteout_idex, input regWriteout_exmem, input regWriteout_memwb, 
						output reg[2:0] comparatorfwda,	output reg[2:0] comparatorfwdb, output reg stall);
		
			always@(rsout_ifid,rtout_ifid, rdout_idex, rdout_exmem, rdout_memwb,regWriteout_idex, regWriteout_exmem, regWriteout_memwb)
				begin
					comparatorfwda = 3'd0;
					comparatorfwdb = 3'd0;
					stall = 1'b0;
					if((regWriteout_idex==1'b1 && (rsout_ifid==rdout_idex))||(regWriteoutc_idex==1'b1&&(rsout_ifid==rdoutc_idex)))
						begin
							if(regWriteout_idex==1'b1 && (rsout_ifid==rdout_idex))
								stall = 1'b1;
							else 
								comparatorfwda = 3'd3;
						end
					else if((regWriteout_exmem==1'b1 && rsout_ifid==rdout_exmem)||(regWriteoutc_exmem==1'b1&&(rsout_ifid==rdfinalout_exmem))||
							(regWriteout_exmem==1'b1 && rsout_ifid==rdout_exmem && memReadout_exmem==1'b1))
						begin
						if(regWriteout_exmem==1'b1 && rsout_ifid==rdout_exmem&& memReadout_exmem==1'b1)
							stall = 1'b1;
						else if(regWriteout_exmem==1'b1 && rsout_ifid==rdout_exmem)
							comparatorfwda = 3'd1;
						else 
							comparatorfwda = 3'd4;
						end
					else if((regWriteout_memwb==1'b1 && rsout_ifid == rdout_memwb)||(regWriteoutc_memwb==1'b1&&(rsout_ifid==rdfinalout_memwb)))
						begin
						if(regWriteout_memwb==1'b1 && rsout_ifid == rdout_memwb)
							comparatorfwda = 3'd2;
						else
							comparatorfwda = 3'd5;
						end
						
						
					if((regWriteout_idex==1'b1 && (rtout_ifid==rdout_idex))||(regWriteoutc_idex==1'b1&&(rtout_ifid==rdoutc_idex)))
						begin
							if(regWriteout_idex==1'b1 && (rtout_ifid==rdout_idex))
								stall = 1'b1;
							else 
								comparatorfwdb = 3'd3;
						end
					else if((regWriteout_exmem==1'b1 && rtout_ifid==rdout_exmem)||(regWriteoutc_exmem==1'b1&&(rtout_ifid==rdfinalout_exmem))||
							(regWriteout_exmem==1'b1 && rtout_ifid==rdout_exmem && memReadout_exmem==1'b1))
						begin
						if(regWriteout_exmem==1'b1 && rtout_ifid==rdout_exmem&& memReadout_exmem==1'b1)
							stall = 1'b1;
						else if(regWriteout_exmem==1'b1 && rtout_ifid==rdout_exmem)
							comparatorfwdb = 3'd1;
						else 
							comparatorfwdb = 3'd4;
						end
						
					else if((regWriteout_memwb==1'b1 && rtout_ifid == rdout_memwb)||(regWriteoutc_memwb==1'b1&&(rtout_ifid==rdfinalout_memwb)))
						begin
						if(regWriteout_memwb==1'b1 && rtout_ifid == rdout_memwb)
							comparatorfwdb = 3'd2;
						else
							comparatorfwdb = 3'd5;
						end
						
				end
						
endmodule

module project1(input clk, input reset, output [31:0] Result, output [31:0] Resultc);

        wire  memtoRegout_exmem;
        wire memReadout_exmem,   regWriteout_exmem;
        wire [31:0]   aluOutout_exmem;
        wire [4:0] rdout_exmem;
        wire lessflagout_exmem;
        wire stall ,branch;

        wire [31:0] pcout, muxpcin, adderpcout, branch_address;
        wire pcselect;
        wire pcWrite, ifidWrite,idex_controlmux; // loadusehazard

        // muxes for selecting branchaddress or jump address
		  wire branchstall;
		  assign branchstall = ~(branch&stall);
        wire [31:0] branchmuxout, jump_address;
        wire jump;

        mux2to1_32bits muxbranch(adderpcout,branch_address,pcselect,branchmuxout);
        mux2to1_32bits muxjump(branchmuxout,jump_address,jump,muxpcin);

        // mux end for pc


        pc_adder adder1(32'd4, pcout,adderpcout);
        register32bit PC(clk, reset, pcWrite&branchstall,1,muxpcin, pcout);

        // instructionmemory

        wire [31:0] irIn,irOut;
        wire [15:0] irInc, irOutc;

        IM instructionmemory(clk, reset, pcout[4:0],  irIn,irInc);
		
        // instructionmemoryend 


        // pipelines IFID

		  wire [31:0] tempout;
        pipelineIFID pipeline1( clk, reset,ifidWrite&branchstall, irIn,  tempout);
        
		  wire ifidWritec;
        wire [31:0] pcoutc_ifid,tempoutcomp;

        pipelineIFIDC pipeline1c(clk,reset, ifidWrite&branchstall,irInc, adderpcout,tempoutcomp,pcoutc_ifid);
		 
		  //muxes for flushing pipelines, jugaad
		  
		  wire ifflush;
		  D_ff_reg muxsignalifidnormal(clk,reset,1,1,((irOut[6]&slt)|irOutc[0])&(branchstall),ifflush); //both pipelines need to be flushed in case of branch OR jump
        mux2to1_32bits mux20(tempout,32'd0,ifflush,irOut);         
		  mux2to1_32bits mux222(tempoutcomp,32'd0,ifflush,irOutc); 
        // jugaad ends
			
		  //ifid pipelines end
		  
        // Register file starts
        
        wire [31:0] rsOut, rtOut, rsOutc, rtOutc;

        wire regWriteout_memwb, regWriteoutc_memwb;
        wire [4:0] rdout_memwb, rdfinalout_memwb;
        
        wire [4:0] rtc;
        mux2to1_5bits muxrsrt({2'd0,irOutc[4:2]},irOutc[6:2],irOutc[1],rtc);

        registerFile regfile(clk,  reset, regWriteout_memwb,regWriteoutc_memwb, irOut[19:15],  irOut[24:20],  rdout_memwb, {2'd0,irOutc[9:7]}, rtc,
            rdfinalout_memwb, Result, Resultc,  rsOut,  rtOut, rsOutc, rtOutc);
        

        // Registerfile ends

        wire [31:0] shamtin,sextin;
        
        zeroExt5to32 zeroext1(irOut[24:20], shamtin);
        signExt12to32 sext1(irOut[31:20],sextin);
        signExt12to32 sext2({irOut[31],irOut[7],irOut[30:25],irOut[11:8]},branch_address);
        

        // Control circuit for normal

        wire [1:0] aluOpin, aluSrcin, memtoRegin;
        wire memReadin, regWritein;
        
		 wire [1:0] aluOpin_temp, aluSrcin_temp, memtoRegin_temp;
		wire memReadin_temp, regWritein_temp;
		
        

        ctrlCkt cc1(irOut[6:0],  irOut[14:12], aluOpin_temp,  aluSrcin_temp,  memReadin_temp,  branch,  regWritein_temp, 
        memtoRegin_temp);
        
        // Control circuit for normal ends

        wire slt;

        // pro comparator for branch
		wire [2:0] comparatorfwda, comparatorfwdb;
		 wire [31:0] rsOut_fwd, rtOut_fwd;
		 

		 
		
		fwdcctcompare profwdcct(irOut[19:15], irOut[24:20],  rdout_idex,  rdout_exmem,rdout_memwb, rdoutc_idex,rdfinalout_exmem,
			rdfinalout_memwb, regWriteoutc_idex, memReadout_exmem, regWriteoutc_exmem, regWriteoutc_memwb, regWriteout_idex,regWriteout_exmem,
		 regWriteout_memwb,  comparatorfwda, comparatorfwdb, stall);
		
		
		mux8to1_32bits m1branch(rsOut,aluOutout_exmem, Result, rtOutc_idex, rtOutc_exmem, Resultc, 32'd0,32'd0,comparatorfwda, rsOut_fwd);
		mux8to1_32bits m2branch(rtOut,aluOutout_exmem, Result, rtOutc_idex, rtOutc_exmem, Resultc, 32'd0,32'd0, comparatorfwdb, rtOut_fwd);

		
		
        comparatorlessthan compare(rsOut_fwd,rtOut_fwd,slt);
		
        and pcsel(pcselect,slt,branch);

        wire [31:0] aluIn1, alusrc_muxin0, alusrc_muxin1, alusrc_muxin2, luiout_idex, inputfwda3,inputfwdb3;
        wire [4:0] rdout_idex,rsout_idex,rtout_idex;
        wire [1:0] aluOp, aluSrc;
		  wire memtoRegout_idex;
        wire memReadout_idex,   regWriteout_idex;
        
		loadusehazard l1(memReadout_idex, rdout_idex,irOut[19:15],irOut[24:20],{2'd0,irOutc[9:7]}, rtc,pcWrite,ifidWrite,idex_controlmux);

        
        wire [2:0] fwda,fwdb;
     
        wire [2:0] latchsrcina,latchsrcinb,latchsrcouta,latchsrcoutb ;
		   wire [2:0] latchsrcina_c,latchsrcinb_c,latchsrcouta_c,latchsrcoutb_c ;
		
		
	 fwdcct fwdcct1(rsout_idex,  rtout_idex, regWriteout_exmem,  rdout_exmem,  rdout_idex,  regWriteout_memwb, 
 rdout_memwb, irOut[19:15], irOut[24:20],  regWriteoutc_idex,  regWriteoutc_exmem,  regWriteoutc_memwb,
 rdoutc_idex, rdfinalout_exmem,  rdfinalout_memwb,  fwda,  fwdb, latchsrcina,  latchsrcinb, latchsrcina_c,  latchsrcinb_c );


		
        
        
		
        
        wire luiselectout_idex;
        // loaduse hazard muxes

		mux2to1_2bits idexaluop(aluOpin_temp,2'd0,idex_controlmux,aluOpin);
		mux2to1_2bits idexalusrc(aluSrcin_temp,2'd0,idex_controlmux,aluSrcin);
		mux2to1_2bits idexmemtoreg(memtoRegin_temp,2'd0,idex_controlmux,memtoRegin);

		mux2to1_1bit idexmemread(memReadin_temp,1'b0,idex_controlmux,memReadin);
		mux2to1_1bit idexregwrite(regWritein_temp,1'b0,idex_controlmux,regWritein);

		
		
        // ID cycle compressed


		
		
		wire memWrite, regWritec, memtoregc, memWrite_temp, regWritec_temp;
        ctrlCktcomp cctcomp(irOutc[1:0],irOutc[15:13],memWrite_temp, regWritec_temp, jump, memtoregc);
		
		pc_adder jumpadder(pcoutc_ifid,{{21{irOutc[12]}},irOutc[12:2]},jump_address);
		
		wire [4:0] rdoutinc_idex;
		mux2to1_5bits rdfin(irOutc[11:7],5'd1,irOutc[0],rdoutinc_idex);
		
		wire [31:0] resultdatac;
		mux2to1_32bits m101(rtOutc, pcoutc_ifid, irOutc[0], resultdatac);

        // ID Cycle compressed ends
		
		//loaduse hazard muxes end regWritec, memWrite,
		mux2to1_2bits idexcomp(regWritec_temp,2'd0,idex_controlmux,regWritec);
		mux2to1_2bits idexcomp2(memWrite_temp,2'd0,idex_controlmux,memWrite);
		
		
		// IDEX Pipelines start

        pipelineIDEX pipeline2(clk,  reset, rsOut, rtOut,  aluOpin, aluSrcin,  memReadin,  regWritein,   memtoRegin,  shamtin,
         irOut[11:7],  sextin, {irOut[31:12],12'd0},irOut[19:15],irOut[24:20],irOut[2], inputfwda3, inputfwdb3,
         aluOp,  aluSrc,  memReadout_idex, regWriteout_idex,  memtoRegout_idex, alusrc_muxin1,
         alusrc_muxin2,
        luiout_idex,   rdout_idex,rsout_idex,rtout_idex,luiselectout_idex);
		
	
		
		wire regWriteoutc_idex,  memWriteout_idex;
		wire [31:0]  rsOutc_idex, rtOutc_idex, swout_idex;
		wire [4:0] rdoutc_idex,rs_idex,rt_idex;
		 
		pipelineIDEXC pipeline2c(clk,reset, regWritec, memWrite, {2'd0,irOutc[9:7]}, rtc, rsOutc, resultdatac, {25'd0,irOutc[5],irOutc[12:10],irOutc[6],2'd0},
		rdoutinc_idex, regWriteoutc_idex,  memWriteout_idex, rs_idex,rt_idex, rsOutc_idex, rtOutc_idex, swout_idex, rdoutc_idex);
		
	
		// IDEX Pipelines end
            
        // ALU Normal
		
		
        wire [31:0] aluIn2,latchResult,latchResultc; 
        mux8to1_32bits mux10(latchResult,aluOutout_exmem, Result,inputfwda3, rtOutc_exmem,Resultc,32'd0,latchResultc,((fwda&latchsrcouta)|latchsrcouta_c),aluIn1);
        mux8to1_32bits mux11(latchResult,aluOutout_exmem, Result,inputfwdb3, rtOutc_exmem,Resultc,32'd0,latchResultc,((fwdb&latchsrcoutb)|latchsrcoutb_c),alusrc_muxin0);
        
        
        mux4to1_32bits mux4(alusrc_muxin0,alusrc_muxin1, alusrc_muxin2, 32'd0,aluSrc, aluIn2);
        
        wire [31:0] aluOutin, aluOut_alu;
        wire lessflagin;
        
        
        ALU aluu(aluIn1, aluIn2, aluOp, aluOut_alu,lessflagin);
        
        
        mux2to1_32bits muxchoosealui(aluOut_alu,luiout_idex,luiselectout_idex,aluOutin);
        
        // ALU Normal ends
		  
		wire [2:0] fwdrs,  fwdrt, latchsrcinrs,latchsrcinrt, latchsrcinrs_c,latchsrcoutrs,latchsrcoutrt, latchsrcinrt_c, latchsrcoutrs_c, latchsrcoutrt_c;
			
		fwdcctc fowardcircuitcompressed(rs_idex,rt_idex,{2'd0,irOutc[9:7]} , rtc,  rdoutc_idex,  rdfinalout_exmem,  rdfinalout_memwb,  
				regWriteoutc_idex, regWriteoutc_exmem, regWriteoutc_memwb, rdout_idex,  rdout_exmem,  
				rdout_memwb,  regWriteout_idex, regWriteout_exmem, regWriteout_memwb, 
				fwdrs,  fwdrt, latchsrcinrs,latchsrcinrt, latchsrcinrs_c, latchsrcinrt_c);
				
		// Execute cycle compressed
		wire [31:0] adderoutc,addIn0,rtOut_fwdc;
			
		mux8to1_32bits mux40(latchResultc, rtOutc_exmem,Resultc, rsOutc_idex, aluOutout_exmem, Result,32'd0,latchResult,((fwdrs&latchsrcoutrs_c)|latchsrcoutrs),addIn0);
        
		mux8to1_32bits mux41(latchResultc, rtOutc_exmem,Resultc, rtOutc_idex, aluOutout_exmem, Result,32'd0,latchResult,((fwdrt&latchsrcoutrt_c)|latchsrcoutrt),rtOut_fwdc);
   
			
			pc_adder storeadd(addIn0,swout_idex,adderoutc);
		
			
		
		// Execute cycle compressed ends
		
		// Exmem pipelines 
        
        pipelineEXMEM pipeline3( clk,  reset,  memReadout_idex,    regWriteout_idex,
        memtoRegout_idex, lessflagin,  aluOutin,
          rdout_idex,  memReadout_exmem,   regWriteout_exmem, 
        memtoRegout_exmem,  lessflagout_exmem,  aluOutout_exmem,
           rdout_exmem);
		

		wire regWriteoutc_exmem,  memWriteout_exmem;
		wire [31:0] rtOutc_exmem, adderoutc_exmem;
		wire [4:0] rdfinalout_exmem;
 
		pipelineEXMEMC pipeline3c(clk,reset, regWriteoutc_idex,  memWriteout_idex,  rtOut_fwdc, adderoutc, rdoutc_idex,
			regWriteoutc_exmem, memWriteout_exmem,  rtOutc_exmem, adderoutc_exmem, rdfinalout_exmem);
		   
            
		// exmem pipelines end

		// Data memorystart
		
		wire [31:0]  memtoRegin0,  memtoRegin1, memtoRegin2;
        wire [15:0] signextinput,memDataout;
        
        wire  memtoRegout_memwb;
         		 
        DM datamemory(clk, reset, memReadout_exmem,memWriteout_exmem,aluOutout_exmem[4:0],adderoutc_exmem[4:0], rtOutc_exmem, memDataout);
        
        // Data memory end
		
		// MemWB pipelines start
		
		pipelineMEMWB pipeline4(clk,  reset, regWriteout_exmem,  memtoRegout_exmem,  rdout_exmem, memDataout,  aluOutout_exmem, 
         regWriteout_memwb,  memtoRegout_memwb,  rdout_memwb, signextinput,  memtoRegin0);
        
       
	
	
		
		pipelineMEMWBC pipeline4c(clk,reset,regWriteoutc_exmem,  rtOutc_exmem, rdfinalout_exmem, regWriteoutc_memwb,
			Resultc, rdfinalout_memwb);
		
		// MemWB Pipielines end
		
		// Muxes for Normal result
        signExt16to32 sext4(signextinput,memtoRegin1);
        
        
        mux2to1_32bits mux5(memtoRegin0,  memtoRegin1,memtoRegout_memwb,Result);
		 
		 
		register32bit latchdata(clk,reset,1'b1,1'b1,Result,latchResult);
		register32bit latchdatac(clk,reset,1'b1,1'b1,Resultc,latchResultc);
		
		register3bit latchsignal1(clk,reset,1'b1,1'b1,latchsrcina,latchsrcouta);
		register3bit latchsignal2(clk,reset,1'b1,1'b1,latchsrcinb,latchsrcoutb);
		
		register3bit latchsignal1c(clk,reset,1'b1,1'b1,latchsrcina_c,latchsrcouta_c);
		register3bit latchsignal2c(clk,reset,1'b1,1'b1,latchsrcinb_c,latchsrcoutb_c);
		
		
		register3bit latchsignal111(clk,reset,1'b1,1'b1,latchsrcinrs,latchsrcoutrs);
		register3bit latchsignal2222(clk,reset,1'b1,1'b1,latchsrcinrt,latchsrcoutrt);
		
		register3bit latchsignal1111c(clk,reset,1'b1,1'b1,latchsrcinrs_c,latchsrcoutrs_c);
		register3bit latchsignal2222c(clk,reset,1'b1,1'b1,latchsrcinrt_c,latchsrcoutrt_c);
		
		
		
endmodule
        
module testbench;
    reg clk;
    reg reset;
    wire [31:0] Result, Resultc;
    project1 SC(clk, reset, Result, Resultc);

    always
    #5 clk=~clk;
    
    initial
    begin
        clk=0; reset=1;
        #5  reset=0;    
        #200 $finish;
    end
endmodule