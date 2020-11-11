`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 12/15/2019 08:48:15 AM
// Design Name: 
// Module Name: main
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define stop 3'd0
`define forward 3'd1
`define left_forward 3'd2
`define right_forward 3'd3
`define backward 3'd4
`define left_backward 3'd5
`define right_backward 3'd6
`define hhg 32'd2959
`define SONG1 16
`define SONG2 500
`define sil 32'd50000000

module main(
    input clk, // clock from crystal
    input rst, // active high reset: BTNC
    input SW0, // forward
    input SW1, // backward
    input SW2, // turn left
    input SW3, // turn right
    input RxD, // bluetooth input signal
	input Echo,
    output wire IN1,
    output wire IN2,
    output wire IN3,
    output wire IN4,
	output audio_mclk, 
    output audio_lrck, 
    output audio_sck,
    output audio_sdin,
	output Trig,
    output wire [3:0] _led, //  _led[3:0] = {turn right , turn left , backward , forward}
    output wire [3:0] DIGIT,
    output wire [6:0] DISPLAY
    );
   
    wire clkDiv13, clkDiv16, clkDiv18, clkDiv19, clkDiv22;
	wire [15:0] audio_in_left, audio_in_right;
    wire [7:0] RxData;
    wire [3:0] BCD0, BCD1, BCD2, BCD3;
	
	wire [11:0] ibeatNum; // Beat counter
    wire [31:0] freqL, freqR; // Raw frequency, produced by music module
    wire [21:0] freq_outL, freq_outR; // Processed Frequency, adapted to the clock rate of Basys3
	wire [32:0] dis;
    
	reg clkSelect, play_beep, music_beep, ultrasonic_mode;
    reg [2:0] direction , next_direction;
	reg [2:0] volume = 3'd0;

    assign _led = (direction==`stop) ? 4'b0000 :
                  (direction==`forward) ? 4'b0001 :
                  (direction==`left_forward) ? 4'b0101 :
                  (direction==`right_forward) ? 4'b1001 :
                  (direction==`backward) ? 4'b0010 :
                  (direction==`left_backward) ? 4'b0110 :
                  (direction==`right_backward) ? 4'b1010 : 4'b0000;
    
	assign freq_outL = 50000000 / freqL;
    assign freq_outR = 50000000 / freqR;
	
    assign BCD0 = RxData%10;
    assign BCD1 = (RxData/10)%10;
    assign BCD2 = (RxData/100)%10;
    assign BCD3 = (RxData/1000)%10;
    
    clock_divider #(.n(13)) clock_13(
        .clk(clk),
        .clk_div(clkDiv13)
    );
    
    clock_divider #(.n(16)) clock_16(
        .clk(clk),
        .clk_div(clkDiv16)
    );
    
    clock_divider #(.n(18)) clock_18(
        .clk(clk),
        .clk_div(clkDiv18)
    );
    
    clock_divider #(.n(19)) clock_19(
        .clk(clk),
        .clk_div(clkDiv19)
    );
    
    clock_divider #(.n(22)) clock_22(
        .clk(clk),
        .clk_div(clkDiv22)
    );
    
    bluetooth_RX(
        .clk(clk), //input clock
        .reset(rst), //input reset 
        .RxD(RxD), //input receving data line
        .RxData(RxData)
	);
    
    motor_control(
        .direction(direction),
        .IN1(IN1),
        .IN2(IN2),
        .IN3(IN3),
        .IN4(IN4)
    );
    
    seven_segment(
        .rst(rst),
        .clk(clkDiv13),
        .BCD3(BCD3),
        .BCD2(BCD2),
        .BCD1(BCD1),
        .BCD0(BCD0),
        .DIGIT(DIGIT),
        .DISPLAY(DISPLAY)
    ); // show the value of RxData
    
	_player_control playerCtrl_00 ( 
        .clk(clkSelect),
        .reset(rst),
        ._play(play_beep),
        ._repeat(play_beep),
        ._music(music_beep),
        .ibeat(ibeatNum)
    );

    // Music module
    // [in]  beat number and en
    // [out] left & right raw frequency
    music_example music_00 (
        .ibeatNum(ibeatNum),
        .en(!play_beep),
        .toneL(freqL),
        .toneR(freqR)
    );

    // Note generation
    // [in]  processed frequency
    // [out] audio wave signal (using square wave here)
    note_gen noteGen_00(
        .clk(clk), // clock from crystal
        .rst(rst), // active high reset
        .note_div_left(freq_outL),
        .note_div_right(freq_outR),
        .audio_left(audio_in_left), // left sound audio
        .audio_right(audio_in_right),
        .volume(volume) // 3 bits for 5 levels
    );

    // Speaker controller
    speaker_control sc(
        .clk(clk),  // clock from the crystal
        .rst(rst),  // active high reset
        .audio_in_left(audio_in_left), // left channel audio data input
        .audio_in_right(audio_in_right), // right channel audio data input
        .audio_mclk(audio_mclk), // master clock
        .audio_lrck(audio_lrck), // left-right clock
        .audio_sck(audio_sck), // serial clock
        .audio_sdin(audio_sdin) // serial audio data input
    );
	
	sonic(.clock(clk),.trig(Trig),.echo(Echo),.distance(dis));
	
    always@(posedge clk) begin
        if(rst)begin
            direction = `stop;
            
        end else begin
            direction = next_direction;
        end
    end
	
	always@(posedge clk)begin
        if(!ultrasonic_mode || rst) begin
            play_beep = 0;
            music_beep = 0;
            clkSelect = clkDiv22;
            volume = 3'd0;
        end
        else if(ultrasonic_mode) begin
            if(dis <= 3 && dis >= 2) begin
                play_beep = 1;
                music_beep = 1;
                clkSelect = clkDiv19;
                volume = 3'd4;
            end
            else if(dis >= 4 && dis <= 6) begin
                play_beep = 1;
                music_beep = 0;
                clkSelect = clkDiv22;
                volume = 3'd4;
            end
            else if(dis >= 7 && dis <= 9)begin
                play_beep = 0;
                music_beep = music_beep;
                clkSelect = clkDiv22;
                volume = 3'd0;
            end
       end
    end
    
    always@(posedge clk) begin
        if(RxData == 8'd82) begin
            ultrasonic_mode = 1'd1;
        end
        else if(RxData == 8'd76) begin
            ultrasonic_mode = 1'd0;
        end
        else begin
            ultrasonic_mode = ultrasonic_mode;
        end    
    end
    
    always@*begin
        case(RxData)
            8'd71: next_direction=`left_forward;
            8'd70: next_direction=`forward;
            8'd73: next_direction=`right_forward;
            8'd72: next_direction=`left_backward;
            8'd66: next_direction=`backward;
            8'd74: next_direction=`right_backward;
            default:begin
                if(SW0==1 && SW1==0)begin
                    if(SW2==1 && SW3==0) next_direction=`left_forward;
                    else if(SW2==0 && SW3==1) next_direction=`right_forward;
                    else next_direction=`forward;
                end else if(SW0==0 && SW1==1)begin
                    if(SW2==1 && SW3==0) next_direction=`left_backward;
                    else if(SW2==0 && SW3==1) next_direction=`right_backward;
                    else next_direction=`backward;
                end else begin
                    next_direction=`stop;
                end
            end
        endcase
    end
endmodule

module motor_control(
    input [2:0] direction,
    output reg IN1,
    output reg IN2,
    output reg IN3,
    output reg IN4
    );
    always@*begin
        case(direction)
            `stop: {IN1,IN2,IN3,IN4} = 4'b0000;
            `forward: {IN1,IN2,IN3,IN4} = 4'b1010;
            `left_forward: {IN1,IN2,IN3,IN4} = 4'b1000;
            `right_forward: {IN1,IN2,IN3,IN4} = 4'b0010;
            `backward: {IN1,IN2,IN3,IN4} = 4'b0101;
            `left_backward: {IN1,IN2,IN3,IN4} = 4'b0100;
            `right_backward: {IN1,IN2,IN3,IN4} = 4'b0001;
            default: {IN1,IN2,IN3,IN4} = 4'b0000;
        endcase
    end
endmodule

module bluetooth_RX(
	input clk, //input clock
	input reset, //input reset 
	input RxD, //input receving data line
	output [7:0] RxData // output for 8 bits data
	// output [7:0]LED // output 8 LEDs
    );
	//internal variables
	reg shift; // shift signal to trigger shifting data
	reg state, nextstate; // initial state and next state variable
	reg [3:0] bitcounter; // 4 bits counter to count up to 9 for UART receiving
	reg [1:0] samplecounter; // 2 bits sample counter to count up to 4 for oversampling
	reg [13:0] counter; // 14 bits counter to count the baud rate
	reg [9:0] rxshiftreg; //bit shifting register
	reg clear_bitcounter,inc_bitcounter,inc_samplecounter,clear_samplecounter; //clear or increment the counter

	// constants
	parameter clk_freq = 100_000_000;  // system clock frequency
	parameter baud_rate = 9_600; //baud rate
	parameter div_sample = 4; //oversampling
	parameter div_counter = clk_freq/(baud_rate*div_sample);  // this is the number we have to divide the system clock frequency to get a frequency (div_sample) time higher than (baud_rate)
	parameter mid_sample = (div_sample/2);  // this is the middle point of a bit where you want to sample it
	parameter div_bit = 10; // 1 start, 8 data, 1 stop


	assign RxData = rxshiftreg [8:1]; // assign the RxData from the shiftregister

	//UART receiver logic
	always @ (posedge clk)begin 
		if (reset)begin // if reset is asserted
			state <=0; // set state to idle 
			bitcounter <=0; // reset the bit counter
			counter <=0; // reset the counter
			samplecounter <=0; // reset the sample counter
		end else begin // if reset is not asserted
			counter <= counter +1; // start count in the counter
			if (counter >= div_counter-1) begin // if counter reach the baud rate with sampling 
				counter <=0; //reset the counter
				state <= nextstate; // assign the state to nextstate
				if (shift)rxshiftreg <= {RxD,rxshiftreg[9:1]}; //if shift asserted, load the receiving data
				if (clear_samplecounter) samplecounter <=0; // if clear sampl counter asserted, reset sample counter
				if (inc_samplecounter) samplecounter <= samplecounter +1; //if increment counter asserted, start sample count
				if (clear_bitcounter) bitcounter <=0; // if clear bit counter asserted, reset bit counter
				if (inc_bitcounter)bitcounter <= bitcounter +1; // if increment bit counter asserted, start count bit counter
			end
		end
	end
	   
	//state machine

	always @ (posedge clk) begin //trigger by clock
		shift <= 0; // set shift to 0 to avoid any shifting 
		clear_samplecounter <=0; // set clear sample counter to 0 to avoid reset
		inc_samplecounter <=0; // set increment sample counter to 0 to avoid any increment
		clear_bitcounter <=0; // set clear bit counter to 0 to avoid claring
		inc_bitcounter <=0; // set increment bit counter to avoid any count
		nextstate <=0; // set next state to be idle state
		case (state)
			0: begin // idle state
				if (RxD) begin// if input RxD data line asserted 
				    nextstate <=0; // back to idle state because RxD needs to be low to start transmission    
				end else begin // if input RxD data line is not asserted
					nextstate <=1; //jump to receiving state 
					clear_bitcounter <=1; // trigger to clear bit counter
					clear_samplecounter <=1; // trigger to clear sample counter
				end
			end
			1: begin // receiving state
				nextstate <= 1; // DEFAULT 
				if (samplecounter== mid_sample - 1) 
				    shift <= 1; // if sample counter is 1, trigger shift 
				if (samplecounter== div_sample - 1) begin // if sample counter is 3 as the sample rate used is 3
					if (bitcounter == div_bit - 1) begin // check if bit counter if 9 or not
					    nextstate <= 0; // back to idle state if bit counter is 9 as receving is complete
					end 
					inc_bitcounter <=1; // trigger the increment bit counter if bit counter is not 9
					clear_samplecounter <=1; //trigger the sample counter to reset the sample counter
				end else 
				    inc_samplecounter <=1; // if sample is not equal to 3, keep counting
		        end
		    default: nextstate <=0; //default idle state
		endcase
	end         
endmodule

module clock_divider(clk, clk_div);   
    parameter n = 26;     
    input clk;   
    output clk_div;   
    reg [n-1:0] num;
    wire [n-1:0] next_num;
    
    always@(posedge clk)begin
    	num<=next_num;
    end
    
    assign next_num = num +1;
    assign clk_div = num[n-1];  
endmodule

module seven_segment(
    input rst,
    input clk,
    input [3:0] BCD3,
    input [3:0] BCD2,
    input [3:0] BCD1,
    input [3:0] BCD0,
    output reg [3:0] DIGIT,
    output wire [6:0] DISPLAY
    );
    reg [3:0] value;

	always @ (posedge clk) begin	
		case(DIGIT) 
			4'b0111: begin
			    if(rst) value=0;
			    else value = BCD2;
				DIGIT <= 4'b1011;
			end
			4'b1011: begin
			    if(rst) value=0;
			    else value = BCD1;
				DIGIT <= 4'b1101;
			end
			4'b1101: begin
			    if(rst) value=0;
				else value = BCD0;
				DIGIT <= 4'b1110;
			end
			4'b1110: begin
			    if(rst) value=0;
				else value = BCD3;
				DIGIT <= 4'b0111;
			end
			default begin
				DIGIT <= 4'b1110;
			end
		endcase	
	end

	assign DISPLAY = (value==4'd0) ? 7'b0000001 :
					 (value==4'd1) ? 7'b1001111 :
					 (value==4'd2) ? 7'b0010010 :
					 (value==4'd3) ? 7'b0000110 :
					 (value==4'd4) ? 7'b1001100 :
					 (value==4'd5) ? 7'b0100100 :
					 (value==4'd6) ? 7'b0100000 :
					 (value==4'd7) ? 7'b0001111 :
					 (value==4'd8) ? 7'b0000000 : 
					 (value==4'd9) ? 7'b0000100 : 7'b1111111;
	
endmodule

//-------------------------------------------------

module _player_control (
	input clk,
	input reset,
	input _play,
	input _repeat,
	input _music,
	output reg [11:0] ibeat
);
    reg last_music;
    
    always @(posedge clk or posedge reset) begin
        if(_music == 0) begin
            if(reset || last_music) begin
                ibeat = 0;
                last_music = 0;
            end
            else begin
                ibeat = ibeat;
                last_music = last_music;
            end
            
            if(_play) begin
                if(ibeat < `SONG1) begin
                    ibeat = ibeat + 1;
                end
                else if(ibeat >= `SONG1) begin
                    if(_repeat) begin
                        ibeat = 0;
                    end
                    else begin
                        ibeat = 4095;
                    end
                end
            end
            else begin
                ibeat = ibeat;
            end
        end 
        else begin
            if(reset || last_music == 0) begin
                ibeat = 16;
                last_music = 1;
            end
            else begin
                ibeat = ibeat;
                last_music = last_music;
            end
            
            if(_play) begin
                if(ibeat < `SONG2) begin
                    ibeat = ibeat + 1;
                end
                else if(ibeat >= `SONG2) begin
                    if(_repeat) begin
                        ibeat = 16;
                    end
                    else begin
                        ibeat = 4095;
                    end
                end
            end
        end  
    end  
endmodule

module music_example (
	input [11:0] ibeatNum,
	input en,
	output reg [31:0] toneL,
    output reg [31:0] toneR
);

    always @* begin
        if(en == 0) begin
            case(ibeatNum)
                // --- Measure 1 ---
                12'd0: toneR = `hhg;      12'd1: toneR = `hhg;
                12'd2: toneR = `hhg;      12'd3: toneR = `hhg;
                12'd4: toneR = `hhg;      12'd5: toneR = `hhg;
                12'd6: toneR = `hhg;      12'd7: toneR = `hhg;
                12'd8: toneR = `hhg;      12'd9: toneR = `hhg;
                12'd10: toneR = `hhg;      12'd11: toneR = `hhg;
                12'd12: toneR = `hhg;      12'd13: toneR = `hhg;
                12'd14: toneR = `hhg;      12'd15: toneR = `sil;
                
                12'd16: toneR = `hhg;      12'd17: toneR = `hhg;
                12'd18: toneR = `hhg;      12'd19: toneR = `hhg;
                12'd20: toneR = `hhg;      12'd21: toneR = `hhg;
                12'd22: toneR = `hhg;      12'd23: toneR = `hhg;
                12'd24: toneR = `hhg;      12'd25: toneR = `hhg;
                12'd26: toneR = `hhg;      12'd27: toneR = `hhg;
                12'd28: toneR = `hhg;      12'd29: toneR = `hhg;
                12'd30: toneR = `hhg;      12'd31: toneR = `hhg;
                12'd32: toneR = `hhg;      12'd33: toneR = `hhg;

                12'd34: toneR = `hhg;      12'd35: toneR = `hhg;
                12'd36: toneR = `hhg;      12'd37: toneR = `hhg;
                12'd38: toneR = `hhg;      12'd39: toneR = `hhg;
                12'd40: toneR = `hhg;      12'd41: toneR = `hhg;
                12'd42: toneR = `hhg;      12'd43: toneR = `hhg;
                12'd44: toneR = `hhg;      12'd45: toneR = `hhg;
                12'd46: toneR = `hhg;      12'd47: toneR = `hhg;
                12'd48: toneR = `hhg;      12'd49: toneR = `hhg;

                12'd50: toneR = `hhg;      12'd51: toneR = `hhg;
                12'd52: toneR = `hhg;      12'd53: toneR = `hhg;
                12'd54: toneR = `hhg;      12'd55: toneR = `hhg;
                12'd56: toneR = `hhg;      12'd57: toneR = `hhg;
                12'd58: toneR = `hhg;      12'd59: toneR = `hhg;
                12'd60: toneR = `hhg;      12'd61: toneR = `hhg;
                12'd62: toneR = `hhg;      12'd63: toneR = `hhg;
                12'd64: toneR = `hhg;      12'd65: toneR = `hhg;

                12'd66: toneR = `hhg;      12'd67: toneR = `hhg;
                12'd68: toneR = `hhg;      12'd69: toneR = `hhg;
                12'd70: toneR = `hhg;      12'd71: toneR = `hhg;
                12'd72: toneR = `hhg;      12'd73: toneR = `hhg;
                12'd74: toneR = `hhg;      12'd75: toneR = `hhg;
                12'd76: toneR = `hhg;      12'd77: toneR = `hhg;
                12'd78: toneR = `hhg;      12'd79: toneR = `hhg;
                12'd80: toneR = `hhg;      12'd81: toneR = `hhg;

                12'd82: toneR = `hhg;      12'd83: toneR = `hhg;
                12'd84: toneR = `hhg;      12'd85: toneR = `hhg;
                12'd86: toneR = `hhg;      12'd87: toneR = `hhg;
                12'd88: toneR = `hhg;      12'd89: toneR = `hhg;
                12'd90: toneR = `hhg;      12'd91: toneR = `hhg;
                12'd92: toneR = `hhg;      12'd93: toneR = `hhg;
                12'd94: toneR = `hhg;      12'd95: toneR = `hhg;
                12'd96: toneR = `hhg;      12'd97: toneR = `hhg;

                12'd98: toneR = `hhg;      12'd99: toneR = `hhg;
                12'd100: toneR = `hhg;      12'd101: toneR = `hhg;
                12'd102: toneR = `hhg;      12'd103: toneR = `hhg;
                12'd104: toneR = `hhg;      12'd105: toneR = `hhg;
                12'd106: toneR = `hhg;      12'd107: toneR = `hhg;
                12'd108: toneR = `hhg;      12'd109: toneR = `hhg;
                12'd110: toneR = `hhg;      12'd111: toneR = `hhg;
                12'd112: toneR = `hhg;      12'd113: toneR = `hhg;

                12'd114: toneR = `hhg;      12'd115: toneR = `hhg;
                12'd116: toneR = `hhg;      12'd117: toneR = `hhg;
                12'd118: toneR = `hhg;      12'd119: toneR = `hhg;
                12'd120: toneR = `hhg;      12'd121: toneR = `hhg;
                12'd122: toneR = `hhg;      12'd123: toneR = `hhg;
                12'd124: toneR = `hhg;      12'd125: toneR = `hhg;
                12'd126: toneR = `hhg;      12'd127: toneR = `hhg;
                12'd128: toneR = `hhg;      12'd129: toneR = `hhg;

                12'd130: toneR = `hhg;      12'd131: toneR = `hhg;
                12'd132: toneR = `hhg;      12'd133: toneR = `hhg;
                12'd134: toneR = `hhg;      12'd135: toneR = `hhg;
                12'd136: toneR = `hhg;      12'd137: toneR = `hhg;
                12'd138: toneR = `hhg;      12'd139: toneR = `hhg;
                12'd140: toneR = `hhg;      12'd141: toneR = `hhg;
                12'd142: toneR = `hhg;      12'd143: toneR = `hhg;
                12'd144: toneR = `hhg;      12'd145: toneR = `hhg;

                12'd146: toneR = `hhg;      12'd147: toneR = `hhg;
                12'd148: toneR = `hhg;      12'd149: toneR = `hhg;
                12'd150: toneR = `hhg;      12'd151: toneR = `hhg;
                12'd152: toneR = `hhg;      12'd153: toneR = `hhg;
                12'd154: toneR = `hhg;      12'd155: toneR = `hhg;
                12'd156: toneR = `hhg;      12'd157: toneR = `hhg;
                12'd158: toneR = `hhg;      12'd159: toneR = `hhg;
                12'd160: toneR = `hhg;      12'd161: toneR = `hhg;

                12'd162: toneR = `hhg;      12'd163: toneR = `hhg;
                12'd164: toneR = `hhg;      12'd165: toneR = `hhg;
                12'd166: toneR = `hhg;      12'd167: toneR = `hhg;
                12'd168: toneR = `hhg;      12'd169: toneR = `hhg;
                12'd170: toneR = `hhg;      12'd171: toneR = `hhg;
                12'd172: toneR = `hhg;      12'd173: toneR = `hhg;
                12'd174: toneR = `hhg;      12'd175: toneR = `hhg;
                12'd176: toneR = `hhg;      12'd177: toneR = `hhg;

                12'd178: toneR = `hhg;      12'd179: toneR = `hhg;
                12'd180: toneR = `hhg;      12'd181: toneR = `hhg;
                12'd182: toneR = `hhg;      12'd183: toneR = `hhg;
                12'd184: toneR = `hhg;      12'd185: toneR = `hhg;
                12'd186: toneR = `hhg;      12'd187: toneR = `hhg;
                12'd188: toneR = `hhg;      12'd189: toneR = `hhg;
                12'd190: toneR = `hhg;      12'd191: toneR = `hhg;
                12'd192: toneR = `hhg;      12'd193: toneR = `hhg;

                12'd194: toneR = `hhg;      12'd195: toneR = `hhg;
                12'd196: toneR = `hhg;      12'd197: toneR = `hhg;
                12'd198: toneR = `hhg;      12'd199: toneR = `hhg;
                12'd200: toneR = `hhg;      12'd201: toneR = `hhg;
                12'd202: toneR = `hhg;      12'd203: toneR = `hhg;
                12'd204: toneR = `hhg;      12'd205: toneR = `hhg;
                12'd206: toneR = `hhg;      12'd207: toneR = `hhg;
                12'd208: toneR = `hhg;      12'd209: toneR = `hhg;

                12'd210: toneR = `hhg;      12'd211: toneR = `hhg;
                12'd212: toneR = `hhg;      12'd213: toneR = `hhg;
                12'd214: toneR = `hhg;      12'd215: toneR = `hhg;
                12'd216: toneR = `hhg;      12'd217: toneR = `hhg;
                12'd218: toneR = `hhg;      12'd219: toneR = `hhg;
                12'd220: toneR = `hhg;      12'd221: toneR = `hhg;
                12'd222: toneR = `hhg;      12'd223: toneR = `hhg;
                12'd224: toneR = `hhg;      12'd225: toneR = `hhg;

                12'd226: toneR = `hhg;      12'd227: toneR = `hhg;
                12'd228: toneR = `hhg;      12'd229: toneR = `hhg;
                12'd230: toneR = `hhg;      12'd231: toneR = `hhg;
                12'd232: toneR = `hhg;      12'd233: toneR = `hhg;
                12'd234: toneR = `hhg;      12'd235: toneR = `hhg;
                12'd236: toneR = `hhg;      12'd237: toneR = `hhg;
                12'd238: toneR = `hhg;      12'd239: toneR = `hhg;
                12'd240: toneR = `hhg;      12'd241: toneR = `hhg;

                12'd242: toneR = `hhg;      12'd243: toneR = `hhg;
                12'd244: toneR = `hhg;      12'd245: toneR = `hhg;
                12'd246: toneR = `hhg;      12'd247: toneR = `hhg;
                12'd248: toneR = `hhg;      12'd249: toneR = `hhg;
                12'd250: toneR = `hhg;      12'd251: toneR = `hhg;
                12'd252: toneR = `hhg;      12'd253: toneR = `hhg;
                12'd254: toneR = `hhg;      12'd255: toneR = `hhg;
                12'd256: toneR = `hhg;      12'd257: toneR = `hhg;

                12'd258: toneR = `hhg;      12'd259: toneR = `hhg;
                12'd260: toneR = `hhg;      12'd261: toneR = `hhg;
                12'd262: toneR = `hhg;      12'd263: toneR = `hhg;
                12'd264: toneR = `hhg;      12'd265: toneR = `hhg;
                12'd266: toneR = `hhg;      12'd267: toneR = `hhg;
                12'd268: toneR = `hhg;      12'd269: toneR = `hhg;
                12'd270: toneR = `hhg;      12'd271: toneR = `hhg;
                12'd272: toneR = `hhg;      12'd273: toneR = `hhg;

                12'd274: toneR = `hhg;      12'd275: toneR = `hhg;
                12'd276: toneR = `hhg;      12'd277: toneR = `hhg;
                12'd278: toneR = `hhg;      12'd279: toneR = `hhg;
                12'd280: toneR = `hhg;      12'd281: toneR = `hhg;
                12'd282: toneR = `hhg;      12'd283: toneR = `hhg;
                12'd284: toneR = `hhg;      12'd285: toneR = `hhg;
                12'd286: toneR = `hhg;      12'd287: toneR = `hhg;
                12'd288: toneR = `hhg;      12'd289: toneR = `hhg;

                12'd290: toneR = `hhg;      12'd291: toneR = `hhg;
                12'd292: toneR = `hhg;      12'd293: toneR = `hhg;
                12'd294: toneR = `hhg;      12'd295: toneR = `hhg;
                12'd296: toneR = `hhg;      12'd297: toneR = `hhg;
                12'd298: toneR = `hhg;      12'd299: toneR = `hhg;
                12'd300: toneR = `hhg;      12'd301: toneR = `hhg;
                12'd302: toneR = `hhg;      12'd303: toneR = `hhg;
                12'd304: toneR = `hhg;      12'd305: toneR = `hhg;

                12'd306: toneR = `hhg;      12'd307: toneR = `hhg;
                12'd308: toneR = `hhg;      12'd309: toneR = `hhg;
                12'd310: toneR = `hhg;      12'd311: toneR = `hhg;
                12'd312: toneR = `hhg;      12'd313: toneR = `hhg;
                12'd314: toneR = `hhg;      12'd315: toneR = `hhg;
                12'd316: toneR = `hhg;      12'd317: toneR = `hhg;
                12'd318: toneR = `hhg;      12'd319: toneR = `hhg;
                12'd320: toneR = `hhg;      12'd321: toneR = `hhg;

                12'd322: toneR = `hhg;      12'd323: toneR = `hhg;
                12'd324: toneR = `hhg;      12'd325: toneR = `hhg;
                12'd326: toneR = `hhg;      12'd327: toneR = `hhg;
                12'd328: toneR = `hhg;      12'd329: toneR = `hhg;
                12'd330: toneR = `hhg;      12'd331: toneR = `hhg;
                12'd332: toneR = `hhg;      12'd333: toneR = `hhg;
                12'd334: toneR = `hhg;      12'd335: toneR = `hhg;
                12'd336: toneR = `hhg;      12'd337: toneR = `hhg;

                12'd338: toneR = `hhg;      12'd339: toneR = `hhg;
                12'd340: toneR = `hhg;      12'd341: toneR = `hhg;
                12'd342: toneR = `hhg;      12'd343: toneR = `hhg;
                12'd344: toneR = `hhg;      12'd345: toneR = `hhg;
                12'd346: toneR = `hhg;      12'd347: toneR = `hhg;
                12'd348: toneR = `hhg;      12'd349: toneR = `hhg;
                12'd350: toneR = `hhg;      12'd351: toneR = `hhg;
                12'd352: toneR = `hhg;      12'd353: toneR = `hhg;

                12'd354: toneR = `hhg;      12'd355: toneR = `hhg;
                12'd356: toneR = `hhg;      12'd357: toneR = `hhg;
                12'd358: toneR = `hhg;      12'd359: toneR = `hhg;
                12'd360: toneR = `hhg;      12'd361: toneR = `hhg;
                12'd362: toneR = `hhg;      12'd363: toneR = `hhg;
                12'd364: toneR = `hhg;      12'd365: toneR = `hhg;
                12'd366: toneR = `hhg;      12'd367: toneR = `hhg;
                12'd368: toneR = `hhg;      12'd369: toneR = `hhg;

                12'd370: toneR = `hhg;      12'd371: toneR = `hhg;
                12'd372: toneR = `hhg;      12'd373: toneR = `hhg;
                12'd374: toneR = `hhg;      12'd375: toneR = `hhg;
                12'd376: toneR = `hhg;      12'd377: toneR = `hhg;
                12'd378: toneR = `hhg;      12'd379: toneR = `hhg;
                12'd380: toneR = `hhg;      12'd381: toneR = `hhg;
                12'd382: toneR = `hhg;      12'd383: toneR = `hhg;
                12'd384: toneR = `hhg;      12'd385: toneR = `hhg;

                12'd386: toneR = `hhg;      12'd387: toneR = `hhg;
                12'd388: toneR = `hhg;      12'd389: toneR = `hhg;
                12'd390: toneR = `hhg;      12'd391: toneR = `hhg;
                12'd392: toneR = `hhg;      12'd393: toneR = `hhg;
                12'd394: toneR = `hhg;      12'd395: toneR = `hhg;
                12'd396: toneR = `hhg;      12'd397: toneR = `hhg;
                12'd398: toneR = `hhg;      12'd399: toneR = `hhg;
                12'd400: toneR = `hhg;      12'd401: toneR = `hhg;

                12'd402: toneR = `hhg;      12'd403: toneR = `hhg;
                12'd404: toneR = `hhg;      12'd405: toneR = `hhg;
                12'd406: toneR = `hhg;      12'd407: toneR = `hhg;
                12'd408: toneR = `hhg;      12'd409: toneR = `hhg;
                12'd410: toneR = `hhg;      12'd411: toneR = `hhg;
                12'd412: toneR = `hhg;      12'd413: toneR = `hhg;
                12'd414: toneR = `hhg;      12'd415: toneR = `hhg;
                12'd416: toneR = `hhg;      12'd417: toneR = `hhg;

                12'd418: toneR = `hhg;      12'd419: toneR = `hhg;
                12'd420: toneR = `hhg;      12'd421: toneR = `hhg;
                12'd422: toneR = `hhg;      12'd423: toneR = `hhg;
                12'd424: toneR = `hhg;      12'd425: toneR = `hhg;
                12'd426: toneR = `hhg;      12'd427: toneR = `hhg;
                12'd428: toneR = `hhg;      12'd429: toneR = `hhg;
                12'd430: toneR = `hhg;      12'd431: toneR = `hhg;
                12'd432: toneR = `hhg;      12'd433: toneR = `hhg;

                12'd434: toneR = `hhg;      12'd435: toneR = `hhg;
                12'd436: toneR = `hhg;      12'd437: toneR = `hhg;
                12'd438: toneR = `hhg;      12'd439: toneR = `hhg;
                12'd440: toneR = `hhg;      12'd441: toneR = `hhg;
                12'd442: toneR = `hhg;      12'd443: toneR = `hhg;
                12'd444: toneR = `hhg;      12'd445: toneR = `hhg;
                12'd446: toneR = `hhg;      12'd447: toneR = `hhg;
                12'd448: toneR = `hhg;      12'd449: toneR = `hhg;

                12'd450: toneR = `hhg;      12'd451: toneR = `hhg;
                12'd452: toneR = `hhg;      12'd453: toneR = `hhg;
                12'd454: toneR = `hhg;      12'd455: toneR = `hhg;
                12'd456: toneR = `hhg;      12'd457: toneR = `hhg;
                12'd458: toneR = `hhg;      12'd459: toneR = `hhg;
                12'd460: toneR = `hhg;      12'd461: toneR = `hhg;
                12'd462: toneR = `hhg;      12'd463: toneR = `hhg;
                12'd464: toneR = `hhg;      12'd465: toneR = `hhg;

                12'd466: toneR = `hhg;      12'd467: toneR = `hhg;
                12'd468: toneR = `hhg;      12'd469: toneR = `hhg;
                12'd470: toneR = `hhg;      12'd471: toneR = `hhg;
                12'd472: toneR = `hhg;      12'd473: toneR = `hhg;
                12'd474: toneR = `hhg;      12'd475: toneR = `hhg;
                12'd476: toneR = `hhg;      12'd477: toneR = `hhg;
                12'd478: toneR = `hhg;      12'd479: toneR = `hhg;
                12'd480: toneR = `hhg;      12'd481: toneR = `hhg;

                12'd482: toneR = `hhg;      12'd483: toneR = `hhg;
                12'd484: toneR = `hhg;      12'd485: toneR = `hhg;
                12'd486: toneR = `hhg;      12'd487: toneR = `hhg;
                12'd488: toneR = `hhg;      12'd489: toneR = `hhg;
                12'd490: toneR = `hhg;      12'd491: toneR = `hhg;
                12'd492: toneR = `hhg;      12'd493: toneR = `hhg;
                12'd494: toneR = `hhg;      12'd495: toneR = `hhg;
                12'd496: toneR = `hhg;      12'd497: toneR = `hhg;

                12'd498: toneR = `hhg;      12'd499: toneR = `sil;
                
                
                
                default: toneR = `sil;
            endcase
        end else begin
            toneR = `sil;
        end
    end

    always @(*) begin
        if(en==0)begin
            case(ibeatNum)
                // --- Measure 1 ---
                12'd0: toneL = `hhg;      12'd1: toneL = `hhg;
                12'd2: toneL = `hhg;      12'd3: toneL = `hhg;
                12'd4: toneL = `hhg;      12'd5: toneL = `hhg;
                12'd6: toneL = `hhg;      12'd7: toneL = `hhg;
                12'd8: toneL = `hhg;      12'd9: toneL = `hhg;
                12'd10: toneL = `hhg;      12'd11: toneL = `hhg;
                12'd12: toneL = `hhg;      12'd13: toneL = `hhg;
                12'd14: toneL = `hhg;      12'd15: toneL = `hhg;
                12'd16: toneL = `hhg;      12'd17: toneL = `hhg;

                12'd18: toneL = `hhg;      12'd19: toneL = `hhg;
                12'd20: toneL = `hhg;      12'd21: toneL = `hhg;
                12'd22: toneL = `hhg;      12'd23: toneL = `hhg;
                12'd24: toneL = `hhg;      12'd25: toneL = `hhg;
                12'd26: toneL = `hhg;      12'd27: toneL = `hhg;
                12'd28: toneL = `hhg;      12'd29: toneL = `hhg;
                12'd30: toneL = `hhg;      12'd31: toneL = `hhg;
                12'd32: toneL = `hhg;      12'd33: toneL = `hhg;

                12'd34: toneL = `hhg;      12'd35: toneL = `hhg;
                12'd36: toneL = `hhg;      12'd37: toneL = `hhg;
                12'd38: toneL = `hhg;      12'd39: toneL = `hhg;
                12'd40: toneL = `hhg;      12'd41: toneL = `hhg;
                12'd42: toneL = `hhg;      12'd43: toneL = `hhg;
                12'd44: toneL = `hhg;      12'd45: toneL = `hhg;
                12'd46: toneL = `hhg;      12'd47: toneL = `hhg;
                12'd48: toneL = `hhg;      12'd49: toneL = `hhg;

                12'd50: toneL = `hhg;      12'd51: toneL = `hhg;
                12'd52: toneL = `hhg;      12'd53: toneL = `hhg;
                12'd54: toneL = `hhg;      12'd55: toneL = `hhg;
                12'd56: toneL = `hhg;      12'd57: toneL = `hhg;
                12'd58: toneL = `hhg;      12'd59: toneL = `hhg;
                12'd60: toneL = `hhg;      12'd61: toneL = `hhg;
                12'd62: toneL = `hhg;      12'd63: toneL = `hhg;
                12'd64: toneL = `hhg;      12'd65: toneL = `hhg;

                12'd66: toneL = `hhg;      12'd67: toneL = `hhg;
                12'd68: toneL = `hhg;      12'd69: toneL = `hhg;
                12'd70: toneL = `hhg;      12'd71: toneL = `hhg;
                12'd72: toneL = `hhg;      12'd73: toneL = `hhg;
                12'd74: toneL = `hhg;      12'd75: toneL = `hhg;
                12'd76: toneL = `hhg;      12'd77: toneL = `hhg;
                12'd78: toneL = `hhg;      12'd79: toneL = `hhg;
                12'd80: toneL = `hhg;      12'd81: toneL = `hhg;

                12'd82: toneL = `hhg;      12'd83: toneL = `hhg;
                12'd84: toneL = `hhg;      12'd85: toneL = `hhg;
                12'd86: toneL = `hhg;      12'd87: toneL = `hhg;
                12'd88: toneL = `hhg;      12'd89: toneL = `hhg;
                12'd90: toneL = `hhg;      12'd91: toneL = `hhg;
                12'd92: toneL = `hhg;      12'd93: toneL = `hhg;
                12'd94: toneL = `hhg;      12'd95: toneL = `hhg;
                12'd96: toneL = `hhg;      12'd97: toneL = `hhg;

                12'd98: toneL = `hhg;      12'd99: toneL = `hhg;
                12'd100: toneL = `hhg;      12'd101: toneL = `hhg;
                12'd102: toneL = `hhg;      12'd103: toneL = `hhg;
                12'd104: toneL = `hhg;      12'd105: toneL = `hhg;
                12'd106: toneL = `hhg;      12'd107: toneL = `hhg;
                12'd108: toneL = `hhg;      12'd109: toneL = `hhg;
                12'd110: toneL = `hhg;      12'd111: toneL = `hhg;
                12'd112: toneL = `hhg;      12'd113: toneL = `hhg;

                12'd114: toneL = `hhg;      12'd115: toneL = `hhg;
                12'd116: toneL = `hhg;      12'd117: toneL = `hhg;
                12'd118: toneL = `hhg;      12'd119: toneL = `hhg;
                12'd120: toneL = `hhg;      12'd121: toneL = `hhg;
                12'd122: toneL = `hhg;      12'd123: toneL = `hhg;
                12'd124: toneL = `hhg;      12'd125: toneL = `hhg;
                12'd126: toneL = `hhg;      12'd127: toneL = `hhg;
                12'd128: toneL = `hhg;      12'd129: toneL = `hhg;

                12'd130: toneL = `hhg;      12'd131: toneL = `hhg;
                12'd132: toneL = `hhg;      12'd133: toneL = `hhg;
                12'd134: toneL = `hhg;      12'd135: toneL = `hhg;
                12'd136: toneL = `hhg;      12'd137: toneL = `hhg;
                12'd138: toneL = `hhg;      12'd139: toneL = `hhg;
                12'd140: toneL = `hhg;      12'd141: toneL = `hhg;
                12'd142: toneL = `hhg;      12'd143: toneL = `hhg;
                12'd144: toneL = `hhg;      12'd145: toneL = `hhg;

                12'd146: toneL = `hhg;      12'd147: toneL = `hhg;
                12'd148: toneL = `hhg;      12'd149: toneL = `hhg;
                12'd150: toneL = `hhg;      12'd151: toneL = `hhg;
                12'd152: toneL = `hhg;      12'd153: toneL = `hhg;
                12'd154: toneL = `hhg;      12'd155: toneL = `hhg;
                12'd156: toneL = `hhg;      12'd157: toneL = `hhg;
                12'd158: toneL = `hhg;      12'd159: toneL = `hhg;
                12'd160: toneL = `hhg;      12'd161: toneL = `hhg;

                12'd162: toneL = `hhg;      12'd163: toneL = `hhg;
                12'd164: toneL = `hhg;      12'd165: toneL = `hhg;
                12'd166: toneL = `hhg;      12'd167: toneL = `hhg;
                12'd168: toneL = `hhg;      12'd169: toneL = `hhg;
                12'd170: toneL = `hhg;      12'd171: toneL = `hhg;
                12'd172: toneL = `hhg;      12'd173: toneL = `hhg;
                12'd174: toneL = `hhg;      12'd175: toneL = `hhg;
                12'd176: toneL = `hhg;      12'd177: toneL = `hhg;

                12'd178: toneL = `hhg;      12'd179: toneL = `hhg;
                12'd180: toneL = `hhg;      12'd181: toneL = `hhg;
                12'd182: toneL = `hhg;      12'd183: toneL = `hhg;
                12'd184: toneL = `hhg;      12'd185: toneL = `hhg;
                12'd186: toneL = `hhg;      12'd187: toneL = `hhg;
                12'd188: toneL = `hhg;      12'd189: toneL = `hhg;
                12'd190: toneL = `hhg;      12'd191: toneL = `hhg;
                12'd192: toneL = `hhg;      12'd193: toneL = `hhg;

                12'd194: toneL = `hhg;      12'd195: toneL = `hhg;
                12'd196: toneL = `hhg;      12'd197: toneL = `hhg;
                12'd198: toneL = `hhg;      12'd199: toneL = `hhg;
                12'd200: toneL = `hhg;      12'd201: toneL = `hhg;
                12'd202: toneL = `hhg;      12'd203: toneL = `hhg;
                12'd204: toneL = `hhg;      12'd205: toneL = `hhg;
                12'd206: toneL = `hhg;      12'd207: toneL = `hhg;
                12'd208: toneL = `hhg;      12'd209: toneL = `hhg;

                12'd210: toneL = `hhg;      12'd211: toneL = `hhg;
                12'd212: toneL = `hhg;      12'd213: toneL = `hhg;
                12'd214: toneL = `hhg;      12'd215: toneL = `hhg;
                12'd216: toneL = `hhg;      12'd217: toneL = `hhg;
                12'd218: toneL = `hhg;      12'd219: toneL = `hhg;
                12'd220: toneL = `hhg;      12'd221: toneL = `hhg;
                12'd222: toneL = `hhg;      12'd223: toneL = `hhg;
                12'd224: toneL = `hhg;      12'd225: toneL = `hhg;

                12'd226: toneL = `hhg;      12'd227: toneL = `hhg;
                12'd228: toneL = `hhg;      12'd229: toneL = `hhg;
                12'd230: toneL = `hhg;      12'd231: toneL = `hhg;
                12'd232: toneL = `hhg;      12'd233: toneL = `hhg;
                12'd234: toneL = `hhg;      12'd235: toneL = `hhg;
                12'd236: toneL = `hhg;      12'd237: toneL = `hhg;
                12'd238: toneL = `hhg;      12'd239: toneL = `hhg;
                12'd240: toneL = `hhg;      12'd241: toneL = `hhg;

                12'd242: toneL = `hhg;      12'd243: toneL = `hhg;
                12'd244: toneL = `hhg;      12'd245: toneL = `hhg;
                12'd246: toneL = `hhg;      12'd247: toneL = `hhg;
                12'd248: toneL = `hhg;      12'd249: toneL = `hhg;
                12'd250: toneL = `hhg;      12'd251: toneL = `hhg;
                12'd252: toneL = `hhg;      12'd253: toneL = `hhg;
                12'd254: toneL = `hhg;      12'd255: toneL = `hhg;
                12'd256: toneL = `hhg;      12'd257: toneL = `hhg;

                12'd258: toneL = `hhg;      12'd259: toneL = `hhg;
                12'd260: toneL = `hhg;      12'd261: toneL = `hhg;
                12'd262: toneL = `hhg;      12'd263: toneL = `hhg;
                12'd264: toneL = `hhg;      12'd265: toneL = `hhg;
                12'd266: toneL = `hhg;      12'd267: toneL = `hhg;
                12'd268: toneL = `hhg;      12'd269: toneL = `hhg;
                12'd270: toneL = `hhg;      12'd271: toneL = `hhg;
                12'd272: toneL = `hhg;      12'd273: toneL = `hhg;

                12'd274: toneL = `hhg;      12'd275: toneL = `hhg;
                12'd276: toneL = `hhg;      12'd277: toneL = `hhg;
                12'd278: toneL = `hhg;      12'd279: toneL = `hhg;
                12'd280: toneL = `hhg;      12'd281: toneL = `hhg;
                12'd282: toneL = `hhg;      12'd283: toneL = `hhg;
                12'd284: toneL = `hhg;      12'd285: toneL = `hhg;
                12'd286: toneL = `hhg;      12'd287: toneL = `hhg;
                12'd288: toneL = `hhg;      12'd289: toneL = `hhg;

                12'd290: toneL = `hhg;      12'd291: toneL = `hhg;
                12'd292: toneL = `hhg;      12'd293: toneL = `hhg;
                12'd294: toneL = `hhg;      12'd295: toneL = `hhg;
                12'd296: toneL = `hhg;      12'd297: toneL = `hhg;
                12'd298: toneL = `hhg;      12'd299: toneL = `hhg;
                12'd300: toneL = `hhg;      12'd301: toneL = `hhg;
                12'd302: toneL = `hhg;      12'd303: toneL = `hhg;
                12'd304: toneL = `hhg;      12'd305: toneL = `hhg;

                12'd306: toneL = `hhg;      12'd307: toneL = `hhg;
                12'd308: toneL = `hhg;      12'd309: toneL = `hhg;
                12'd310: toneL = `hhg;      12'd311: toneL = `hhg;
                12'd312: toneL = `hhg;      12'd313: toneL = `hhg;
                12'd314: toneL = `hhg;      12'd315: toneL = `hhg;
                12'd316: toneL = `hhg;      12'd317: toneL = `hhg;
                12'd318: toneL = `hhg;      12'd319: toneL = `hhg;
                12'd320: toneL = `hhg;      12'd321: toneL = `hhg;

                12'd322: toneL = `hhg;      12'd323: toneL = `hhg;
                12'd324: toneL = `hhg;      12'd325: toneL = `hhg;
                12'd326: toneL = `hhg;      12'd327: toneL = `hhg;
                12'd328: toneL = `hhg;      12'd329: toneL = `hhg;
                12'd330: toneL = `hhg;      12'd331: toneL = `hhg;
                12'd332: toneL = `hhg;      12'd333: toneL = `hhg;
                12'd334: toneL = `hhg;      12'd335: toneL = `hhg;
                12'd336: toneL = `hhg;      12'd337: toneL = `hhg;

                12'd338: toneL = `hhg;      12'd339: toneL = `hhg;
                12'd340: toneL = `hhg;      12'd341: toneL = `hhg;
                12'd342: toneL = `hhg;      12'd343: toneL = `hhg;
                12'd344: toneL = `hhg;      12'd345: toneL = `hhg;
                12'd346: toneL = `hhg;      12'd347: toneL = `hhg;
                12'd348: toneL = `hhg;      12'd349: toneL = `hhg;
                12'd350: toneL = `hhg;      12'd351: toneL = `hhg;
                12'd352: toneL = `hhg;      12'd353: toneL = `hhg;

                12'd354: toneL = `hhg;      12'd355: toneL = `hhg;
                12'd356: toneL = `hhg;      12'd357: toneL = `hhg;
                12'd358: toneL = `hhg;      12'd359: toneL = `hhg;
                12'd360: toneL = `hhg;      12'd361: toneL = `hhg;
                12'd362: toneL = `hhg;      12'd363: toneL = `hhg;
                12'd364: toneL = `hhg;      12'd365: toneL = `hhg;
                12'd366: toneL = `hhg;      12'd367: toneL = `hhg;
                12'd368: toneL = `hhg;      12'd369: toneL = `hhg;

                12'd370: toneL = `hhg;      12'd371: toneL = `hhg;
                12'd372: toneL = `hhg;      12'd373: toneL = `hhg;
                12'd374: toneL = `hhg;      12'd375: toneL = `hhg;
                12'd376: toneL = `hhg;      12'd377: toneL = `hhg;
                12'd378: toneL = `hhg;      12'd379: toneL = `hhg;
                12'd380: toneL = `hhg;      12'd381: toneL = `hhg;
                12'd382: toneL = `hhg;      12'd383: toneL = `hhg;
                12'd384: toneL = `hhg;      12'd385: toneL = `hhg;

                12'd386: toneL = `hhg;      12'd387: toneL = `hhg;
                12'd388: toneL = `hhg;      12'd389: toneL = `hhg;
                12'd390: toneL = `hhg;      12'd391: toneL = `hhg;
                12'd392: toneL = `hhg;      12'd393: toneL = `hhg;
                12'd394: toneL = `hhg;      12'd395: toneL = `hhg;
                12'd396: toneL = `hhg;      12'd397: toneL = `hhg;
                12'd398: toneL = `hhg;      12'd399: toneL = `hhg;
                12'd400: toneL = `hhg;      12'd401: toneL = `hhg;

                12'd402: toneL = `hhg;      12'd403: toneL = `hhg;
                12'd404: toneL = `hhg;      12'd405: toneL = `hhg;
                12'd406: toneL = `hhg;      12'd407: toneL = `hhg;
                12'd408: toneL = `hhg;      12'd409: toneL = `hhg;
                12'd410: toneL = `hhg;      12'd411: toneL = `hhg;
                12'd412: toneL = `hhg;      12'd413: toneL = `hhg;
                12'd414: toneL = `hhg;      12'd415: toneL = `hhg;
                12'd416: toneL = `hhg;      12'd417: toneL = `hhg;

                12'd418: toneL = `hhg;      12'd419: toneL = `hhg;
                12'd420: toneL = `hhg;      12'd421: toneL = `hhg;
                12'd422: toneL = `hhg;      12'd423: toneL = `hhg;
                12'd424: toneL = `hhg;      12'd425: toneL = `hhg;
                12'd426: toneL = `hhg;      12'd427: toneL = `hhg;
                12'd428: toneL = `hhg;      12'd429: toneL = `hhg;
                12'd430: toneL = `hhg;      12'd431: toneL = `hhg;
                12'd432: toneL = `hhg;      12'd433: toneL = `hhg;

                12'd434: toneL = `hhg;      12'd435: toneL = `hhg;
                12'd436: toneL = `hhg;      12'd437: toneL = `hhg;
                12'd438: toneL = `hhg;      12'd439: toneL = `hhg;
                12'd440: toneL = `hhg;      12'd441: toneL = `hhg;
                12'd442: toneL = `hhg;      12'd443: toneL = `hhg;
                12'd444: toneL = `hhg;      12'd445: toneL = `hhg;
                12'd446: toneL = `hhg;      12'd447: toneL = `hhg;
                12'd448: toneL = `hhg;      12'd449: toneL = `hhg;

                12'd450: toneL = `hhg;      12'd451: toneL = `hhg;
                12'd452: toneL = `hhg;      12'd453: toneL = `hhg;
                12'd454: toneL = `hhg;      12'd455: toneL = `hhg;
                12'd456: toneL = `hhg;      12'd457: toneL = `hhg;
                12'd458: toneL = `hhg;      12'd459: toneL = `hhg;
                12'd460: toneL = `hhg;      12'd461: toneL = `hhg;
                12'd462: toneL = `hhg;      12'd463: toneL = `hhg;
                12'd464: toneL = `hhg;      12'd465: toneL = `hhg;

                12'd466: toneL = `hhg;      12'd467: toneL = `hhg;
                12'd468: toneL = `hhg;      12'd469: toneL = `hhg;
                12'd470: toneL = `hhg;      12'd471: toneL = `hhg;
                12'd472: toneL = `hhg;      12'd473: toneL = `hhg;
                12'd474: toneL = `hhg;      12'd475: toneL = `hhg;
                12'd476: toneL = `hhg;      12'd477: toneL = `hhg;
                12'd478: toneL = `hhg;      12'd479: toneL = `hhg;
                12'd480: toneL = `hhg;      12'd481: toneL = `hhg;

                12'd482: toneL = `hhg;      12'd483: toneL = `hhg;
                12'd484: toneL = `hhg;      12'd485: toneL = `hhg;
                12'd486: toneL = `hhg;      12'd487: toneL = `hhg;
                12'd488: toneL = `hhg;      12'd489: toneL = `hhg;
                12'd490: toneL = `hhg;      12'd491: toneL = `hhg;
                12'd492: toneL = `hhg;      12'd493: toneL = `hhg;
                12'd494: toneL = `hhg;      12'd495: toneL = `hhg;
                12'd496: toneL = `hhg;      12'd497: toneL = `hhg;

                12'd498: toneL = `hhg;      12'd499: toneL = `sil;
				
                default : toneL = `sil;
            endcase
        end
        else begin
            toneL = `sil;
        end
    end
endmodule

module speaker_control(
    clk,  // clock from the crystal
    rst,  // active high reset
    audio_in_left, // left channel audio data input
    audio_in_right, // right channel audio data input
    audio_mclk, // master clock
    audio_lrck, // left-right clock, Word Select clock, or sample rate clock
    audio_sck, // serial clock
    audio_sdin // serial audio data input
);

    // I/O declaration
    input clk;  // clock from the crystal
    input rst;  // active high reset
    input [15:0] audio_in_left; // left channel audio data input
    input [15:0] audio_in_right; // right channel audio data input
    output audio_mclk; // master clock
    output audio_lrck; // left-right clock
    output audio_sck; // serial clock
    output audio_sdin; // serial audio data input
    reg audio_sdin;

    // Declare internal signal nodes 
    wire [8:0] clk_cnt_next;
    reg [8:0] clk_cnt;
    reg [15:0] audio_left, audio_right;

    // Counter for the clock divider
    assign clk_cnt_next = clk_cnt + 1'b1;

    always @(posedge clk or posedge rst)
        if (rst == 1'b1)
            clk_cnt <= 9'd0;
        else
            clk_cnt <= clk_cnt_next;

    // Assign divided clock output
    assign audio_mclk = clk_cnt[1];
    assign audio_lrck = clk_cnt[8];
    assign audio_sck = 1'b1; // use internal serial clock mode

    // audio input data buffer
    always @(posedge clk_cnt[8] or posedge rst)
        if (rst == 1'b1)
            begin
                audio_left <= 16'd0;
                audio_right <= 16'd0;
            end
        else
            begin
                audio_left <= audio_in_left;
                audio_right <= audio_in_right;
            end

    always @*
        case (clk_cnt[8:4])
            5'b00000: audio_sdin = audio_right[0];
            5'b00001: audio_sdin = audio_left[15];
            5'b00010: audio_sdin = audio_left[14];
            5'b00011: audio_sdin = audio_left[13];
            5'b00100: audio_sdin = audio_left[12];
            5'b00101: audio_sdin = audio_left[11];
            5'b00110: audio_sdin = audio_left[10];
            5'b00111: audio_sdin = audio_left[9];
            5'b01000: audio_sdin = audio_left[8];
            5'b01001: audio_sdin = audio_left[7];
            5'b01010: audio_sdin = audio_left[6];
            5'b01011: audio_sdin = audio_left[5];
            5'b01100: audio_sdin = audio_left[4];
            5'b01101: audio_sdin = audio_left[3];
            5'b01110: audio_sdin = audio_left[2];
            5'b01111: audio_sdin = audio_left[1];
            5'b10000: audio_sdin = audio_left[0];
            5'b10001: audio_sdin = audio_right[15];
            5'b10010: audio_sdin = audio_right[14];
            5'b10011: audio_sdin = audio_right[13];
            5'b10100: audio_sdin = audio_right[12];
            5'b10101: audio_sdin = audio_right[11];
            5'b10110: audio_sdin = audio_right[10];
            5'b10111: audio_sdin = audio_right[9];
            5'b11000: audio_sdin = audio_right[8];
            5'b11001: audio_sdin = audio_right[7];
            5'b11010: audio_sdin = audio_right[6];
            5'b11011: audio_sdin = audio_right[5];
            5'b11100: audio_sdin = audio_right[4];
            5'b11101: audio_sdin = audio_right[3];
            5'b11110: audio_sdin = audio_right[2];
            5'b11111: audio_sdin = audio_right[1];
            default: audio_sdin = 1'b0;
        endcase
endmodule

module note_gen(
    clk, // clock from crystal
    rst, // active high reset
    note_div_left, // div for note generation
    note_div_right,
    audio_left,
    audio_right,
    volume
);

    // I/O declaration
    input clk; // clock from crystal
    input rst; // active low reset
    input [21:0] note_div_left, note_div_right; // div for note generation
    output reg [15:0] audio_left, audio_right;
    input [2:0] volume;

    // Declare internal signals
    reg [21:0] clk_cnt_next, clk_cnt;
    reg [21:0] clk_cnt_next_2, clk_cnt_2;
    reg b_clk, b_clk_next;
    reg c_clk, c_clk_next;

    // Note frequency generation
    always @(posedge clk or posedge rst)
        if (rst == 1'b1)
            begin
                clk_cnt <= 22'd0;
                clk_cnt_2 <= 22'd0;
                b_clk <= 1'b0;
                c_clk <= 1'b0;
            end
        else
            begin
                clk_cnt <= clk_cnt_next;
                clk_cnt_2 <= clk_cnt_next_2;
                b_clk <= b_clk_next;
                c_clk <= c_clk_next;
            end
        
    always @*
        if (clk_cnt == note_div_left)
            begin
                clk_cnt_next = 22'd0;
                b_clk_next = ~b_clk;
            end
        else
            begin
                clk_cnt_next = clk_cnt + 1'b1;
                b_clk_next = b_clk;
            end

    always @*
        if (clk_cnt_2 == note_div_right)
            begin
                clk_cnt_next_2 = 22'd0;
                c_clk_next = ~c_clk;
            end
        else
            begin
                clk_cnt_next_2 = clk_cnt_2 + 1'b1;
                c_clk_next = c_clk;
            end

    // Assign the amplitude of the note
    // Volume is controlled here
    //assign audio_left = (note_div_left == 22'd1) ? 16'h0000 : (b_clk == 1'b0) ? 16'hE000 : 16'h2000;
    //assign audio_right = (note_div_right == 22'd1) ? 16'h0000 : (c_clk == 1'b0) ? 16'hE000 : 16'h2000;
    
    always @(posedge clk) begin
        if(note_div_left == 22'd1000 && note_div_right == 22'd1000) begin
           audio_left = 16'h0;
           audio_right = 16'h0; 
        end 
        else if (note_div_left == 22'd1000) begin
            audio_left = 16'h0;
        end
        else if (note_div_right == 22'd1000) begin
            audio_right = 16'h0;
        end
        else begin
            if(volume == 0) begin
                audio_right = 16'h0;
                audio_left = 16'h0;
            end
            else if(volume == 4) begin
                audio_right = (b_clk) ? 16'h7FFF : 16'h8001;
                audio_left = (c_clk) ? 16'h7FFF : 16'h8001;
            end
            else begin
                audio_left = audio_left;
                audio_right = audio_right;
            end
        end
    end
endmodule

//-------------------------------------------------

module sonic(
    input clock,
    output trig,
    input echo,
    output reg [32:0] distance
    );
    reg [32:0] distance_temp = 0;
    wire clkDiv25;
    clock_divider #(.n(25)) clock_25(
        .clk(clock),
        .clk_div(clkDiv25)
    );
    
    reg [32:0] us_counter = 0;
    reg _trig = 1'b0;
    
    reg [9:0] one_us_cnt = 0;
    wire one_us = (one_us_cnt == 0);
    
    reg [9:0] ten_us_cnt = 0;
    wire ten_us = (ten_us_cnt == 0);
    
    reg [21:0] forty_ms_cnt = 0;
    wire forty_ms = (forty_ms_cnt == 0);
    
    assign trig = _trig;
    
    always @(posedge clock) begin
        one_us_cnt <= (one_us ? 100 : one_us_cnt) - 1;
        ten_us_cnt <= (ten_us ? 1000 : ten_us_cnt) - 1;
        forty_ms_cnt <= (forty_ms ? 4000000 : forty_ms_cnt) - 1;
        
        if (ten_us && _trig)
            _trig <= 1'b0;
        
        if (one_us) begin	
            if (echo)
                us_counter <= us_counter + 1;
            else if (us_counter) begin
                distance_temp <= us_counter / 58;
                us_counter <= 0;
            end
        end
        
       if (forty_ms)
            _trig <= 1'b1;
    end
    
    always@(posedge clkDiv25) begin
        distance = distance_temp;
    end
    
endmodule