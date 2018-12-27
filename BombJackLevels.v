module BombJackLevels(CLOCK_50, KEY, SW, VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N, VGA_R, VGA_G, VGA_B, HEX0, HEX1, HEX2, HEX3, HEX4, HEX5, LEDR0);
	input CLOCK_50;
	input [3:0]KEY;
	input [9:0] SW;
	
	reg exit;
	output VGA_CLK, VGA_HS, VGA_VS, VGA_BLANK_N, VGA_SYNC_N;
	output [9:0] VGA_R, VGA_G, VGA_B;
	output [6:0] HEX0, HEX1, HEX2, HEX3, HEX4, HEX5;
	output [0:0] LEDR0;
	
	wire left, right, up, down;
	assign left = KEY[3];
	assign right = KEY[0];
	assign up = KEY[1];
	assign down = KEY[2];
	
	//stage 1 lose signals
	wire lose_bomb1;
	wire lose_bomb2;
	wire lose_bomb3;
	//stage 2 lose signals
	wire lose_bomb4;
	wire lose_bomb5;
	wire lose_bomb6;
	
	wire resetn;
	wire resetn_S2;
	wire resetn_S3;
	wire clock;
	wire [8:0] colour;
	wire [7:0] x;
	wire [6:0] y;
	reg[3:0] item_selector;
	reg writeEn;
	
	assign clock = CLOCK_50;
	assign resetn = SW[0];
	assign resetn_S2 = SW[7];
	assign resetn_S3 = SW[8];
	assign resetn_S4 = SW[9];
	
	//stage 1 has three bombs
	wire done_bomb1;
	wire done_bomb2;
	wire done_bomb3;
	//stage 2 has six bombs
	wire done_bomb4;
	wire done_bomb5;
	wire done_bomb6;
	
	//stage 1 done draw signals
	wire done_plyr;
	//wire done_plyr_S2;
	wire done_black;
	wire done_gameover;
	//stage two done draw bg signal
	wire done_bgS2;
	wire done_bgS3;
	wire done_bgS4;
	
	reg done_wait;
	//reg done_wait_S2;
	
	reg go_bomb1;
	reg go_bomb2;
	reg go_bomb3;
	reg go_plyr;
	//reg go_plyr_S2;
	reg go_black;
	reg go_gameover;
	reg go_wait;
	//reg go_wait_S2;
	//stage 2 go signals
	reg go_bomb4;
	reg go_bomb5;
	reg go_bomb6;
	reg go_bgS2;
	reg go_bgS3;
	reg go_bgS4;
	
	
	wire [7:0] x_bomb1; //vga_x
	wire [7:0] x_bomb2;
	wire [7:0] x_bomb3;
	wire [7:0] x_bomb4;
	wire [7:0] x_bomb5;
	wire [7:0] x_bomb6;
	
	wire [6:0] y_bomb1; //vga_y
	wire [6:0] y_bomb2;
	wire [6:0] y_bomb3;
	wire [6:0] y_bomb4;
	wire [6:0] y_bomb5;
	wire [6:0] y_bomb6;
	
	wire [7:0] x_plyr; //vga_x for jack
	wire [6:0] y_plyr; //vga_y for jack
	wire [8:0] colour_plyr;
	
	//wire [7:0] x_plyr_S2; //vga_x for jack
	//wire [6:0] y_plyr_S2; //vga_y for jack
	//wire [8:0] colour_plyr_S2;
	
	
	wire [7:0] x_black; //vga_x for stage 1 bg
	wire [6:0] y_black; //vga_y for stage 1 bg
	wire [8:0] colour_black; //stage 1 bg colour
	
	wire [7:0] x_bgS2; //vga_x for stage 2 bg
	wire [6:0] y_bgS2; //vga_y for stage 2 bg
	wire [8:0] colour_bgS2; //stage 2 bg colour
	
	wire [7:0] x_bgS3; //vga_x for stage 2 bg
	wire [6:0] y_bgS3; //vga_y for stage 2 bg
	wire [8:0] colour_bgS3; //stage 2 bg colour
	
	wire [7:0] x_bgS4; //vga_x for stage 2 bg
	wire [6:0] y_bgS4; //vga_y for stage 2 bg
	wire [8:0] colour_bgS4; //stage 2 bg colour
	
	
	//assign x_bgS2 = x_black;
	//assign y_bgS2 = y_black;
	
	wire [7:0] x_gameover;
	wire [6:0] y_gameover;
	wire [8:0] colour_gameover;
	
	reg [7:0] start_bomb1_x = 20;
	reg [7:0] start_bomb2_x = 83;
	reg [7:0] start_bomb3_x = 145;
	reg [7:0] start_bomb4_x = 40;
	reg [7:0] start_bomb5_x = 60;
	reg [7:0] start_bomb6_x = 130;
	reg [7:0] start_plyr_x = 60;
	//reg [7:0] start_plyr_x_S2 = 60;
	
	reg [7:0] start_bomb1_y = 20;
	reg [7:0] start_bomb2_y = 20;
	reg [7:0] start_bomb3_y = 20;
	reg [7:0] start_bomb4_y = 20;
	reg [7:0] start_bomb5_y = 20;
	reg [7:0] start_bomb6_y = 20;
	reg [7:0] start_plyr_y;
	//reg [7:0] start_plyr_y_S2;
	
	wire [7:0] out_bomb1_x;
	wire [7:0] out_bomb2_x;
	wire [7:0] out_bomb3_x;
	wire [7:0] out_bomb4_x;
	wire [7:0] out_bomb5_x;
	wire [7:0] out_bomb6_x;
	wire [7:0] out_bomb1_y;
	wire [7:0] out_bomb2_y;
	wire [7:0] out_bomb3_y;
	wire [7:0] out_bomb4_y;
	wire [7:0] out_bomb5_y;
	wire [7:0] out_bomb6_y;
	wire[7:0] out_plyr_x;
	wire[7:0] out_plyr_y;
	//wire[7:0] out_plyr_x_S2;
	//wire[7:0] out_plyr_y_S2;
	
	
	wire [8:0] colour_1;
	wire [8:0] colour_2;
	wire [8:0] colour_3;
	wire [8:0] colour_4;
	wire [8:0] colour_5;
	wire [8:0] colour_6;
	
	wire [2:0] bomb1_speed;
	wire [2:0] bomb2_speed;
	wire [2:0] bomb3_speed;
	wire [2:0] bomb4_speed;
	wire [2:0] bomb5_speed;
	wire [2:0] bomb6_speed;
	
	//reg go_S2 = 0;
	
	//for 30 seconds 
	wire done_30s;
	wire done_30s2;
	wire done_30s3;
	reg reset_30s;
	//reg en_30s;
	//wire enable30s = en_30s;
	
	assign colour = colour_1;
	assign bomb1_speed = 3'b001;
	assign bomb2_speed = 3'b011;
	assign bomb3_speed = 3'b001;
	assign bomb4_speed = 3'b111;
	assign bomb5_speed = 3'b101;
	assign bomb6_speed = 3'b110;
	
	wire [23:0]vga_in;
	
	//count30s(.enable(en_30s), .resetn(reset_30s), .CLOCK_50(CLOCK_50), .donesig(done_30s));
	count30s count1(CLOCK_50, SW[1], 1'b1, HEX0, HEX1, done_30s);
	count30s count2(CLOCK_50, done_30s, 1'b1, HEX2, HEX3, done_30s2);
	count30s count3(CLOCK_50, done_30s2, 1'b1, HEX4, HEX5, done_30s3);
	
	mux vga_selec(.x_plyr(x_plyr), .y_plyr(y_plyr), .colour_plyr(colour_plyr),
				  .x_bgS3(x_bgS3), .y_bgS3(y_bgS3), .colour_bgS3(colour_bgS3),	
				  .x_bgS4(x_bgS4), .y_bgS4(y_bgS4), .colour_bgS4(colour_bgS4),	
				  .x_black(x_black), .y_black(y_black), .colour_black(colour_black), 
				  .x_bomb1(x_bomb1),
				  .x_bomb2(x_bomb2),
				  .x_bomb3(x_bomb3),
				  .y_bomb1(y_bomb1),
				  .y_bomb2(y_bomb2),
				  .y_bomb3(y_bomb3),
				  .colour_bomb(colour),
				  .x_bgS2(x_bgS2), .y_bgS2(y_bgS2), .colour_bgS2(colour_bgS2),
				  .x_bomb4(x_bomb4), .x_bomb5(x_bomb5), .x_bomb6(x_bomb6),
				  .y_bomb4(y_bomb4), .y_bomb5(y_bomb5), .y_bomb6(y_bomb6),
				  .x_gameover(x_gameover), .y_gameover(y_gameover), .colour_gameover(colour_gameover),
				  .out(vga_in), 
				  .muxSelect(item_selector));				  
	
	animate_plyr ply0(.go(go_plyr),
					  .resetn(resetn),
					  .vga_x(x_plyr),
					  .vga_y(y_plyr),
					  .in_x(start_plyr_x),
					  .in_y(start_plyr_y),
					  .out_x(out_plyr_x),
					  .out_y(out_plyr_y),
					  .colour(colour_plyr),
					  .done(done_plyr),
					  .clock(CLOCK_50),
					  .left(left),
					  .right(right),
					  .up(up),
					  .down(down));
					  
	/*animate_plyr ply1(.go(go_plyr_S2),
					  .resetn(resetn_S2),
					  .vga_x(x_plyr_S2),
					  .vga_y(y_plyr_S2),
					  .in_x(start_plyr_x_S2),
					  .in_y(start_plyr_y_S2),
					  .out_x(out_plyr_x_S2),
					  .out_y(out_plyr_y_S2),
					  .colour(colour_plyr_S2),
					  .done(done_plyr_S2),
					  .clock(CLOCK_50),
					  .left(left),
					  .right(right),
					  .up(up),
					  .down(down)); */
					  
	draw_gameover gameover (
					.enable(go_gameover),
					.clock(CLOCK_50),
					.resetn(resetn),
					.vga_x(x_gameover),
					.vga_y(y_gameover),
					.colour(colour_gameover),
					.done(done_gameover)
					);
					 
	animate_bomb bomb01(.go(go_bomb1), .lose(lose_bomb1), .resetn(resetn), 
			   .vga_x(x_bomb1), .vga_y(y_bomb1), .in_x(start_bomb1_x), .in_y(start_bomb1_y), 
			   .out_y(out_bomb1_y), .colour(colour_1), 
			   .plyr_x(x_plyr), .plyr_y(y_plyr), .done(done_bomb1), .clock(CLOCK_50), .speed(bomb1_speed));	
			   
	animate_bomb bomb02(.go(go_bomb2), .lose(lose_bomb2), .resetn(resetn), 
			   .vga_x(x_bomb2), .vga_y(y_bomb2), .in_x(start_bomb2_x), .in_y(start_bomb2_y), 
			   .out_y(out_bomb2_y), .colour(colour_2), 
			   .plyr_x(x_plyr), .plyr_y(y_plyr), .done(done_bomb2), .clock(CLOCK_50), .speed(bomb2_speed));
			   
	animate_bomb bomb03(.go(go_bomb3), .lose(lose_bomb3), .resetn(resetn), 
			   .vga_x(x_bomb3), .vga_y(y_bomb3), .in_x(start_bomb3_x), .in_y(start_bomb3_y), 
			   .out_y(out_bomb3_y), .colour(colour_3), 
			   .plyr_x(x_plyr), .plyr_y(y_plyr), .done(done_bomb3), .clock(CLOCK_50), .speed(bomb3_speed));	
			   
	//stage 2 additional bombs
	
	animate_bomb bomb04(.go(go_bomb4), .lose(lose_bomb4), .resetn(resetn), 
			   .vga_x(x_bomb4), .vga_y(y_bomb4), .in_x(start_bomb4_x), .in_y(start_bomb4_y), 
			   .out_y(out_bomb4_y), .colour(colour_4), 
			   .plyr_x(x_plyr), .plyr_y(y_plyr), .done(done_bomb4), .clock(CLOCK_50), .speed(bomb4_speed));	
			   
	animate_bomb bomb05(.go(go_bomb5), .lose(lose_bomb5), .resetn(resetn), 
			   .vga_x(x_bomb5), .vga_y(y_bomb5), .in_x(start_bomb5_x), .in_y(start_bomb5_y), 
			   .out_y(out_bomb5_y), .colour(colour_5), 
			   .plyr_x(x_plyr), .plyr_y(y_plyr), .done(done_bomb5), .clock(CLOCK_50), .speed(bomb5_speed));	
			   
	animate_bomb bomb06(.go(go_bomb6), .lose(lose_bomb6), .resetn(resetn), 
			   .vga_x(x_bomb6), .vga_y(y_bomb6), .in_x(start_bomb6_x), .in_y(start_bomb6_y), 
			   .out_y(out_bomb6_y), .colour(colour_6), 
			   .plyr_x(x_plyr), .plyr_y(y_plyr), .done(done_bomb6), .clock(CLOCK_50), .speed(bomb6_speed));			   
	
	draw_black black(.enable(go_black),
					 .clock(CLOCK_50),
					 .resetn(resetn),
					 .vga_x(x_black),
					 .vga_y(y_black),
					 .colour(colour_black),
					 .done(done_black));  
		
	draw_black2 stage2bg(
						.enable(go_bgS2),
						.clock(CLOCK_50),
						.resetn(resetn_S2),
						.vga_x(x_bgS2),
						.vga_y(y_bgS2),
						.colour(colour_bgS2),
						.done(done_bgS2)
						);	

	draw_black3 stage3bg(
						.enable(go_bgS3),
						.clock(CLOCK_50),
						.resetn(resetn_S3),
						.vga_x(x_bgS3),
						.vga_y(y_bgS3),
						.colour(colour_bgS3),
						.done(done_bgS3)
						);

	draw_black4 stage4bg(
						.enable(go_bgS4),
						.clock(CLOCK_50),
						.resetn(resetn_S4),
						.vga_x(x_bgS4),
						.vga_y(y_bgS4),
						.colour(colour_bgS4),
						.done(done_bgS4)
						);						
	
		 
	//ANIMATION SPEED//
	reg [27:0] count_wait = 27'b0;
	always @(posedge clock)
	begin
		if (go_wait)
		begin
			count_wait = count_wait + 1'b1;
			done_wait = 0;

			if (count_wait == 4999999)
			begin
				count_wait = 27'b0;
				done_wait = 1;
			end
		end
	end
	
	
	
	//MAIN FSM//
	wire start;
	assign start = KEY[0];
	parameter [4:0] 
			  //stage 1
			  BEGIN = 5'b00000, PRINT_BOMB1 = 5'b00001, PRINT_BOMB2 = 5'b00010, PRINT_BOMB3 = 5'b00011, PRINT_PLYR = 5'b00100, 
			  PRINT_BLACK = 5'b00101,  EXIT = 5'b00110, WAIT = 5'b00111,
			  //stage 2
			  EXIT_S2 = 5'b01000, PRINT_BOMB4 = 5'b01001, PRINT_BOMB5= 5'b01010, PRINT_BOMB6 = 5'b01011, 
			  //stage 3
			  EXIT_S3 = 5'b01100, EXIT_S4 = 5'b01101;
			  
	reg [3:0]PresentState, NextState;
	parameter [6:0] HEIGHT_SCREEN = 7'b1111000;
	parameter [4:0] HEIGHT_PLYR = 5'b01111;
	
	
always @(*)
	begin: STATETABLE
		case(PresentState)
		//stage 1
		BEGIN: 
		begin 
			if(start == 1)
				NextState = BEGIN;
			else
				NextState = PRINT_BLACK;
		end
		PRINT_BOMB1:
		begin
			if(lose_bomb1)
			begin
				NextState = EXIT;
			end
			else
			begin
				if(done_bomb1)
					NextState = PRINT_BOMB2;
				else
					NextState = PRINT_BOMB1;
			end
		end
		PRINT_BOMB2:
		begin
			if(lose_bomb2)
			begin
				NextState = EXIT;
			end
			else
			begin
				if(done_bomb2)
					NextState = PRINT_BOMB3;
				else
					NextState = PRINT_BOMB2;
			end
		end
		PRINT_BOMB3:
		begin
			if(lose_bomb3)
			begin
				NextState = EXIT;
			end
			else
			begin
				if(done_bomb3 && (done_30s2 == 0) && (done_30s3 == 0))
					NextState = PRINT_PLYR;
				else if (done_bomb3 && (done_30s2 == 1) && (done_30s3 == 0))
					NextState = PRINT_BOMB4;
				else
					NextState = PRINT_BOMB3;
			end
		end
		PRINT_PLYR:
		begin
			if(done_plyr)
				NextState = WAIT;
			else
				NextState = PRINT_PLYR;
		end
		PRINT_BLACK:
		begin
			if(done_black)
				NextState = PRINT_BOMB1;
			else
				NextState = PRINT_BLACK;
		end		
		EXIT:
		begin
			if(start == 1)
				NextState = EXIT;
			else 
				NextState = BEGIN;
		end
		WAIT:
		begin
			if(done_wait && (done_30s == 0) && (done_30s2 == 0)  && (done_30s3 == 0))
				NextState = PRINT_BLACK;
			else if(done_wait && (done_30s == 1) && (done_30s2 == 0)  && (done_30s3 == 0))			
				NextState = EXIT_S2;
			else if(done_wait && (done_30s == 1) && (done_30s2 == 1)  && (done_30s3 == 0))
				NextState = EXIT_S3;
			else if (done_wait && (done_30s == 1)&& (done_30s2 == 1) && (done_30s3 == 1))
				NextState = EXIT_S4;
			else
				NextState = WAIT;
		end
		
		
		//STAGE 2
		EXIT_S2:
		begin
			if(done_bgS2) 
				NextState = PRINT_BOMB4;
			else
				NextState = EXIT_S2;
		end
		
		//STAGE 3 bg
		EXIT_S3:
		begin 
			if(done_bgS3)
				NextState = PRINT_BOMB1;
			else 
				NextState = EXIT_S3;
		end
		
		EXIT_S4:
		begin
			if(start == 1)
				NextState = EXIT_S4;
			else 
				NextState = BEGIN;
		end
		
		PRINT_BOMB4:
		begin
		if(lose_bomb4)
			begin
				NextState = EXIT;
			end
		else
			begin
				if(done_bomb4)
					NextState = PRINT_BOMB5;
				else
					NextState = PRINT_BOMB4;
			end
		end 
		
		PRINT_BOMB5:
		begin
		if(lose_bomb5)
			begin
				NextState = EXIT;
			end
		else
			begin
				if(done_bomb5)
					NextState = PRINT_BOMB6;
				else
					NextState = PRINT_BOMB5;
			end
		end 
		PRINT_BOMB6:
		begin
		if(lose_bomb6)
			begin
				NextState = EXIT;
			end
		else
			begin
				if(done_bomb6)
					NextState = PRINT_PLYR;
				else
					NextState = PRINT_BOMB6;
			end
		end
		
		default: NextState = BEGIN;
		endcase
	end
	
	always @(*)
	begin: ouputlogic
		case(PresentState)
			BEGIN:
			begin
				//reset_30s = 1;
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; 
				go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 0;
				//go_score = 0;
				exit = 0;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = HEIGHT_SCREEN - HEIGHT_PLYR;
				//start_plyr_y_S2 = HEIGHT_SCREEN - HEIGHT_PLYR;
				item_selector = 4'b0;
			end
			PRINT_BOMB1:
			begin
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 1;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				//go_score = 1;
				exit = 0;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b0;
			end
			PRINT_BOMB2:
			begin
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 1;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				//go_score = 1;
				exit = 0;
				start_bomb2_y = out_bomb2_y;
				start_bomb1_y = out_bomb1_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b0001;
			end
			PRINT_BOMB3:
			begin
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 1; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				//go_score = 1;
				exit = 0;
				start_bomb3_y = out_bomb3_y;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b0010;
			end
			PRINT_BOMB4:
			begin		
				//reset_30s = 0;
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 1; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				exit = 0;
				start_bomb4_y = out_bomb4_y;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;				
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b0111;
			end
			PRINT_BOMB5:
			begin				
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 1; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				exit = 0;
				start_bomb5_y = out_bomb5_y;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;				
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b1000;
			end
			PRINT_BOMB6:
			begin				
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 1;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				exit = 0;
				//en_30s = 1;
				//reset_30s = 1;
				start_bomb6_y = out_bomb6_y;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;				
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b1001;
			end
			PRINT_PLYR:
			begin
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 1; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				exit = 0;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				item_selector = 4'b0100;
			end
			PRINT_BLACK:
			begin
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 1; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 1;
				exit = 0;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b0011;
			end
			EXIT:
			begin
				//reset_30s = 0; //negedge reset
				//en_30s = 0;
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 1;
				exit = 1;
				writeEn = 1;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b1011;
			end
			WAIT:
			begin
				go_wait = 1;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2= 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				exit = 0;
				writeEn = 0;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				item_selector = 4'b0;
			end		
			
			EXIT_S2:
			begin
				//reset_30s = 1; //negedge reset
				//done_30s = 0;
				go_wait = 0;
				//go_S2 = 1;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2 = 0;  
				go_black = 0; go_bgS2 = 1; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				exit = 0;
				writeEn = 1;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				//start_plyr_x_S2 = out_plyr_x_S2;
				//start_plyr_y_S2 = out_plyr_y_S2;
				item_selector = 4'b1010;
			end 
			
			EXIT_S3:
			begin
				//reset_30s = 1; //negedge reset
				//done_30s = 0;
				go_wait = 0;
				//go_S2 = 1;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2 = 0;  
				go_black = 0; go_bgS2 = 0; go_bgS3 = 1; go_bgS4 = 0;
				go_gameover = 0;
				exit = 0;
				writeEn = 1;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				//start_plyr_x_S2 = out_plyr_x_S2;
				//start_plyr_y_S2 = out_plyr_y_S2;
				item_selector = 4'b0101;
			end 
			
			EXIT_S4:
			begin
				//reset_30s = 1; //negedge reset
				//done_30s = 0;
				go_wait = 0;
				//go_S2 = 1;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0; go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2 = 0;  
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 1;
				go_gameover = 0;
				exit = 0;
				writeEn = 1;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				//start_plyr_x_S2 = out_plyr_x_S2;
				//start_plyr_y_S2 = out_plyr_y_S2;
				item_selector = 4'b0110;
			end 
			
			default:
			begin
				//en_30s = 0;
				//reset_30s = 0;
				go_wait = 0;
				//go_wait_S2 = 0;
				go_bomb1 = 0;
				go_bomb2 = 0;
				go_bomb3 = 0;
				go_bomb4 = 0; go_bomb5 = 0; go_bomb6 = 0;
				go_plyr = 0; //go_plyr_S2 = 0;
				go_black = 0; go_bgS2 = 0; go_bgS3 = 0; go_bgS4 = 0;
				go_gameover = 0;
				writeEn = 0;
				exit = 0;
				start_bomb1_y = out_bomb1_y;
				start_bomb2_y = out_bomb2_y;
				start_bomb3_y = out_bomb3_y;
				start_bomb4_y = out_bomb4_y;
				start_bomb5_y = out_bomb5_y;
				start_bomb6_y = out_bomb6_y;
				start_plyr_x = out_plyr_x;
				start_plyr_y = out_plyr_y;
				//start_plyr_x_S2 = out_plyr_x_S2;
				//start_plyr_y_S2 = out_plyr_y_S2;
				item_selector = 4'b0000;
			end
		endcase
	end
	
	
	always @(posedge clock)
	begin: state_FFs
		if(resetn == 1'b0)
			PresentState <= BEGIN;
		else
			PresentState <= NextState;
	end
	
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(vga_in[8:0]),
			.x(vga_in[23:16]),
			.y(vga_in[15:9]),
			.plot(writeEn), 
			.VGA_R(VGA_R),
			.VGA_G(VGA_G),
			.VGA_B(VGA_B),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK_N),
			.VGA_SYNC(VGA_SYNC_N),
			.VGA_CLK(VGA_CLK));
		defparam VGA.RESOLUTION = "160x120";
		defparam VGA.MONOCHROME = "FALSE";
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 3;
		defparam VGA.BACKGROUND_IMAGE = "start.mif";
	
endmodule


module mux(x_plyr, y_plyr, colour_plyr, x_black, y_black, colour_black, x_bgS3, y_bgS3, colour_bgS3, //stage 1
		   x_bgS4, y_bgS4, colour_bgS4,
		   x_bomb1, x_bomb2, x_bomb3, y_bomb1, y_bomb2, y_bomb3, colour_bomb, //stage 1
		   x_bgS2, y_bgS2, colour_bgS2, //stage 2
		   x_bomb4, x_bomb5, x_bomb6, y_bomb4, y_bomb5, y_bomb6, //stage 2
		   x_gameover, y_gameover, colour_gameover,
		   out, muxSelect);
	input [7:0] x_plyr, x_bgS3, x_bgS4, x_black, x_bgS2, x_bomb1, x_bomb2, x_bomb3, x_bomb4, x_bomb5, x_bomb6, x_gameover;
	input [6:0] y_plyr, y_bgS3, y_bgS4, y_black, y_bgS2, y_bomb1, y_bomb2, y_bomb3, y_bomb4, y_bomb5, y_bomb6, y_gameover;
	input [8:0] colour_black, colour_bgS2, colour_plyr, colour_bgS3, colour_bgS4, colour_bomb, colour_gameover;
	input [3:0] muxSelect;
	
	output reg [23:0] out;
	
	always @(*)
	begin
		case(muxSelect[3:0])
			4'b0000: out = {x_bomb1, y_bomb1, colour_bomb};
			4'b0001: out = {x_bomb2, y_bomb2, colour_bomb};
			4'b0010: out = {x_bomb3, y_bomb3, colour_bomb};
			4'b0100: out = {x_plyr, y_plyr, colour_plyr};
			4'b0101: out = {x_bgS3, y_bgS3, colour_bgS3};
			4'b0110: out = {x_bgS4, y_bgS4, colour_bgS4};
			4'b0011: out = {x_black, y_black, colour_black};
			4'b0111: out = {x_bomb4, y_bomb4, colour_bomb};
			4'b1000: out = {x_bomb5, y_bomb5, colour_bomb};
			4'b1001: out = {x_bomb6, y_bomb6, colour_bomb};
			4'b1010: out = {x_bgS2, y_bgS2, colour_bgS2};
			4'b1011: out = {x_gameover, y_gameover, colour_gameover};
			default: out = 0;
		endcase
	end
	
	
	
endmodule


module animate_plyr (go, resetn, vga_x, vga_y, in_x, in_y, out_x, out_y, colour, done, clock, left, right, up, down);	
	input go;
	input resetn;
	input clock;
	input left;
	input right;
	input up;
	input down;
	reg writeEn;
	reg go_increment;
	reg go_decrement;
	reg go_increment_init;
	reg go_up;
	reg go_down;

	parameter [2:0] WAIT = 3'b000, SHIFT = 3'b010, PRINT = 3'b110;
	parameter [6:0] HEIGHT_SCREEN = 7'b1111000;
	parameter [3:0] HEIGHT_PLYR = 4'b1111, WIDTH_PLYR = 4'b1111;
		
	wire [7:0] w_out_x;
	wire [6:0] w_out_y;
	//assign w_out_y = HEIGHT_SCREEN - HEIGHT_PLYR;
	wire done_print; //done signal to know when finished printing
	
	input [7:0] in_x; //original start x
	input [6:0] in_y; //original start y 
	reg [7:0] w_in_x;
	reg [6:0] w_in_y;
	
	wire [7:0] w_vga_x; 
	wire [6:0] w_vga_y;
	
	output [7:0] vga_x; //all pixels to be printed x
	output [6:0] vga_y; //all pixels to be printed y
	output [7:0] out_x; //new shifted start x
	output [6:0] out_y; //new shifted out y 
	output [8:0] colour;
	output reg done = 0;
	
	reg [2:0] PresentState, NextState;
	reg [3:0] count;
	always @(*)
	begin : StateTable
		case (PresentState)
		WAIT:
		begin
			done = 0;
			if (go == 0)
				NextState = WAIT;
			else
			begin
				NextState = SHIFT;
			end
		end
		SHIFT:
		begin
			NextState = PRINT;
			done = 0;
		end
		PRINT:
		begin
			if (done_print == 1)
			begin
				NextState = WAIT;
				done = 1;
			end
			else
			begin
				NextState = PRINT;
				done = 0;
			end
		end
		default: 
		begin 
			NextState = WAIT;
			done = 0;
		end
		endcase
	end
	
	always @(posedge clock)
	begin
		if (go_increment_init)
		begin
			w_in_x = in_x;
			w_in_y = in_y;
		end 
		else if (go_increment)
			w_in_x = w_in_x + 3'b100;
		else if (go_decrement)
			w_in_x = w_in_x - 3'b100;
		else if (go_up)
			w_in_y = w_in_y - 3'b100;
		else if (go_down)
			w_in_y = w_in_y + 3'b100;
	end
	
	always @(*)
	begin: output_logic
		case (PresentState)
			WAIT:
				begin
					go_increment_init = 1;
					go_increment = 0;
					go_decrement = 0;
					go_up = 0;
					go_down = 0;
					writeEn = 0;
				end
			SHIFT:
				begin
					go_increment_init = 0;
					if (left == 0)
					begin
						go_decrement = 1;
						go_increment = 0;
						go_up = 0;
						go_down = 0;
					end
					else if (right == 0)
					begin
						go_increment = 1;
						go_decrement = 0;
						go_up = 0;
						go_down = 0;
					end
					else if (up == 0)
					begin 
						go_up = 1;
						go_increment = 0;
						go_decrement = 0;
						go_down = 0;
					end
					else if (down == 0)
					begin
						go_down = 1;
						go_up = 0;
						go_increment = 0;
						go_decrement = 0;
					end	
					else
					begin
						go_increment = 0;
						go_decrement = 0;
						go_up = 0;
						go_down = 0;
					end
					writeEn = 0;
				end
			PRINT:
				begin
					writeEn = 1;
					go_increment_init = 0;
					go_increment = 0;
					go_decrement = 0;
					go_up = 0;
					go_down = 0;
				end
			default:
				begin
					writeEn = 0;
					go_increment_init = 0;
					go_increment = 0;
					go_decrement = 0;
					go_up = 0;
					go_down = 1;
				end
		endcase
	end
	
	always @(posedge clock)
	begin: state_FFs
		if(resetn == 1'b0)
			PresentState <= WAIT;
		else
			PresentState <= NextState;
	end
	

	assign out_x = w_in_x;
	assign w_out_x = w_in_x;
	assign out_y = w_in_y;
	assign w_out_y = w_in_y;
	
	assign vga_x = w_vga_x;
	assign vga_y = w_vga_y;
	
	draw_plyr plyr1 (
					.reset(resetn),
					.writeEn(writeEn),
					.x(w_vga_x),
					.y(w_vga_y),
					.startx(w_out_x),
					.starty(w_out_y),
					.clock(clock),
					.color(colour),
					.done_print(done_print) 
					);
endmodule

module draw_plyr (x, y, startx, starty, color, clock, writeEn, reset, done_print);
	input clock;
	input writeEn;
	input reset;
	input [7:0] startx;
	input [6:0] starty;
	output [7:0] x;
	output [6:0] y;
	output reg done_print;
	output [8:0] color;
	
	wire [8:0] address;
	reg [7:0] count_x = 0;
	reg [6:0] count_y = 0;
	
	
	jacksmall_ROM jack(
					.address(address),
					.clock(clock),
					.q(color)
					);
	
	reg [4:0] addr_x = 0;
	reg [4:0] addr_y = 0;
	always @(posedge clock)
	begin
		if (~reset)
		begin
			addr_x = 0;
			addr_y = 0;
			count_x = 0;
			count_y = 0;
		end
		else if (writeEn)
		begin
			done_print = 0;
			if (addr_x != 15)
			begin
				count_x = count_x + 1'b1;
				addr_x = addr_x + 1'b1;
			end
			else
			begin
				addr_y = addr_y + 1'b1;
				count_y = count_y + 1'b1;
				addr_x = 0;
				count_x = 0;
				if (addr_y == 15)
				begin
					count_y = 0;
					addr_y = 0;
					done_print = 1;
				end
			end
		end
	end

	
	assign address = addr_x + 15*(addr_y);
	assign x = count_x + startx;
	assign y = count_y + starty;
	
	
endmodule


module draw_black (enable, clock, resetn, vga_x, vga_y, colour, done);
	input clock;
	input enable;
	input resetn;
	output [7:0] vga_x;
	output [6:0] vga_y;
	output reg done = 0;
	output [8:0] colour;
	
	
	wire [14:0] address;
	reg [7:0] count_x = 0;
	reg [6:0] count_y = 0;
	
	
	jackbackground_ROM black (
					.address(address),
					.clock(clock),
					.q(colour)
					);
	
	reg [7:0] addr_x = 0;
	reg [6:0] addr_y = 0;
	always @(posedge clock)
	begin
		if (~resetn)
		begin
			addr_x = 0;
			addr_y = 0;
			count_x = 0;
			count_y = 0;
		end
		else if (enable)
		begin
			done = 0;
			if (addr_x != 160)
			begin
				count_x = count_x + 1'b1;
				addr_x = addr_x + 1'b1;
			end
			else
			begin
				addr_y = addr_y + 1'b1;
				count_y = count_y + 1'b1;
				addr_x = 0;
				count_x = 0;
				if (addr_y == 120)
				begin
					count_y = 0;
					addr_y = 0;
					done = 1;
				end
			end
		end
	end

	
	assign address = addr_x + 160*(addr_y);
	
	assign vga_x = count_x;
	assign vga_y = count_y;
	
endmodule
	
module draw_gameover(enable, clock, resetn, vga_x, vga_y, colour, done);
	input clock;
	input enable;
	input resetn;
	output [7:0] vga_x;
	output [6:0] vga_y;
	output reg done = 0;
	output [8:0] colour;
	
	wire [14:0] address;
	reg [7:0] count_x = 0;
	reg[6:0] count_y = 0;
	
	game_over_ROM gameover (.address(address),
						   .clock(clock),
						   .q(colour));
	
	reg [7:0] addr_x = 0;
	reg [6:0] addr_y = 0;
	always @(posedge clock)
	begin
		if(~resetn)
		begin
			addr_x = 0;
			addr_y = 0;
			count_x = 0;
			count_y = 0;
		end
		else if (enable) begin
			done = 0;
			if(addr_x != 160)
			begin	
				count_x = count_x + 1'b1;
				addr_x = addr_x + 1'b1;
			end
			else
			begin
				addr_y = addr_y + 1'b1;
				count_y = count_y + 1'b1;
				addr_x = 0;
				count_x = 0;
				if(addr_y == 120)
				begin
					count_y = 0;
					addr_y = 0;
					done = 1;
				end
			end
		end
	end
	
	assign address = addr_x + 160*(addr_y);
	
	assign vga_x = count_x;
	assign vga_y = count_y;
	
endmodule

module count30s(input CLOCK_50, resetn, enable, output [6:0]HEX0, HEX1, output reg done);
    wire W1; 
    wire [25:0] W2; 
    wire [4:0] Qout;
            //reg done;
            wire [3:0]hex1out;
            //reg done;
    EnableSignal ES1(.S(2'b01), .Clk(CLOCK_50), .Clear_b(resetn), .Q(W2)); 
    assign W1 = (W2 == 0)?1:0;
    Counter C1(.Enable(W1 && enable), .Clk(CLOCK_50), .Clear_b(resetn) , .Q(Qout)); 
            always @(*) begin
                        if (Qout == 5'b11111)
                                   done = 1;
                        else done = 0;
            end
            assign hex1out = {3'b000,Qout[4]};
            
			/*always @ (*) begin
				if(done == 1)
					count = Qout;
			end	*/
    HEX H0(.B(Qout[3:0]), .S(HEX0));  
             HEX H1(.B(hex1out), .S(HEX1));  
 
endmodule // HexCounter
 
module HEX(input [3:0]B, output [6:0]S);
    wire [6:0]W; wire [3:0]D;
    assign D = B;
    assign W[0] = !((!D[0]|D[1]|D[2]|D[3]) & (D[0]|D[1]|!D[2]|D[3]) & (!D[0]|!D[1]|D[2]|!D[3]) & (!D[0]|D[1]|!D[2]|!D[3]));
    assign W[1] = !((!D[0]|D[1]|!D[2]|D[3]) & (D[0]|!D[1]|!D[2]|D[3]) & (!D[0]|!D[1]|D[2]|!D[3]) & (D[0]|D[1]|!D[2]|!D[3]) & (D[0]|!D[1]|!D[2]|!D[3]) & (!D[0]|!D[1]|!D[2]|!D[3]));
    assign W[2] = !((D[0]|!D[1]|D[2]|D[3]) & (D[0]|D[1]|!D[2]|!D[3]) & (D[0]|!D[1]|!D[2]|!D[3]) & (!D[0]|!D[1]|!D[2]|!D[3]));
    assign W[3] = !((!D[0]|D[1]|D[2]|D[3]) & (D[0]|D[1]|!D[2]|D[3]) & (!D[0]|!D[1]|!D[2]|D[3]) & (D[0]|!D[1]|D[2]|!D[3]) & (!D[0]|!D[1]|!D[2]|!D[3]));
    assign W[4] = !((!D[0]|D[1]|D[2]|D[3]) & (!D[0]|!D[1]|D[2]|D[3]) & (D[0]|D[1]|!D[2]|D[3]) & (!D[0]|D[1]|!D[2]|D[3]) & (!D[0]|!D[1]|!D[2]|D[3]) & (!D[0]|D[1]|D[2]|!D[3]));
    assign W[5] = !((!D[0]|D[1]|D[2]|D[3]) & (D[0]|!D[1]|D[2]|D[3]) & (!D[0]|!D[1]|D[2]|D[3]) & (!D[0]|!D[1]|!D[2]|D[3])&(!D[0]|D[1]|!D[2]|!D[3]));
    assign W[6] = !((D[0]|D[1]|D[2]|D[3]) & (!D[0]|D[1]|D[2]|D[3]) & (!D[0]|!D[1]|!D[2]|D[3]) & (D[0]|D[1]|!D[2]|!D[3]));
    assign S = W;
endmodule // hexDecoder
 
module EnableSignal(input Clear_b, Clk, input [1:0]S, output reg [25:0]Q); 
    always @(posedge Clk) 
    begin
        if(Clear_b == 1'b0)
            Q <= 26'b0000000000000000000000000;  
        else if(Q == 0)
            if(S == 2'b01)
                Q <= 26'b00101111101011110000011111; 
            else
                Q <= 26'b0000000000000000000000000; 
        else
            Q <= Q - 1;  
    end
endmodule // EnableSignal
 
module Counter(input Enable, Clk, Clear_b, output reg [4:0]Q); 
    always @(posedge Clk) 
    begin
        if(Clear_b == 1'b0)
            Q <= 5'b00000;
        else if(Q == 5'b11111)
            Q <= 5'b11111; 
        else if(Enable == 1'b1) 
            Q <= Q + 1;      
    end
endmodule // Counter


//STAGE 2 BACKGROUND
module draw_black2 (enable, clock, resetn, vga_x, vga_y, colour, done);
input clock;
	input enable;
	input resetn;
	output [7:0] vga_x;
	output [6:0] vga_y;
	output reg done = 0;
	output [8:0] colour;	
	
	wire [14:0] address;
	reg [7:0] count_x = 0;
	reg [6:0] count_y = 0;
	
	
	jackbackgroundlvl2_ROM black2 (
					.address(address),
					.clock(clock),
					.q(colour)
					);
	
	reg [7:0] addr_x = 0;
	reg [6:0] addr_y = 0;
	always @(posedge clock)
	begin
		if (~resetn)
		begin
			addr_x = 0;
			addr_y = 0;
			count_x = 0;
			count_y = 0;
		end
		else if (enable)
		begin
			done = 0;
			if (addr_x != 160)
			begin
				count_x = count_x + 1'b1;
				addr_x = addr_x + 1'b1;
			end
			else
			begin
				addr_y = addr_y + 1'b1;
				count_y = count_y + 1'b1;
				addr_x = 0;
				count_x = 0;
				if (addr_y == 120)
				begin
					count_y = 0;
					addr_y = 0;
					done = 1;
				end
			end
		end
	end

	
	assign address = addr_x + 160*(addr_y);
	
	assign vga_x = count_x;
	assign vga_y = count_y;
endmodule


//STAGE 3 BACKGROUND
module draw_black3 (enable, clock, resetn, vga_x, vga_y, colour, done);
input clock;
	input enable;
	input resetn;
	output [7:0] vga_x;
	output [6:0] vga_y;
	output reg done = 0;
	output [8:0] colour;	
	
	wire [14:0] address;
	reg [7:0] count_x = 0;
	reg [6:0] count_y = 0;
	
	
	jackbackgroundlvl3_ROM black3 (
					.address(address),
					.clock(clock),
					.q(colour)
					);
	
	reg [7:0] addr_x = 0;
	reg [6:0] addr_y = 0;
	always @(posedge clock)
	begin
		if (~resetn)
		begin
			addr_x = 0;
			addr_y = 0;
			count_x = 0;
			count_y = 0;
		end
		else if (enable)
		begin
			done = 0;
			if (addr_x != 160)
			begin
				count_x = count_x + 1'b1;
				addr_x = addr_x + 1'b1;
			end
			else
			begin
				addr_y = addr_y + 1'b1;
				count_y = count_y + 1'b1;
				addr_x = 0;
				count_x = 0;
				if (addr_y == 120)
				begin
					count_y = 0;
					addr_y = 0;
					done = 1;
				end
			end
		end
	end

	
	assign address = addr_x + 160*(addr_y);
	
	assign vga_x = count_x;
	assign vga_y = count_y;
endmodule

module draw_black4 (enable, clock, resetn, vga_x, vga_y, colour, done);
input clock;
	input enable;
	input resetn;
	output [7:0] vga_x;
	output [6:0] vga_y;
	output reg done = 0;
	output [8:0] colour;	
	
	wire [14:0] address;
	reg [7:0] count_x = 0;
	reg [6:0] count_y = 0;
	
	
	win_ROM win (
					.address(address),
					.clock(clock),
					.q(colour)
					);
	
	reg [7:0] addr_x = 0;
	reg [6:0] addr_y = 0;
	always @(posedge clock)
	begin
		if (~resetn)
		begin
			addr_x = 0;
			addr_y = 0;
			count_x = 0;
			count_y = 0;
		end
		else if (enable)
		begin
			done = 0;
			if (addr_x != 160)
			begin
				count_x = count_x + 1'b1;
				addr_x = addr_x + 1'b1;
			end
			else
			begin
				addr_y = addr_y + 1'b1;
				count_y = count_y + 1'b1;
				addr_x = 0;
				count_x = 0;
				if (addr_y == 120)
				begin
					count_y = 0;
					addr_y = 0;
					done = 1;
				end
			end
		end
	end

	
	assign address = addr_x + 160*(addr_y);
	
	assign vga_x = count_x;
	assign vga_y = count_y;
endmodule




module animate_bomb (go, lose, resetn, vga_x, vga_y, in_x, in_y, out_y, colour, plyr_x, plyr_y, done, clock, speed);
           input go;
           input resetn;
           input clock;
           input [7:0] plyr_x;
           input [6:0] plyr_y;
           input [2:0] speed;
           output reg lose;
           reg writeEn;
           reg go_increment;
           reg go_increment_init;
           
           parameter [2:0] WAIT = 3'b000, SHIFT = 3'b010, CHECK = 3'b100, PRINT = 3'b110;
           parameter [6:0] HEIGHT_SCREEN = 7'b1111000;
           parameter [3:0] BOMB_WIDTH = 4'b1010;
           parameter [4:0] HEIGHT_PLYR = 5'b01111, WIDTH_PLYR = 5'b01111;
                      
           wire [6:0] w_out_y;
           wire done_print; //done signal to know when finished printing
           
           input [7:0] in_x; //original start x
           wire [7:0] w_in_x;
           assign w_in_x = in_x;
           input [6:0] in_y; //original start y
           reg [6:0] w_in_y;
           
           wire [7:0] w_vga_x; 
           wire [6:0] w_vga_y;
           
           output [7:0] vga_x; //all pixels to be printed x
           output [6:0] vga_y; //all pixels to be printed y
           output [6:0] out_y; //new shifted start y
           output [8:0] colour;
           output reg done = 0;
           
           reg [2:0] PresentState, NextState;
           reg [3:0] count;
           always @(*)
           begin : StateTable
                      case (PresentState)
                      WAIT:
                      begin
                                 done = 0;
                                 if (go == 0)
                                 begin
                                            NextState = WAIT;
                                            lose = 0;
                                 end
                                 else
                                 begin
                                            NextState = SHIFT;
                                            lose = 0;
                                 end
                      end
                      SHIFT:
                      begin
                                 NextState = CHECK;
                                 done = 0;
                                 lose = 0;
                      end
                      CHECK:
                      begin
                                 if (((w_out_y + BOMB_WIDTH >= (plyr_y)) && (w_out_y <= plyr_y + HEIGHT_PLYR)) && (((w_in_x >= plyr_x)&&(w_in_x <= (plyr_x + WIDTH_PLYR))) || ((w_in_x + BOMB_WIDTH >= plyr_x)&& (w_in_x + BOMB_WIDTH <= (plyr_x + WIDTH_PLYR))))) //updated
                                 begin
                                            NextState = WAIT;
                                            lose = 1;
                                            done = 0;
                                 end
                                 else
                                 begin
                                            NextState = PRINT;
                                            done = 0;
                                            lose = 0;
                                 end
                      end
                      PRINT:
                      begin
                                 if (done_print == 1)
                                 begin
                                            NextState = WAIT;
                                            done = 1;
                                            lose = 0;
                                 end
                                 else
                                 begin
                                            NextState = PRINT;
                                            done = 0;
                                            lose = 0;
                                 end
                      end
                      default: 
                      begin 
                                 NextState = WAIT;
                                 done = 0;
                                 lose = 0;
                      end
                      endcase
           end
           
           always @(posedge clock)
           begin
                      if (go_increment_init)
                      begin
                                 w_in_y = in_y;
                      end
                      if (go_increment)
                      begin
                                 w_in_y = w_in_y + speed;
                      end
           end
           
           always @(*)
           begin: output_logic
                      case (PresentState)
                                 WAIT:
                                            begin
                                                       go_increment_init = 1;
                                                       go_increment = 0;
                                                       writeEn = 0;
                                            end
                                 SHIFT:
                                            begin
                                                       go_increment_init = 0;
                                                       go_increment = 1;
                                                       writeEn = 0;
                                            end
                                 CHECK:
                                            begin
                                                       go_increment_init = 0;
                                                       go_increment = 0;
                                                       writeEn = 0;
                                            end
                                 PRINT:
                                            begin
                                                       go_increment_init = 0;
                                                       go_increment = 0;
                                                       writeEn = 1;
                                            end
                                 default:
                                            begin
                                                       go_increment_init = 0;
                                                       go_increment = 0;
                                                       writeEn = 0;
                                            end
                      endcase
           end
           
           always @(posedge clock)
           begin: state_FFs
                      if(resetn == 1'b0)
                                 PresentState <= WAIT;
                      else
                                 PresentState <= NextState;
           end
           
           assign out_y = w_in_y;
           assign w_out_y = w_in_y;
           
           assign vga_x = w_vga_x;
           assign vga_y = w_vga_y;
           
           draw_bomb bomb1 (
														   .reset(resetn),
														   .writeEn(writeEn),
														   .x(w_vga_x),
														   .y(w_vga_y),
														   .startx(w_in_x),
														   .starty(w_out_y),
														   .clock(clock),
														   .color(colour),
														   .done_print(done_print) 
														   );
                                                       
endmodule
module draw_bomb (x, y, startx, starty, color, clock, writeEn, reset, done_print);
 
           input clock;
           input writeEn;
           input reset;
           input [7:0] startx;
           input [6:0] starty;
           output [7:0] x;
           output [6:0] y;
           output reg done_print;
           output [8:0] color;
           
           wire [8:0] address;
           reg [7:0] count_x = 0;
           reg [6:0] count_y = 0;
           
           
           bombmemory_ROM bomb(
                                                       .address(address),
                                                       .clock(clock),
                                                       .q(color)
                                                       );        
           
           reg [4:0] addr_x = 0;
           reg [4:0] addr_y = 0;
           always @(posedge clock)
           begin
                      if (~reset)
                      begin
                                 addr_x = 0;
                                 addr_y = 0;
                                 count_x = 0;
                                 count_y = 0;
                      end
                      else if (writeEn)
                      begin
                                 done_print = 0;
                                 if (addr_x != 10)
                                 begin
                                            count_x = count_x + 1'b1;
                                            addr_x = addr_x + 1'b1;
                                 end
                                 else
                                 begin
                                            addr_y = addr_y + 1'b1;
                                            count_y = count_y + 1'b1;
                                            addr_x = 0;
                                            count_x = 0;
                                            if (addr_y == 10)
                                            begin
                                                       count_y = 0;
                                                       addr_y = 0;
                                                       done_print = 1;
                                            end
                                 end
                      end
           end
 
           
           assign address = addr_x + 10*(addr_y);
           assign x = count_x + startx;
           assign y = count_y + starty;
           
endmodule

