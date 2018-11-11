module blockmove
	(
		CLOCK_50,						//	On Board 50 MHz
		// Your inputs and outputs here
		// The ports below are for the VGA output.  Do not change.
		VGA_CLK,   						//	VGA Clock
		VGA_HS,							//	VGA H_SYNC
		VGA_VS,							//	VGA V_SYNC
		VGA_BLANK_N,						//	VGA BLANK
		VGA_SYNC_N,						//	VGA SYNC
		VGA_R,   						//	VGA Red[9:0]
		VGA_G,	 						//	VGA Green[9:0]
		VGA_B,   						//	VGA Blue[9:0]
		KEY,
		SW
	);

	input			CLOCK_50;				//	50 MHz
	
	// Declare your inputs and outputs here
	// Do not change the following outputs
	output			VGA_CLK;   				//	VGA Clock
	output			VGA_HS;					//	VGA H_SYNC
	output			VGA_VS;					//	VGA V_SYNC
	output			VGA_BLANK_N;				//	VGA BLANK
	output			VGA_SYNC_N;				//	VGA SYNC
	output	[9:0]	VGA_R;   				//	VGA Red[9:0]
	output	[9:0]	VGA_G;	 				//	VGA Green[9:0]
	output	[9:0]	VGA_B;   				//	VGA Blue[9:0]
	
	// Create the colour, x, y and writeEn wires that are inputs to the controller.
	
	// Own inputs
	input [1:0] KEY;
	input [9:7] SW;

	// To VGA
	wire [2:0] colour;
	wire [23:0] realcolour;
	assign realcolour = {colour[2],colour[2],colour[2],colour[2],colour[2],colour[2],colour[2],colour[2],colour[1],colour[1],colour[1],colour[1],colour[1],colour[1],colour[1],colour[1],colour[0],colour[0],colour[0],colour[0],colour[0],colour[0],colour[0],colour[0]};
	wire [7:0] x;
	wire [6:0] y;
	wire writeEn;
	wire [14:0] xy;
	
	wire resetn;
	
	assign resetn = KEY[0];
	assign x = xy[14:7];
	assign y = xy[6:0];

	// Create an Instance of a VGA controller - there can be only one!
	// Define the number of colours as well as the initial background
	// image file (.MIF) for the controller.
	vga_adapter VGA(
			.resetn(resetn),
			.clock(CLOCK_50),
			.colour(realcolour),
			.x(x),
			.y(y),
			.plot(writeEn),
			/* Signals for the DAC to drive the monitor. */
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
		defparam VGA.BITS_PER_COLOUR_CHANNEL = 8;
		defparam VGA.BACKGROUND_IMAGE = "background.mif";
			
	// Put your code here. Your code should produce signals x,y,colour and writeEn
	// for the VGA controller, in addition to any other functionality your design may require.
	
	handler u0(
		.Go(KEY[1]),
		.CLOCK_50(CLOCK_50),
		.resetn(resetn),
		.inputSW(SW[9:7]),
		.xy(xy),
		.colour(colour),
		.writeEn(writeEn)
		);
	
endmodule

module handler(CLOCK_50, resetn, inputSW, xy, colour, writeEn, Go);

	input CLOCK_50, resetn;
	input Go;
	input [2:0] inputSW;
	output [14:0] xy;
	output [2:0] colour;
	output writeEn;
	
	wire En_x_counter, En_y_counter, ld_colour, ld_direction;
	wire sel_colour, square_counter_enable, square_done;
	wire CLOCK_60HZ, UpdateFrame;
	
	control u1(
		.Go(Go),
		.CLOCK_50(CLOCK_50), 
		.resetn(resetn),
		.En_x_counter(En_x_counter), 
		.En_y_counter(En_y_counter), 
		.ld_colour(ld_colour),
		.ld_direction(ld_direction),
		.sel_colour(sel_colour),
		.writeEn(writeEn),
		.square_counter_enable(square_counter_enable),
		.square_done(square_done),
		.UpdateFrame(UpdateFrame)
		);
	
	datapath u2(
		.CLOCK_50(CLOCK_50), 
		.resetn(resetn),
		.En_x_counter(En_x_counter), 
		.En_y_counter(En_y_counter),
		.ld_colour(ld_colour),
		.ld_direction(ld_direction),
		.sel_colour(sel_colour),
		.inputSW(inputSW), 
		.xy(xy),
		.colour(colour),
		.square_counter_enable(square_counter_enable),
		.square_done(square_done)
		);
		
		RateDivider_4HZ u5(
			.CLOCK_50(CLOCK_50), 
			.resetn(resetn), 
			.CLOCK_4HZ(UpdateFrame)
			);
		
//		RateDivider_60HZ u3(
//			.CLOCK_50(CLOCK_50), 
//			.resetn(resetn), 
//			.CLOCK_60HZ(CLOCK_60HZ)
//			);
//		
//		FrameCounter u4(
//			.CLOCK_50(CLOCK_50), 
//			.resetn(resetn), 
//			.CLOCK_60HZ(CLOCK_60HZ), 
//			.UpdateFrame(UpdateFrame)
//			);
	
endmodule

module RateDivider_4HZ(CLOCK_50, resetn, CLOCK_4HZ);

	input CLOCK_50, resetn;
	output CLOCK_4HZ;
	
	reg CLOCK_4HZ;
	reg [23:0] counter;

	always @(posedge CLOCK_50, negedge resetn) begin
		if (!resetn) begin
			counter <= 24'd0;
			CLOCK_4HZ <= 1'b0;
		end
		else begin
			counter <= (counter == 24'd12499999) ? 0 : counter + 1'b1;
			CLOCK_4HZ <= (counter == 24'd12499999);
		end
	end

endmodule

//module RateDivider_60HZ(CLOCK_50, resetn, CLOCK_60HZ);
//
//	input CLOCK_50, resetn;
//	output CLOCK_60HZ;
//	
//	reg CLOCK_60HZ;
//	reg [21:0] counter;
//
//	always @(posedge CLOCK_50, negedge resetn) begin
//		if (!resetn) begin
//			counter <= 22'd0;
//			CLOCK_60HZ <= 1'b0;
//		end
//		else begin
//			counter <= (counter == 22'd2499) ? 0 : counter + 1'b1;
//			CLOCK_60HZ <= (counter == 22'd2499) | (counter == 22'd833) | (counter == 22'd1666);
//		end
//	end
//
//endmodule
//
//module FrameCounter(CLOCK_50, resetn, CLOCK_60HZ, UpdateFrame);
//
//	input CLOCK_50, resetn, CLOCK_60HZ;
//	output UpdateFrame;
//	
//	reg UpdateFrame;
//	reg [3:0] counter;
//	
////	always @(posedge CLOCK_50) begin
////		if (!resetn) begin
////			counter <= 4'b0000;
////			UpdateFrame <= 1'b0;
////		end
////		else if (CLOCK_60HZ) begin
////			counter <= (counter == 4'd15) ? 0 : counter + 1'b1;
////			UpdateFrame <= (counter == 4'd15);
////		end
////	end
//	
//	always @(posedge CLOCK_60HZ, negedge resetn) begin
//		if (!resetn) begin
//			counter <= 4'b0000;
//			UpdateFrame <= 1'b0;
//		end
//		else if (counter == 4'b1110) begin
//			counter <= 4'b0000;
//			UpdateFrame <= 1'b1;
//		end
//		else begin
//			counter <= counter + 1'b1;
//			UpdateFrame <= 1'b0;
//		end
//	end
//
//endmodule

module control(CLOCK_50, resetn, Go, En_x_counter, En_y_counter, ld_colour, ld_direction, sel_colour, writeEn, square_counter_enable, square_done, UpdateFrame);

	input CLOCK_50, resetn, square_done, UpdateFrame, Go;
	output En_x_counter, En_y_counter, ld_colour, ld_direction, sel_colour, writeEn, square_counter_enable;
	
	reg En_x_counter, En_y_counter, ld_colour, ld_direction, sel_colour, writeEn, square_counter_enable;
	
	reg [2:0] current_state, next_state; 
	
	localparam	INIT						= 3'b000,
					DRAW						= 3'b001,
					DRAW_WAIT				= 3'b010,
					ERASE						= 3'b011,
					UPDATE_COUNTER			= 3'b100,
					UPDATE_DIRECTION		= 3'b101;

// Next state logic aka our state table
	always@(*)
	begin:state_table 
		case (current_state)
			INIT: next_state = DRAW;
			DRAW: next_state = ~square_done ? DRAW : DRAW_WAIT;
			DRAW_WAIT: next_state = (UpdateFrame && ~Go) ? ERASE : DRAW_WAIT;
			ERASE: next_state = ~square_done ? ERASE : UPDATE_COUNTER;
			UPDATE_COUNTER: next_state = UPDATE_DIRECTION;
			UPDATE_DIRECTION: next_state = DRAW;
			default:next_state = INIT;
		endcase
	end // state_table
	
	always @(*)
	begin: enable_signals
		En_x_counter = 1'b0;
		En_y_counter = 1'b0;
		ld_direction = 1'b0;
		ld_colour = 1'b1;
		square_counter_enable = 1'b0;
		writeEn = 1'b0;
		sel_colour = 1'b0;
		case (current_state)
			INIT: begin
			end
			DRAW: begin
				writeEn = 1'b1;
				square_counter_enable = 1'b1;
			end
			ERASE: begin
				writeEn = 1'b1;
				square_counter_enable = 1'b1;
				sel_colour = 1'b1;
			end
			UPDATE_COUNTER: begin
				En_x_counter = 1'b1;
				En_y_counter = 1'b1;
			end
			UPDATE_DIRECTION: begin
				ld_direction = 1'b1;
			end
		endcase
	end // enable_signals
	
	always@(posedge CLOCK_50)
		begin: state_FFs
		if (!resetn)
			current_state <= INIT;
		else
			current_state <= next_state;
	end // state_FFS

endmodule

module datapath(CLOCK_50, resetn, En_x_counter, En_y_counter, ld_colour, ld_direction, sel_colour, inputSW, xy, colour, square_counter_enable, square_done);

	input CLOCK_50, resetn, En_x_counter, En_y_counter, ld_colour, ld_direction, sel_colour, square_counter_enable;
	input [2:0] inputSW;
	output [14:0] xy;
	output [2:0] colour;
	output square_done;
	
	reg [7:0] x_reg;
	reg [6:0] y_reg;
	reg [2:0] colour, colour_reg;
	reg [14:0] xy, temp_xy;
	
	reg [4:0] squareCounter;
	reg square_done, Vertical_1_UP, Horizontal_1_LEFT;
	
	always@(posedge CLOCK_50) begin
		if (!resetn) begin
			x_reg <= 8'b0;
			y_reg <= 7'b1000000;
			colour_reg <= 3'b0;
			Vertical_1_UP <= 1'b0;
			Horizontal_1_LEFT <= 1'b0;
		end
		else begin
			if (En_x_counter) begin
				if (Horizontal_1_LEFT)
					x_reg <= x_reg - 1'b1;
				else
					x_reg <= x_reg + 1'b1;
			end
			if (En_y_counter) begin
				if (Vertical_1_UP)
					y_reg <= y_reg;// - 1'b1;
				else
					y_reg <= y_reg;// + 1'b1;
			end
			if (ld_colour)
				colour_reg <= inputSW;
			if (ld_direction) begin
				if (x_reg == 8'd0 || x_reg == 8'd160)
					Horizontal_1_LEFT <= ~Horizontal_1_LEFT;
				if (y_reg == 7'd0 || y_reg == 7'd120)
					Vertical_1_UP <= ~Vertical_1_UP;
			end
		end
	end
	
	always @(posedge CLOCK_50) begin
		if (!resetn) begin
			squareCounter <= 5'b00000;
		end
		else if (squareCounter == 5'b10000) begin
			squareCounter <= 5'b00000;
		end
		else if (square_counter_enable) begin
			squareCounter <= squareCounter + 1'b1;
		end
	end
	
	always @(*)
		begin : ALU
			// alu
			case (squareCounter)
				5'b00000: begin
					temp_xy = {{x_reg}, {y_reg}};
				end
				5'b00001: begin
					temp_xy = {{x_reg}, {y_reg + 2'b01}};
				end
				5'b00010: begin
					temp_xy = {{x_reg}, {y_reg + 2'b10}};
				end
				5'b00011: begin
					temp_xy = {{x_reg}, {y_reg + 2'b11}};
				end
				5'b00100: begin
					temp_xy = {{x_reg + 2'b01}, {y_reg}};
				end
				5'b00101: begin
					temp_xy = {{x_reg + 2'b01}, {y_reg + 2'b01}};
				end
				5'b00110: begin
					temp_xy = {{x_reg + 2'b01}, {y_reg + 2'b10}};
				end
				5'b00111: begin
					temp_xy = {{x_reg + 2'b01}, {y_reg + 2'b11}};
				end
				5'b01000: begin
					temp_xy = {{x_reg + 2'b10}, {y_reg}};
				end
				5'b01001: begin
					temp_xy = {{x_reg + 2'b10}, {y_reg + 2'b01}};
				end
				5'b01010: begin
					temp_xy = {{x_reg + 2'b10}, {y_reg + 2'b10}};
				end
				5'b01011: begin
					temp_xy = {{x_reg + 2'b10}, {y_reg + 2'b11}};
				end
				5'b01100: begin
					temp_xy = {{x_reg + 2'b11}, {y_reg}};
				end
				5'b01101: begin
					temp_xy = {{x_reg + 2'b11}, {y_reg + 2'b01}};
				end
				5'b01110: begin
					temp_xy = {{x_reg + 2'b11}, {y_reg + 2'b10}};
				end
				5'b01111: begin
					temp_xy = {{x_reg + 2'b11}, {y_reg + 2'b11}};
				end
				default: temp_xy = 15'b0;
			endcase
	end
	
		always @(*) begin
			if (!resetn)
				square_done = 1'b0;
			else
				square_done = (squareCounter == 5'b10000);
		end
 
	// Output result register
	always@(posedge CLOCK_50) begin
		if (!resetn) begin
			xy <= 15'b0; 
			colour <= 3'b000; 
		end
		else begin
			xy <= temp_xy;
			if (sel_colour)
				colour <= 3'b000;
			if (!sel_colour)
				colour <= colour_reg;
		end
	end

endmodule

module vga_adapter(
			resetn,
			clock,
			colour,
			x, y, plot,
			/* Signals for the DAC to drive the monitor. */
			VGA_R,
			VGA_G,
			VGA_B,
			VGA_HS,
			VGA_VS,
			VGA_BLANK,
			VGA_SYNC,
			VGA_CLK);
 
	parameter BITS_PER_COLOUR_CHANNEL = 1;
	/* The number of bits per colour channel used to represent the colour of each pixel. A value
	 * of 1 means that Red, Green and Blue colour channels will use 1 bit each to represent the intensity
	 * of the respective colour channel. For BITS_PER_COLOUR_CHANNEL=1, the adapter can display 8 colours.
	 * In general, the adapter is able to use 2^(3*BITS_PER_COLOUR_CHANNEL ) colours. The number of colours is
	 * limited by the screen resolution and the amount of on-chip memory available on the target device.
	 */	
	
	parameter MONOCHROME = "FALSE";
	/* Set this parameter to "TRUE" if you only wish to use black and white colours. Doing so will reduce
	 * the amount of memory you will use by a factor of 3. */
	
	parameter RESOLUTION = "320x240";
	/* Set this parameter to "160x120" or "320x240". It will cause the VGA adapter to draw each dot on
	 * the screen by using a block of 4x4 pixels ("160x120" resolution) or 2x2 pixels ("320x240" resolution).
	 * It effectively reduces the screen resolution to an integer fraction of 640x480. It was necessary
	 * to reduce the resolution for the Video Memory to fit within the on-chip memory limits.
	 */
	
	parameter BACKGROUND_IMAGE = "background.mif";
	/* The initial screen displayed when the circuit is first programmed onto the DE2 board can be
	 * defined useing an MIF file. The file contains the initial colour for each pixel on the screen
	 * and is placed in the Video Memory (VideoMemory module) upon programming. Note that resetting the
	 * VGA Adapter will not cause the Video Memory to revert to the specified image. */


	/*****************************************************************************/
	/* Declare inputs and outputs.                                               */
	/*****************************************************************************/
	input resetn;
	input clock;
	
	/* The colour input can be either 1 bit or 3*BITS_PER_COLOUR_CHANNEL bits wide, depending on
	 * the setting of the MONOCHROME parameter.
	 */
	input [((MONOCHROME == "TRUE") ? (0) : (BITS_PER_COLOUR_CHANNEL*3-1)):0] colour;
	
	/* Specify the number of bits required to represent an (X,Y) coordinate on the screen for
	 * a given resolution.
	 */
	input [((RESOLUTION == "320x240") ? (8) : (7)):0] x; 
	input [((RESOLUTION == "320x240") ? (7) : (6)):0] y;
	
	/* When plot is high then at the next positive edge of the clock the pixel at (x,y) will change to
	 * a new colour, defined by the value of the colour input.
	 */
	input plot;
	
	/* These outputs drive the VGA display. The VGA_CLK is also used to clock the FSM responsible for
	 * controlling the data transferred to the DAC driving the monitor. */
	output [7:0] VGA_R;
	output [7:0] VGA_G;
	output [7:0] VGA_B;
	output VGA_HS;
	output VGA_VS;
	output VGA_BLANK;
	output VGA_SYNC;
	output VGA_CLK;

	/*****************************************************************************/
	/* Declare local signals here.                                               */
	/*****************************************************************************/
	
	wire valid_160x120;
	wire valid_320x240;
	/* Set to 1 if the specified coordinates are in a valid range for a given resolution.*/
	
	wire writeEn;
	/* This is a local signal that allows the Video Memory contents to be changed.
	 * It depends on the screen resolution, the values of X and Y inputs, as well as 
	 * the state of the plot signal.
	 */
	
	wire [((MONOCHROME == "TRUE") ? (0) : (BITS_PER_COLOUR_CHANNEL*3-1)):0] to_ctrl_colour;
	/* Pixel colour read by the VGA controller */
	
	wire [((RESOLUTION == "320x240") ? (16) : (14)):0] user_to_video_memory_addr;
	/* This bus specifies the address in memory the user must write
	 * data to in order for the pixel intended to appear at location (X,Y) to be displayed
	 * at the correct location on the screen.
	 */
	
	wire [((RESOLUTION == "320x240") ? (16) : (14)):0] controller_to_video_memory_addr;
	/* This bus specifies the address in memory the vga controller must read data from
	 * in order to determine the colour of a pixel located at coordinate (X,Y) of the screen.
	 */
	
	wire clock_25;
	/* 25MHz clock generated by dividing the input clock frequency by 2. */
	
	wire vcc, gnd;
	
	/*****************************************************************************/
	/* Instances of modules for the VGA adapter.                                 */
	/*****************************************************************************/	
	assign vcc = 1'b1;
	assign gnd = 1'b0;
	
	vga_address_translator user_input_translator(
					.x(x), .y(y), .mem_address(user_to_video_memory_addr) );
		defparam user_input_translator.RESOLUTION = RESOLUTION;
	/* Convert user coordinates into a memory address. */

	assign valid_160x120 = (({1'b0, x} >= 0) & ({1'b0, x} < 160) & ({1'b0, y} >= 0) & ({1'b0, y} < 120)) & (RESOLUTION == "160x120");
	assign valid_320x240 = (({1'b0, x} >= 0) & ({1'b0, x} < 320) & ({1'b0, y} >= 0) & ({1'b0, y} < 240)) & (RESOLUTION == "320x240");
	assign writeEn = (plot) & (valid_160x120 | valid_320x240);
	/* Allow the user to plot a pixel if and only if the (X,Y) coordinates supplied are in a valid range. */
	
	/* Create video memory. */
	altsyncram	VideoMemory (
				.wren_a (writeEn),
				.wren_b (gnd),
				.clock0 (clock), // write clock
				.clock1 (clock_25), // read clock
				.clocken0 (vcc), // write enable clock
				.clocken1 (vcc), // read enable clock				
				.address_a (user_to_video_memory_addr),
				.address_b (controller_to_video_memory_addr),
				.data_a (colour), // data in
				.q_b (to_ctrl_colour)	// data out
				);
	defparam
		VideoMemory.WIDTH_A = ((MONOCHROME == "FALSE") ? (BITS_PER_COLOUR_CHANNEL*3) : 1),
		VideoMemory.WIDTH_B = ((MONOCHROME == "FALSE") ? (BITS_PER_COLOUR_CHANNEL*3) : 1),
		VideoMemory.INTENDED_DEVICE_FAMILY = "Cyclone II",
		VideoMemory.OPERATION_MODE = "DUAL_PORT",
		VideoMemory.WIDTHAD_A = ((RESOLUTION == "320x240") ? (17) : (15)),
		VideoMemory.NUMWORDS_A = ((RESOLUTION == "320x240") ? (76800) : (19200)),
		VideoMemory.WIDTHAD_B = ((RESOLUTION == "320x240") ? (17) : (15)),
		VideoMemory.NUMWORDS_B = ((RESOLUTION == "320x240") ? (76800) : (19200)),
		VideoMemory.OUTDATA_REG_B = "CLOCK1",
		VideoMemory.ADDRESS_REG_B = "CLOCK1",
		VideoMemory.CLOCK_ENABLE_INPUT_A = "BYPASS",
		VideoMemory.CLOCK_ENABLE_INPUT_B = "BYPASS",
		VideoMemory.CLOCK_ENABLE_OUTPUT_B = "BYPASS",
		VideoMemory.POWER_UP_UNINITIALIZED = "FALSE",
		VideoMemory.INIT_FILE = BACKGROUND_IMAGE;
		
	vga_pll mypll(clock, clock_25);
	/* This module generates a clock with half the frequency of the input clock.
	 * For the VGA adapter to operate correctly the clock signal 'clock' must be
	 * a 50MHz clock. The derived clock, which will then operate at 25MHz, is
	 * required to set the monitor into the 640x480@60Hz display mode (also known as
	 * the VGA mode).
	 */

	wire [9:0] r;
	wire [9:0] g;
	wire [9:0] b;
	
	/* Assign the MSBs from the controller to the VGA signals */
	
	assign VGA_R = r[9:2];
	assign VGA_G = g[9:2];
	assign VGA_B = b[9:2];
	
	vga_controller controller(
			.vga_clock(clock_25),
			.resetn(resetn),
			.pixel_colour(to_ctrl_colour),
			.memory_address(controller_to_video_memory_addr), 
			.VGA_R(r),
			.VGA_G(g),
			.VGA_B(b),
			.VGA_HS(VGA_HS),
			.VGA_VS(VGA_VS),
			.VGA_BLANK(VGA_BLANK),
			.VGA_SYNC(VGA_SYNC),
			.VGA_CLK(VGA_CLK)				
		);
		defparam controller.BITS_PER_COLOUR_CHANNEL  = BITS_PER_COLOUR_CHANNEL ;
		defparam controller.MONOCHROME = MONOCHROME;
		defparam controller.RESOLUTION = RESOLUTION;

endmodule

module vga_address_translator(x, y, mem_address);

	parameter RESOLUTION = "320x240";
	/* Set this parameter to "160x120" or "320x240". It will cause the VGA adapter to draw each dot on
	 * the screen by using a block of 4x4 pixels ("160x120" resolution) or 2x2 pixels ("320x240" resolution).
	 * It effectively reduces the screen resolution to an integer fraction of 640x480. It was necessary
	 * to reduce the resolution for the Video Memory to fit within the on-chip memory limits.
	 */

	input [((RESOLUTION == "320x240") ? (8) : (7)):0] x; 
	input [((RESOLUTION == "320x240") ? (7) : (6)):0] y;	
	output reg [((RESOLUTION == "320x240") ? (16) : (14)):0] mem_address;
	
	/* The basic formula is address = y*WIDTH + x;
	 * For 320x240 resolution we can write 320 as (256 + 64). Memory address becomes
	 * (y*256) + (y*64) + x;
	 * This simplifies multiplication a simple shift and add operation.
	 * A leading 0 bit is added to each operand to ensure that they are treated as unsigned
	 * inputs. By default the use a '+' operator will generate a signed adder.
	 * Similarly, for 160x120 resolution we write 160 as 128+32.
	 */
	wire [16:0] res_320x240 = ({1'b0, y, 8'd0} + {1'b0, y, 6'd0} + {1'b0, x});
	wire [15:0] res_160x120 = ({1'b0, y, 7'd0} + {1'b0, y, 5'd0} + {1'b0, x});
	
	always @(*)
	begin
		if (RESOLUTION == "320x240")
			mem_address = res_320x240;
		else
			mem_address = res_160x120[14:0];
	end
endmodule

module vga_controller(	vga_clock, resetn, pixel_colour, memory_address, 
		VGA_R, VGA_G, VGA_B,
		VGA_HS, VGA_VS, VGA_BLANK,
		VGA_SYNC, VGA_CLK);
	
	/* Screen resolution and colour depth parameters. */
	
	parameter BITS_PER_COLOUR_CHANNEL = 1;
	/* The number of bits per colour channel used to represent the colour of each pixel. A value
	 * of 1 means that Red, Green and Blue colour channels will use 1 bit each to represent the intensity
	 * of the respective colour channel. For BITS_PER_COLOUR_CHANNEL=1, the adapter can display 8 colours.
	 * In general, the adapter is able to use 2^(3*BITS_PER_COLOUR_CHANNEL) colours. The number of colours is
	 * limited by the screen resolution and the amount of on-chip memory available on the target device.
	 */	
	
	parameter MONOCHROME = "FALSE";
	/* Set this parameter to "TRUE" if you only wish to use black and white colours. Doing so will reduce
	 * the amount of memory you will use by a factor of 3. */
	
	parameter RESOLUTION = "320x240";
	/* Set this parameter to "160x120" or "320x240". It will cause the VGA adapter to draw each dot on
	 * the screen by using a block of 4x4 pixels ("160x120" resolution) or 2x2 pixels ("320x240" resolution).
	 * It effectively reduces the screen resolution to an integer fraction of 640x480. It was necessary
	 * to reduce the resolution for the Video Memory to fit within the on-chip memory limits.
	 */
	
	//--- Timing parameters.
	/* Recall that the VGA specification requires a few more rows and columns are drawn
	 * when refreshing the screen than are actually present on the screen. This is necessary to
	 * generate the vertical and the horizontal syncronization signals. If you wish to use a
	 * display mode other than 640x480 you will need to modify the parameters below as well
	 * as change the frequency of the clock driving the monitor (VGA_CLK).
	 */
	parameter C_VERT_NUM_PIXELS  = 10'd480;
	parameter C_VERT_SYNC_START  = 10'd493;
	parameter C_VERT_SYNC_END    = 10'd494; //(C_VERT_SYNC_START + 2 - 1); 
	parameter C_VERT_TOTAL_COUNT = 10'd525;

	parameter C_HORZ_NUM_PIXELS  = 10'd640;
	parameter C_HORZ_SYNC_START  = 10'd659;
	parameter C_HORZ_SYNC_END    = 10'd754; //(C_HORZ_SYNC_START + 96 - 1); 
	parameter C_HORZ_TOTAL_COUNT = 10'd800;	
		
	/*****************************************************************************/
	/* Declare inputs and outputs.                                               */
	/*****************************************************************************/
	
	input vga_clock, resetn;
	input [((MONOCHROME == "TRUE") ? (0) : (BITS_PER_COLOUR_CHANNEL*3-1)):0] pixel_colour;
	output [((RESOLUTION == "320x240") ? (16) : (14)):0] memory_address;
	output reg [9:0] VGA_R;
	output reg [9:0] VGA_G;
	output reg [9:0] VGA_B;
	output reg VGA_HS;
	output reg VGA_VS;
	output reg VGA_BLANK;
	output VGA_SYNC, VGA_CLK;
	
	/*****************************************************************************/
	/* Local Signals.                                                            */
	/*****************************************************************************/
	
	reg VGA_HS1;
	reg VGA_VS1;
	reg VGA_BLANK1; 
	reg [9:0] xCounter, yCounter;
	wire xCounter_clear;
	wire yCounter_clear;
	wire vcc;
	
	reg [((RESOLUTION == "320x240") ? (8) : (7)):0] x; 
	reg [((RESOLUTION == "320x240") ? (7) : (6)):0] y;	
	/* Inputs to the converter. */
	
	/*****************************************************************************/
	/* Controller implementation.                                                */
	/*****************************************************************************/

	assign vcc =1'b1;
	
	/* A counter to scan through a horizontal line. */
	always @(posedge vga_clock or negedge resetn)
	begin
		if (!resetn)
			xCounter <= 10'd0;
		else if (xCounter_clear)
			xCounter <= 10'd0;
		else
		begin
			xCounter <= xCounter + 1'b1;
		end
	end
	assign xCounter_clear = (xCounter == (C_HORZ_TOTAL_COUNT-1));

	/* A counter to scan vertically, indicating the row currently being drawn. */
	always @(posedge vga_clock or negedge resetn)
	begin
		if (!resetn)
			yCounter <= 10'd0;
		else if (xCounter_clear && yCounter_clear)
			yCounter <= 10'd0;
		else if (xCounter_clear)		//Increment when x counter resets
			yCounter <= yCounter + 1'b1;
	end
	assign yCounter_clear = (yCounter == (C_VERT_TOTAL_COUNT-1)); 
	
	/* Convert the xCounter/yCounter location from screen pixels (640x480) to our
	 * local dots (320x240 or 160x120). Here we effectively divide x/y coordinate by 2 or 4,
	 * depending on the resolution. */
	always @(*)
	begin
		if (RESOLUTION == "320x240")
		begin
			x = xCounter[9:1];
			y = yCounter[8:1];
		end
		else
		begin
			x = xCounter[9:2];
			y = yCounter[8:2];
		end
	end
	
	/* Change the (x,y) coordinate into a memory address. */
	vga_address_translator controller_translator(
					.x(x), .y(y), .mem_address(memory_address) );
		defparam controller_translator.RESOLUTION = RESOLUTION;


	/* Generate the vertical and horizontal synchronization pulses. */
	always @(posedge vga_clock)
	begin
		//- Sync Generator (ACTIVE LOW)
		VGA_HS1 <= ~((xCounter >= C_HORZ_SYNC_START) && (xCounter <= C_HORZ_SYNC_END));
		VGA_VS1 <= ~((yCounter >= C_VERT_SYNC_START) && (yCounter <= C_VERT_SYNC_END));
		
		//- Current X and Y is valid pixel range
		VGA_BLANK1 <= ((xCounter < C_HORZ_NUM_PIXELS) && (yCounter < C_VERT_NUM_PIXELS));	
	
		//- Add 1 cycle delay
		VGA_HS <= VGA_HS1;
		VGA_VS <= VGA_VS1;
		VGA_BLANK <= VGA_BLANK1;	
	end
	
	/* VGA sync should be 1 at all times. */
	assign VGA_SYNC = vcc;
	
	/* Generate the VGA clock signal. */
	assign VGA_CLK = vga_clock;
	
	/* Brighten the colour output. */
	// The colour input is first processed to brighten the image a little. Setting the top
	// bits to correspond to the R,G,B colour makes the image a bit dull. To brighten the image,
	// each bit of the colour is replicated through the 10 DAC colour input bits. For example,
	// when BITS_PER_COLOUR_CHANNEL is 2 and the red component is set to 2'b10, then the
	// VGA_R input to the DAC will be set to 10'b1010101010.
	
	integer index;
	integer sub_index;
	
	always @(pixel_colour)
	begin		
		VGA_R <= 'b0;
		VGA_G <= 'b0;
		VGA_B <= 'b0;
		if (MONOCHROME == "FALSE")
		begin
			for (index = 10-BITS_PER_COLOUR_CHANNEL; index >= 0; index = index - BITS_PER_COLOUR_CHANNEL)
			begin
				for (sub_index = BITS_PER_COLOUR_CHANNEL - 1; sub_index >= 0; sub_index = sub_index - 1)
				begin
					VGA_R[sub_index+index] <= pixel_colour[sub_index + BITS_PER_COLOUR_CHANNEL*2];
					VGA_G[sub_index+index] <= pixel_colour[sub_index + BITS_PER_COLOUR_CHANNEL];
					VGA_B[sub_index+index] <= pixel_colour[sub_index];
				end
			end	
		end
		else
		begin
			for (index = 0; index < 10; index = index + 1)
			begin
				VGA_R[index] <= pixel_colour[0:0];
				VGA_G[index] <= pixel_colour[0:0];
				VGA_B[index] <= pixel_colour[0:0];
			end	
		end
	end

endmodule

module vga_pll (
	clock_in,
	clock_out);

	input	  clock_in;
	output	  clock_out;

	wire [5:0] clock_output_bus;
	wire [1:0] clock_input_bus;
	wire gnd;
	
	assign gnd = 1'b0;
	assign clock_input_bus = { gnd, clock_in }; 

	altpll	altpll_component (
				.inclk (clock_input_bus),
				.clk (clock_output_bus)
				);
	defparam
		altpll_component.operation_mode = "NORMAL",
		altpll_component.intended_device_family = "Cyclone II",
		altpll_component.lpm_type = "altpll",
		altpll_component.pll_type = "FAST",
		/* Specify the input clock to be a 50MHz clock. A 50 MHz clock is present
		 * on PIN_N2 on the DE2 board. We need to specify the input clock frequency
		 * in order to set up the PLL correctly. To do this we must put the input clock
		 * period measured in picoseconds in the inclk0_input_frequency parameter.
		 * 1/(20000 ps) = 0.5 * 10^(5) Hz = 50 * 10^(6) Hz = 50 MHz. */
		altpll_component.inclk0_input_frequency = 20000,
		altpll_component.primary_clock = "INCLK0",
		/* Specify output clock parameters. The output clock should have a
		 * frequency of 25 MHz, with 50% duty cycle. */
		altpll_component.compensate_clock = "CLK0",
		altpll_component.clk0_phase_shift = "0",
		altpll_component.clk0_divide_by = 2,
		altpll_component.clk0_multiply_by = 1,		
		altpll_component.clk0_duty_cycle = 50;
		
	assign clock_out = clock_output_bus[0];

endmodule



