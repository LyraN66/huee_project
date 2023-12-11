module esc_1(
    input clk,
    input rst,  // SW0
    // I2C Ports
    inout sda,  // JB1
    input scl,  // JB2
    output sda_enable, // JB3
    
    input pwm_en,   // SW15
    input encoder_a,    // JB4
    input encoder_b,    // JB7
    output motor_positive,  // JB8
    output motor_negative   // JB9
    );
    
    // PID inputs
    wire [15:0] pwm_period_wire;
    wire [15:0] period_reference_i2c_wire;
    wire [7:0] Kp_ext_i2c_wire;
    wire [7:0] Ki_ext_i2c_wire;
    wire [6:0] Kd_ext_i2c_wire;
    wire override_internal_pid_i2c_wire;
    
    wire busy_wire;
    wire valid_wire;
    wire [7:0] data_out_i2c;
    wire [7:0] data_out_ram;
    wire write_i2c;
    wire read_1_i2c;
    wire [7:0] index_1_i2c;
    wire [7:0] Kp_int_wire;
    wire [3:0] tunerreset_autotune;
    
    I2C_SLAVE_1 I2C_SLAVE(
        . clk(clk),
        . rst(rst),
        // I2C Ports
        . sda(sda),
        . scl(scl),
        . sda_enable(sda_enable),
        // RAM control signals
        . write(write_i2c),
        . read_1(read_1_i2c),
        . index_1(index_1_i2c),
        . data_out(data_out_i2c),
        . data_in(data_out_ram),
        // control signals
        . busy(busy_wire),
        . valid(valid_wire)
    );
    
    Register_Module_1 RAM_BLOCK(
        . clk(clk),
        . rst(rst),
        . write(write_i2c),
        . read_1(read_1_i2c),
        . index_1(index_1_i2c),
        . data_in(data_out_i2c),
        . data_out_1(data_out_ram),
        //
        . pwm_period(pwm_period_wire),           // 0x40-0x41
        . period_reference(period_reference_i2c_wire),  //0x40
        . Kp_ext(Kp_ext_i2c_wire),            //0x41
        . Ki_ext(Ki_ext_i2c_wire),            //0x42
        . Kd_ext(Kd_ext_i2c_wire),            //0x43
        . Kp_int_i(Kp_int_wire), 
        . override_internal_pid(override_internal_pid_i2c_wire),   //0x44
        . tunerreset_autotune(tunerreset_autotune)             // 0x48
    );
    
    bldc_esc_1 BLDC_ESC(
      . clk(clk),
      . reset(rst),
      . tunerreset_autotune(tunerreset_autotune),
      . pwm_en(pwm_en),
      . encoder_a(encoder_a),
      . encoder_b(encoder_b),
      . pwm_period(pwm_period_wire),
      . period_reference(period_reference_i2c_wire),
      . Kp_ext(Kp_ext_i2c_wire),
      . Ki_ext(Ki_ext_i2c_wire),
      . Kd_ext(Kd_ext_i2c_wire),
      . override_internal_pid(override_internal_pid_i2c_wire),
      . Kp_int_o(Kp_int_wire),
      . motor_positive(motor_positive),
      . motor_negative(motor_negative)
    );
endmodule

module I2C_SLAVE_1#(parameter debounce = 3)(
    input clk,
    input rst,
    // I2C Ports
    input scl,
    inout sda,
    output sda_enable,
    // RAM control signals 
    output write,
    output read_1,
    output reg [7:0] index_1,
    output [7:0] data_out,
    input [7:0] data_in,
    // control signals
    output reg busy,
    output reg valid
    );
    
    assign write = (!data_will_send && valid && !send_operation) ? (1'b1) : (1'b0);
    assign data_out = temp_data;
    
    assign read_1 = ((data_will_send && bit_count==5'd18) || (data_will_send && bit_count==5'd9)) ? (1'b1) : (1'b0);
    assign sda_enable = sda_enable_reg ;
    
    localparam TIME_THDSTA  =15;         //(0x6us/clock);     // 0.6 us 29 
    localparam TIME_TLOW    =15;        //(0x6us/clock);    // 1.3 us 63
    localparam I2CBITS      =29;
    
    
    reg [6:0] I2C_SLAVE_ADDR;
    
    reg [7:0] i2c_data;
    reg [28:0] i2c_capt;
    
    reg [31:0]counter;
    reg counter_reset;
    reg send_operation;
    
    reg [4:0] bit_count;
    reg [7:0] temp_data;
    reg capture_en;
    
    reg ack_sended;
    reg nack_sended;
    reg half_ok;
    reg data_will_send;
    reg received_one;
    
    reg sda_en;
    reg done_high;
    reg sda_enable_reg ;
    
    reg [31:0] t_high;
    wire [31:0] t_high_2 = t_high>>1;
    reg [31:0] t_low;
    wire [31:0] t_low_2 = t_low>>1;
    reg captured;
    reg distance;
    reg sda_high;
    reg restart;
    
    reg scl_reg;
    reg [debounce-1:0] scl_shift_reg;
    reg sda_reg;
    reg [debounce-1:0] sda_shift_reg;
    
    assign sda = (sda_en) ? (1'bz) : (1'b0);
    
    reg [2:0] fsm_state;
    parameter IDLE  = 0;
    parameter START = 1;
    parameter TLOW  = 2;
    parameter THIGH = 3;
    parameter TSTO  = 4;
    
    always @(posedge clk)begin
        if (rst)begin
        
            I2C_SLAVE_ADDR  <= 7'h72;
            temp_data       <= 8'b0;
            index_1         <= 8'b0;
            send_operation  <= 1'b0;
            
            scl_reg         <= 1'b1;
            scl_shift_reg   <= {debounce{1'b1}};
            sda_reg         <= 1'b1;
            sda_shift_reg   <= {debounce{1'b1}};
            
            sda_en          <= 1'b1;
            counter_reset   <= 1'b0;
            counter         <= 32'b0;
            bit_count       <= 5'b0;
            captured        <= 1'b0;
            distance        <= 1'b0;
            sda_high        <= 1'b0;
            restart         <= 1'b0;
            done_high       <= 1'b0;
            
            t_high          <= 32'b0;
            t_low           <= 32'b0;
            
            fsm_state       <= 3'b0;
            busy            <= 1'b0;
            valid           <= 1'b0;
            data_will_send  <= 1'b0;
            received_one    <= 1'b0;
            
            i2c_data        <= 8'b0;
            i2c_capt        <= 29'b0;
            
            ack_sended      <= 1'b0;
            nack_sended     <= 1'b0;
            half_ok         <= 1'b0;
            sda_enable_reg  <= 1'b1;
        end
        else begin
        
            scl_shift_reg <= {scl_shift_reg[debounce-2:0],scl};
            
            if (scl_shift_reg == {debounce{1'b0}} || scl_shift_reg == {debounce{1'b1}})begin
                scl_reg <= scl_shift_reg[0];
            end
        
            sda_shift_reg <= {sda_shift_reg[debounce-2:0],sda};
            
            if (sda_shift_reg == {debounce{1'b0}} || sda_shift_reg == {debounce{1'b1}})begin
                sda_reg <= sda_shift_reg[0];
            end
            
            capture_en <= i2c_capt[I2CBITS - bit_count - 1];
            
            if((data_will_send && !received_one && (bit_count==5'd18)) || (data_will_send && !received_one && (bit_count==5'd9) && restart))begin
                i2c_data <= data_in;
                received_one <= 1'b1;
            end
            if(counter_reset)begin
                counter <= 32'b0;
                counter_reset <= 1'b0;
            end
            else begin
                counter <= counter + 1'b1;
            end
            case (fsm_state)
                IDLE:begin
                    sda_en <= 1'b1;
                    sda_enable_reg <= 1'b1;
                    if ((counter == (t_high + t_low)) || distance )begin
                        i2c_capt        <= {1'b0, 8'hFF,   1'b0, 8'hFF,    1'b0, 8'hFF,  1'b0, 1'b0};
                        send_operation  <= 1'b0;
                        bit_count       <= 5'b0;
                        ack_sended      <= 1'b0;
                        nack_sended     <= 1'b0;
                        received_one    <= 1'b0;
                        distance        <= 1'b1;
                        busy            <= 1'b0;
                        sda_high        <= 1'b0;
                        restart         <= 1'b0;
                        valid           <= 1'b0;
                        captured        <= 1'b0;
                        counter_reset   <= (scl_reg && !sda_reg) ? (1'b0) : (1'b1);
                        fsm_state       <= (scl_reg && !sda_reg) ? (START) : (IDLE);
                    end
                end
                START:begin
                    distance <= 1'b0;
                    done_high <= 1'b0;
                    if(!scl_reg)begin
                        if(counter >= TIME_THDSTA)begin
                            fsm_state <= TLOW;
                            busy <= 1'b1; 
                            t_high <= (restart) ? (t_high) : (counter);
                            counter_reset <= 1'b1;
                        end
                        else begin
                            fsm_state <= IDLE;
                            counter_reset <= 1'b1;
                        end
                    end
                    if(sda_reg)begin
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                    end
                end
                TLOW:begin
                    if(scl_reg)begin
                        if(counter >= TIME_TLOW)begin
                            bit_count <= (restart && (bit_count==5'd9)) ? (5'd19) : (bit_count + 1'b1);
//                            fsm_state <= THIGH;
                            fsm_state <= (bit_count == I2CBITS-2) ? (TSTO) : (THIGH);
                            t_low <= counter;
                            captured <= 1'b1;
                            counter_reset <= 1'b1;
                        end
                        else begin
                            fsm_state <= IDLE;
                            counter_reset <= 1'b1;
                        end
                    end
                    if(captured)begin
                        if(counter == t_low_2)begin
                            if((data_will_send && (bit_count>17) && (bit_count<28)) || (data_will_send && (bit_count==5'd9)))begin
                                sda_en  <= (bit_count==5'd9) ? (i2c_data[7]) : (i2c_data[I2CBITS - bit_count - 4]);// problem in here
                                sda_enable_reg <= 1'b0;
                            end
                            else if(((bit_count==5'd8) || (bit_count==5'd17) || (bit_count==5'd26)))begin
                                if(ack_sended)begin
                                    sda_en <= 1'b0;
                                    sda_enable_reg <= 1'b0;
                                end
                                else if(nack_sended)begin
                                    sda_en <= 1'b1;
                                    sda_enable_reg <= 1'b0;
                                    fsm_state <= IDLE;
                                    counter_reset <= 1'b1;
                                end
                            end
                            else begin
                                sda_en <= 1'b1;
                                sda_enable_reg <= 1'b1;
                            end
                        end
                        else if ((counter >= (t_low<<3)) && !counter_reset)begin
                            fsm_state <= IDLE;
                            counter_reset <= 1'b1;
                        end
                    end
                end
                THIGH:begin
                    if(scl_reg && sda_reg && !restart && (bit_count == 5'd19))begin
                        sda_high <= 1'b1;
                    end
                    else if (scl_reg && !sda_reg && sda_high && !restart && (bit_count == 5'd19))begin
                        sda_high <= 1'b0;
                        restart <= 1'b1;
                        fsm_state <= START;
                        half_ok <= 1'b0;
                        bit_count <= 5'b0;
                    end
                    
                    if(!scl_reg && done_high)begin
                        fsm_state <= (TLOW);
                        done_high <= 1'b0;
                    end
                    
                    if((counter == t_high_2) && !done_high )begin
                        if((bit_count==5'd8) && half_ok)begin
                            if(temp_data[7:1]==I2C_SLAVE_ADDR)begin
                                ack_sended <= 1'b1;
                                i2c_capt  <= (!temp_data[0]) ? ({1'b0, 8'hFF,1'b0,   8'hFF,1'b0,   8'hFF,1'b0,   1'b0}): // write
                                                               ({1'b0, 8'hFF,1'b0,   8'hFF,1'b0,   8'h00,1'b0,   1'b0}); // read
                                data_will_send <= (!temp_data[0]) ? (1'b0) : (1'b1);
                                send_operation <= (!temp_data[0]) ? (send_operation) : (1'b1);
                            end
                            else begin
                                nack_sended <= 1'b1;
                            end
                        end
                        else if ((bit_count==5'd17) && half_ok)begin
                            if((temp_data>=8'h40) && (temp_data<=8'h53))begin
                                ack_sended <= 1'b1;
                                index_1 <= temp_data;
                            end
                            else begin
                                nack_sended <= 1'b1;
                            end
                        end
                        else if ((bit_count==5'd26) && half_ok)begin
                            data_will_send <= 1'b0;
                            ack_sended <= 1'b1;
                        end
                        else begin
                            ack_sended <= 1'b0;
                        end
                        
                        if(capture_en && !half_ok)begin
                            temp_data <= (temp_data << 1) | {7'b0,sda_reg};
                        end
                        half_ok <= ~half_ok;
                        done_high <= (half_ok);
                        if (!half_ok)begin
                            fsm_state <= (THIGH);
                        end
                        counter_reset <= 1'b1;
                    end
                    else if ((counter >= (t_high<<3)) && !counter_reset)begin
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                    end
                end
                TSTO:begin
                    
                    if(scl_reg && !sda_reg )begin
                        sda_high <= 1'b1;
                    end
                    else if (scl_reg && sda_reg && sda_high )begin
                        sda_high <= 1'b0;
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                        valid <= 1'b1;
                        busy <= 1'b0;
                    end 
                    if ((counter >= (t_high<<1)) && !counter_reset)begin
                        fsm_state <= IDLE;
                        counter_reset <= 1'b1;
                    end
                end
            endcase
        end
    end
endmodule

module Register_Module_1#(parameter depth = 20)(
    input clk,
    input rst,
    input write,
    input read_1,
    input [7:0] index_1,
    input [7:0] data_in,
    input [7:0] Kp_int_i,
    output [7:0] data_out_1,
    //
    output [15:0] pwm_period,           // 0x40-0x41
    output [15:0] period_reference,     // 0x42-0x43
    output [7:0] Kp_ext,               // 0x44
    output [7:0] Ki_ext,               // 0x45
    output [6:0] Kd_ext,               // 0x46 [6:0]
    output override_internal_pid,        // 0x46 [7]
    output [3:0] tunerreset_autotune              // 0x48
    );
    
    reg [7:0] internal_register [0:depth-1]; // start from 0x40
    assign data_out_1 = (read_1) ? (internal_register[index_1[4:0]]) : (8'b0);
    
    assign pwm_period            = {internal_register[0],internal_register[1]};
    assign period_reference      = {internal_register[2],internal_register[3]};
    assign Kp_ext                = internal_register[4];
    assign Ki_ext                = internal_register[5];
    assign Kd_ext                = internal_register[6][6:0];
    assign override_internal_pid = internal_register[6][7];
    assign tunerreset_autotune   = internal_register[8];
    
    always @(posedge clk)begin
        if(rst)begin
            internal_register [0] <= 8'b0;
            internal_register [1] <= 8'b0;
            internal_register [2] <= 8'b0;
            internal_register [3] <= 8'b0;
            internal_register [4] <= 8'b0;
            internal_register [5] <= 8'b0;
            internal_register [6] <= 8'b0;
            internal_register [7] <= 8'b0;
            internal_register [8] <= 8'b0;
            internal_register [9] <= 8'b0;
            internal_register [10] <= 8'b0;
            internal_register [11] <= 8'b0;
            internal_register [12] <= 8'b0;
            internal_register [13] <= 8'b0;
            internal_register [14] <= 8'b0;
            internal_register [15] <= 8'b0;
            internal_register [16] <= 8'b0;
            internal_register [17] <= 8'b0;
            internal_register [18] <= 8'b0;
            internal_register [19] <= 8'b0;
        end
        else begin
            internal_register[index_1[4:0]] <= (write) ? (data_in) : (internal_register[index_1[4:0]]);
            internal_register[7] <= Kp_int_i;
        end
    end
endmodule

module bldc_esc_1 #(parameter DATA_WIDTH = 16,parameter debounce = 3)(
  input clk,              		// clock input. when there is no clk signal motor does not run
  input reset,            		// when it is 1 motor does run active high
  input [3:0] tunerreset_autotune,
  input pwm_en,						//Pin to enable pwm output	active high
  input encoder_a,  				// encoder A pin input
  input encoder_b,  				// encoder B pin input
  input [DATA_WIDTH-1:0] pwm_period,	//change period in clock cycles
  input [DATA_WIDTH-1:0] period_reference,	// period_reference speed input, commented out since the ambiguity, will try with constant 900RPM period_reference speed
  input [7:0] Kp_ext,//External proportional constant
  input [7:0] Ki_ext,//External integral constant
  input [6:0] Kd_ext,//External derivative constant
  input override_internal_pid,		//select pin for external/internal pid constants
  output [7:0] Kp_int_o,
  output reg motor_positive,   		// BLDC motor PWM signal
  output reg motor_negative 	    // Motor direction control (1 bit)
);
    assign Kp_int_o = Kp_int_wire;
	reg [DATA_WIDTH-1:0] period_speed ;// Counter connected to rise of encoder A that will infer speed by measuring period of clk cycles
	wire tuning_done;
	reg [7:0] Kp_int;
	wire [7:0] Kp_int_wire = Kp_int;
	
	reg [7:0] Ki_int;
	wire [7:0] Ki_int_wire=Ki_int;
	reg [6:0] Kd_int;
	wire [6:0] Kd_int_wire=Kd_int;
pid_tuner tuner_inst_1(
	.clk(clk),
	.reset(reset | tunerreset_autotune[3]),
	.pid_select(tunerreset_autotune[2:0]),
	.period_speed(period_speed),
	.tuning_done(tuning_done),
	.Kp(Kp_int_wire),	//internal registers are read
	.Ki(Ki_int_wire),
	.Kd(Kd_int_wire)

);
  reg motor_pwm;
  reg [DATA_WIDTH-1:0] pwm_counter;
  reg [DATA_WIDTH-1:0] pwm_duty_cycle;
  reg [1:0]encoder_state;
  reg [1:0]prev_encoder_state;
  reg [1:0]pwm_direction;
  //PID control////////////////////////////////////////////////
  // PID constants
  reg [DATA_WIDTH-1:0] speed_ctr;
  reg [7:0] Kp ;
  reg [7:0] Ki ;
  reg [6:0] Kd ;
  // Variables for PID control
  reg signed[DATA_WIDTH-1:0] error;
  reg signed[(DATA_WIDTH)-1:0] integral;
  reg signed[DATA_WIDTH-1:0] derivative;
  reg signed[(DATA_WIDTH)-1:0] pid_output;
  reg signed[DATA_WIDTH-1:0] previous_error;
  
  reg [15:0] clk_counter ;
  reg [2:0] encoder_a_shift_reg;
  reg encoder_a_reg;
  reg [2:0] encoder_b_shift_reg;
  reg encoder_b_reg;
  reg [2:0] pwm_en_shift_reg;
  reg pwm_en_reg;
  reg flag;
  reg counter_rst;
  reg [1:0]encoder_a_set;
 always @(posedge clk) begin
    if (reset) begin
        clk_counter <= 16'b0;
        counter_rst <= 1'b1;
        encoder_a_set <= 2'b0;
        flag <= 1'b0;
        integral <= 16'b0;
        pwm_counter <= {DATA_WIDTH{1'b0}};
        encoder_state <= 2'b0;
        prev_encoder_state <= 2'b0;
        pwm_direction<=2'b00;
        motor_positive<=1'b0;
        motor_negative<=1'b0;
        Kp <= 8'h1;
        Ki <= 8'd0;
        Kd <= 7'd0;
        speed_ctr<={DATA_WIDTH{1'b0}};
        previous_error <= {DATA_WIDTH{1'b0}};
        error<=16'b0;
        period_speed <= 16'b0;
        pid_output <= 16'b0;
        derivative <= 16'b0;
        pwm_duty_cycle <= 16'b0;
        motor_pwm <= 1'b0;
        
        encoder_a_shift_reg <= 3'b0;
        encoder_a_reg <= 1'b0;
        encoder_b_shift_reg <= 3'b0;
        encoder_b_reg <= 1'b0;
        pwm_en_shift_reg <= 3'b0;
        pwm_en_reg <= 1'b0;
    end 
    else begin
        if(clk_counter == 16'h0040)begin
            clk_counter <= 16'b0;
            
            pwm_en_shift_reg <= {pwm_en_shift_reg[debounce-2:0],pwm_en};
            if (pwm_en_shift_reg == {debounce{1'b0}} || pwm_en_shift_reg == {debounce{1'b1}})begin
                pwm_en_reg <= pwm_en_shift_reg[0];
            end
            
            encoder_a_shift_reg <= {encoder_a_shift_reg[debounce-2:0],encoder_a};
            if (encoder_a_shift_reg == {debounce{1'b0}} || encoder_a_shift_reg == {debounce{1'b1}})begin
                encoder_a_reg <= encoder_a_shift_reg[0];
            end
            
            encoder_b_shift_reg <= {encoder_b_shift_reg[debounce-2:0],encoder_b};
            if (encoder_b_shift_reg == {debounce{1'b0}} || encoder_b_shift_reg == {debounce{1'b1}})begin
                encoder_b_reg <= encoder_b_shift_reg[0];
            end
            
            pid_output <= (Kp * error) + (Ki * integral)>>3 + (Kd * derivative);	//compute pid response
            if(pid_output<1) begin			//if pid output is negative, cap at 0
                pwm_duty_cycle<=pwm_period;
            end 
            else if (pid_output>pwm_period) begin	//if pid output above period, give 100% pwm
                pwm_duty_cycle<=pwm_period>>2;
            end
            else begin
                pwm_duty_cycle<=pid_output;	//if in between values, give pid output as pwm
            end
            
            derivative <= error - previous_error;
            if (integral + error > 2048) begin //These are very high values for ASIC, need to push for lower register(?)
                integral <= 2047;
            end
            else if (integral + error < -2048) begin
                integral <= -2047;
            end
            else begin
                integral <= integral + error;
            end
            
            previous_error<=error;
            error <= period_reference - period_speed;
            motor_pwm <= (pwm_counter < pwm_duty_cycle) & pwm_en_reg; // Generate PWM signal and send it to the motor
            // Reset PWM counter at the end of the PWM period
            if (pwm_counter == pwm_period) begin
                pwm_counter <= 16'd0;
            end
            else begin
                pwm_counter <= pwm_counter + 1;
            end
            encoder_state <= {encoder_a_reg, encoder_b_reg};
            prev_encoder_state <= encoder_state;
            if(pwm_en_reg)begin
                case ({encoder_state, prev_encoder_state})
                    4'b0100, 4'b1101,4'b1011: begin// Rotate in one direction (e.g., forward)
                        pwm_direction<=2'b10;
                    end
                    4'b1000,4'b1110,4'b0111: begin// Rotate in the opposite direction (e.g., reverse)
                        pwm_direction<=2'b01;
                    end
                    4'b1100, 4'b0011, 4'b1001,4'b0110: begin// No change (no rotation)
                        pwm_direction<=2'b00;
                    end
                    default: begin
                        pwm_direction<=pwm_direction;// Keep the previous direction in case of an unexpected state
                    end
                endcase
            end
//            else begin
//                encoder_state <= 2'b0;
//                prev_encoder_state <= 2'b0;
//            end
            
            if(!flag && (pwm_period!=0))begin
                motor_positive <= (period_reference<32767) ? (1'b1) : (1'b0);
                motor_negative <= (period_reference>=32767) ? (1'b1) : (1'b0);
                flag <= ((period_speed>=150) && (period_reference>=period_speed)) ? (1'b1): (1'b0);
            end
            if(pwm_en_reg==1'b0)begin
                motor_positive<=1'b0;
                motor_negative<=1'b0;
            end
            else if (flag)begin
                case(pwm_direction)
                    2'b00: begin
                        if(period_reference>32767) begin //check negative references by 2's complement
                            motor_positive<=1'b0;
                            motor_negative<=motor_pwm;
                        end
                        else begin
                            motor_positive<=motor_pwm;
                            motor_negative<=1'b0;
                        end
                    end 
                    2'b01:begin
                        motor_positive<=1'b0;
                        motor_negative<=motor_pwm;
                    end
                    2'b10:begin
                        motor_positive<=motor_pwm;
                        motor_negative<=1'b0;
                    end
                    default: begin
                        motor_positive <= 1'b0;
                        motor_negative <= 1'b0;
                    end
                endcase
            end
            if(override_internal_pid) begin
                Kp<=Kp_ext;
                Ki<=Ki_ext;
                Kd<=Kd_ext;
            end 
		else if (tuning_done) begin
			Kp<=Kp_int;
			Ki<=Ki_int;
			Kd<=Kd_int;
		end
		else begin
			Kp<=Kp_int;
			Ki<=0;
			Kd<=0;
		end
            
            if(encoder_a_reg && encoder_a_set==2'b00)begin
              counter_rst <= 1'b0;
              encoder_a_set <= 2'b01;
            end
            else if(encoder_a_set==2'b01 && !encoder_a_reg)begin
              encoder_a_set <= 2'b10;
            end
            else if (encoder_a_reg && encoder_a_set==2'b10)begin
              counter_rst <= 1'b1;
              encoder_a_set <= 2'b00;
              period_speed<=speed_ctr;
            end
            
            if(counter_rst)begin
                speed_ctr <= 16'b0;
            end
            else begin
                speed_ctr<=speed_ctr+1;
            end
        end
        else begin
            clk_counter <= clk_counter + 1;
        end
    end
 end
endmodule

 module pid_tuner #(
  parameter DATA_WIDTH = 16, //only used at the 3rd subroutine
  parameter ENCODER_WIDTH = 3 //stop using it
)(
  input wire clk,              		// clock input. when there is no clk signal motor does not run, CLK @50MHz, T=20ns
  input wire reset,            		// when it is 1 motor does run
  input wire [2:0] pid_select,			//select control type: 100 for P, 110 for PI, 111 for PID (?)
  input wire [DATA_WIDTH-1:0] period_speed,//system output of speed measured in clock cycles
  output reg tuning_done,
	output reg [7:0] Kp,//proportional constant
	 output reg [7:0] Ki,//integral constant
	 output reg [6:0] Kd//derivative constant
  
  
);


	reg division_trig =1'b0;
	reg [31:0] dividend=32'd0;
	reg [31:0] divisor =32'd0;
	wire [31:0] quotient ;
	wire [31:0] remainder ;
	wire division_running;
   	reg reset_divider;
   	wire div_done;

	Divider32bit divider_inst(
   .clk(clk),
   .reset(reset_divider),
   .start_division(division_trig),  // Start signal for division
   .dividend(dividend),       // Dividend input
   .divisor(divisor),        // Divisor input
   .quotient(quotient),   // Quotient output
   .remainder(remainder),  // Remainder output
      .division_active(division_running),   // Declaration of division_active
      .division_done(div_done)
);

	
	reg [DATA_WIDTH-1:0] period_counter=16'd0;	//register for time between peaks
	reg [DATA_WIDTH-1:0] peak_period=16'd0;		//register for last time between peaks
	reg [DATA_WIDTH-1:0] period_speed_reg=16'd0;
	reg [DATA_WIDTH-1:0] prev_period_speed=16'd0;		//save previous speed
	reg [DATA_WIDTH-1:0] peak_level=16'd0;
	reg [DATA_WIDTH-1:0] prev_peak_level=16'd0;
   reg [DATA_WIDTH-1:0]	Kp_max=16'd1;				//temporary Kp for incrementation and find max, init to decimal 1 
	
   //find peaks
	reg autotune_finalized=1'b0;
	reg increasing_flag=1'b0;
	reg Kp_done;
	reg Ki_done;
	reg Kd_done;
	
	always @(posedge clk) begin
    if (reset) begin
		peak_period<=16'd0;
		Kp_done<=1'b0;
		Ki_done<=1'b0;
		Kd_done<=1'b0;
		division_trig <=1'b0;
		dividend<=32'd0;
		divisor<=32'd0;
      	period_counter<=16'd0;
      	period_speed_reg<=16'd0;
      	prev_period_speed<=16'd0;
      	peak_level<=16'd0;
      	prev_peak_level<=16'd0;
      	autotune_finalized<=1'b0;
      	increasing_flag<=1'b0;
      	Kp_max<=16'd1;
      	Kp<=8'b0;
      	Ki<=8'b0;
      	Kd<=7'b0;
      	tuning_done<=1'b0;
      	reset_divider<=1'b1;
    end else
	 begin 
		if(autotune_finalized==1'b0)
			begin
      	prev_period_speed<=period_speed_reg;
		period_speed_reg<=period_speed;
		if(prev_period_speed<period_speed_reg)	//increasing signal
		begin
			increasing_flag<=1'b1;
		end else if (increasing_flag==1'b1 && prev_period_speed>period_speed_reg)	//found peak
		begin
          
          	peak_period<=period_counter;	//replace current period with counted period
          	period_counter<=16'd0;			//TODO: convert to non-blocking type
			prev_peak_level<=peak_level;
			peak_level<=prev_period_speed;	//made prev_period_speed instead of period_speed to get real peak			
          if (prev_peak_level!=16'b0) 
            begin
          if(peak_level==prev_peak_level)	//Found oscillation Kp if peaks are at the same level (?)
			begin
					autotune_finalized<=1'b1;
					
			end else if (peak_level<prev_peak_level)	//decreasing peaks, increment Kp to get closer to max value
			begin
					Kp_max<=Kp_max+1;
			end else begin	//increasing peaks case, unstable Kp has been achieved right after decreasing peaks,
							//decrement for Kpmax 
					Kp_max<=Kp_max-1;
			end
            end
          	Kp<=Kp_max;
          	//prev_peak_flag<=peak_flag;
          
          
          	//prev_peak_flag<=peak_flag;
			//peak_flag<=1'b1;
          	increasing_flag<=1'b0;
		end else begin
          	period_counter<=period_counter+1;		//count period at each clock cycle
        end
		
		 
		
		
		//if(peak_flag==1'b1) begin	//if peak is found, 
		//	
		//	peak_flag<=1'b0;					//reset peak found flag
		//end
		end
		
		else 
			begin
			//TODO:set outputs for different cases
					case (pid_select)
					3'b100: begin //SELECT PROPORTIONAL CONTROL
						Kp<=Kp_max>>1;	//right shift (div by 2) Kpmax to get Kp
						Kp_done<=1'b1;
						Ki<=0;
						Ki_done<=1'b1;
						Kd<=0;
						Kd_done<=1'b1;
					end
					3'b110: begin //SELECT PROPORTIONAL-INTEGRAL CONTROL
						if(Kp_done!=1'b1)
						begin
							//Kp<=Kp_max*45/100;
							if(division_trig==1'b0)
							begin
                              	reset_divider<=1'b1;
								dividend<=45*Kp_max;
								divisor<=100;
								division_trig<=1'b1;
							end else if (division_running==1'b0) begin
								Kp<=quotient;
								division_trig<=1'b0;
								Kp_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
                         	
						end else if (Ki_done!=1'b1)
							begin
								
							//Ki<=54*Kp_max/(peak_period*100);
								if(division_trig==1'b0)
							begin
								dividend<=54*Kp_max;
								divisor<=100*peak_period;
								division_trig<=1'b1;
							end else if (division_running==1'b0) begin
								Ki<=quotient;
								division_trig<=1'b0;
								Ki_done<=1'b1;
							end
							end
						Kd<=0;
						Kd_done<=1'b1;
					end
					3'b111: begin //SELECT PROPORTIONA-INTEGRAL-DERIVATIVE CONTROL
						if(Kp_done!=1'b1)
						begin
							//Kp<=Kp_max*6/10;
							if(division_trig==1'b0)
							begin
                              	reset_divider<=1'b1;
								dividend<=6*Kp_max;
								divisor<=10;
								division_trig<=1'b1;
                            end else if (div_done==1'b1) begin
								Kp<=quotient;
								division_trig<=1'b0;
								Kp_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
							end else if (Ki_done!=1'b1)
							begin
								
							//Ki<=12*Kp_max/(peak_period*10);
								if(division_trig==1'b0)
							begin
                              	
                              	reset_divider<=1'b1;
								dividend<=12*Kp_max;
								divisor<=10*peak_period;
								division_trig<=1'b1;
                            end else if (div_done) begin
								Ki<=quotient;
								division_trig<=1'b0;
								Ki_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
							end else if (Kd_done!=1'b1)
							begin
								//Kd<=3*Kp_max*peak_period/40;
								if(division_trig==1'b0)
							begin
                              	reset_divider<=1'b01;
                              	dividend<=(3*Kp_max*peak_period)>>2;
								divisor<=10;
								division_trig<=1'b1;
							end else if (div_done) begin
                              	
								Kd<=quotient;
								division_trig<=1'b0;
								Kd_done<=1'b1;
							end else begin
                              	reset_divider<=1'b0;
                            end
							end
					end
					default:begin
						Kp<=0;	//STOP PID operation in case of invalid input
						Ki<=0;
						Kd<=0;
						Kp_done<=1'b1;
						Ki_done<=1'b1;
						Kd_done<=1'b1;
					end
					endcase
					tuning_done<=Kp_done & Ki_done & Kd_done;
            end		
    	end
	end
	

endmodule 


module Divider32bit (
   input      clk,
   input      reset,
   input      start_division,  // Start signal for division
   input [31:0] dividend,       // Dividend input
   input [31:0] divisor,        // Divisor input
   output reg [31:0] quotient,   // Quotient output
   output reg [31:0] remainder,  // Remainder output
   output reg division_active,   // Declaration of division_active
  	output reg division_done
);

  reg [5:0] division_cycle = 6'b100001;
   // Internal registers
   reg [32:0] store_divisor;      // Assigned as the original divisor
   reg [32:0] shifting_divisor;     // One bit more than divisor
   reg [31:0] shifting_dividend;     // Assigned as the original dividend

   // State machine
   always @(posedge clk) begin
      if (reset) begin
         store_divisor <= 33'b0;
         shifting_divisor <= 33'b0;
         shifting_dividend <= dividend;
         division_active <= 0;  // Initialize division_active
      	 division_cycle <= 6'b100001; // Initialize division_cycle
       	division_done<=0;
      end
		else if (division_cycle == 6'b000000) begin
         division_active <= 0;
         quotient <= shifting_dividend;
         remainder <= shifting_divisor[32:1];
          division_done<=1;
         //remainder <= shifting_divisor[31:0];
         //quotient <= shifting_divisor[31:0];
      end
      else if (start_division && (division_cycle > 0)) begin
         // Move these assignments to the beginning of the block
         shifting_divisor <= {shifting_divisor[31:0], shifting_dividend[31]};
         shifting_dividend <= {shifting_dividend[30:0], 1'b0};
         
        if (shifting_divisor >= store_divisor) begin
            store_divisor <= {1'b0, divisor};  // Assign divisor to store_divisor
            shifting_divisor <= 33'b0;
            //shifting_dividend <= dividend;
            shifting_divisor[32:1] <= shifting_divisor[31:0] - store_divisor[31:0];
            shifting_divisor[0] <= shifting_dividend[31]; 
            shifting_dividend[0] <= 1;
         end
        division_cycle <= division_cycle - 1;
       	
         division_active <= 1;
      end else begin
         division_active <= 1;
      end

      /* if (division_cycle == 1) begin
         division_active <= 1;
         //remainder <= shifting_dividend;
         remainder <= shifting_divisor[32:1];
      end */
     
   end
endmodule