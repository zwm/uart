
module tb_top();
// macro
`include "tb_define.v"
// port
reg                         rstn;
reg                         clk;
reg                         rx_din;
reg                         rx_en;
reg         [15:0]          baud_rate;
reg                         word_len;       // 0: 8-bit, 1: 9-bit
reg                         parity_en;      // 
reg                         parity_type;    // 0: even, 1: odd
reg         [1:0]           stop_len;       // 00: 1, 01: 0.5, 10: 2, 11: 1.5
wire                        rx_vld_p;
wire        [7:0]           rx_byte;
wire                        rx_parity;
wire                        start_noise_p;
wire                        data_noise_p;
wire                        parity_noise_p;
wire                        stop_noise_p;
wire                        parity_err_p;
wire                        stop_err_p;
wire        [7:0]           rx_state;
// cfg
reg                         cfg_start_err;
reg                         cfg_parity_err;
reg                         cfg_stop_err;
reg                         cfg_start_noise;
reg                         cfg_data_noise;
reg                         cfg_parity_noise;
reg                         cfg_stop_noise;
reg   [3:0]                 cfg_byte_len;       // 7/8/9
reg                         cfg_parity_en;
reg                         cfg_parity_type;          // 0: even, 1: odd
reg   [1:0]                 cfg_stop_len;       // 0: 1
reg   [15:0]                cfg_baud_rate;
// global
integer err_cnt, chk_cnt, case_num, test_cnt;

// main
initial begin
    // init
    sys_init;
    #1_000;

    // main
    main_loop;

    // disp
    #1_000;
    disp_sum;

    #1_000;
    $finish;
end


uart_receiver u_uart_rx (
    .rstn                   ( rstn                      ),
    .clk                    ( clk                       ),
    .rx_din                 ( rx_din                    ),
    .rx_en                  ( rx_en                     ),
    .baud_rate              ( baud_rate                 ),
    .word_len               ( word_len                  ),
    .parity_en              ( parity_en                 ),
    .parity_type            ( parity_type               ),
    .stop_len               ( stop_len                  ),
    .rx_vld_p               ( rx_vld_p                  ),
    .rx_byte                ( rx_byte                   ),
    .rx_parity              ( rx_parity                 ),
    .start_noise_p          ( start_noise_p             ),
    .data_noise_p           ( data_noise_p              ),
    .parity_noise_p         ( parity_noise_p            ),
    .stop_noise_p           ( stop_noise_p              ),
    .parity_err_p           ( parity_err_p              ),
    .stop_err_p             ( stop_err_p                ),
    .rx_state               ( rx_state                  )
);

// fsdb
`ifdef DUMP_FSDB
initial begin
    $fsdbDumpfile("tb_top.fsdb");
    $fsdbDumpvars(0, tb_top);
    `ifdef DUMP_ARRAY
        $fsdbDumpMDA();
    `endif
end
`endif

// clk gen
initial begin
    // init
    rstn                    = 1'bx;
    clk                     = 1'b0;
    // drive
    fork
        // rstn
        begin
            #50;
            rstn            = 0;
            #100;
            rstn            = 1;
        end
        // clk
        begin
            #200;
            forever #`CLK_PERIOD clk = ~clk;
        end
    join
end

// sys_init
task sys_init;
    begin
        // sys
        case_num                = 0;
        chk_cnt                 = 0;
        err_cnt                 = 0;
        test_cnt                = 0;
        // dut
        rx_din                  = 1;
        rx_en                   = 0;
        baud_rate               = 0;
        word_len                = 0;        // 0: 8-bit, 1: 9-bit
        parity_en               = 0;        // 
        parity_type             = 0;        // 0: even, 1: odd
        stop_len                = 0;        // 00: 1, 01: 0.5, 10: 2, 11: 1.5
    end
endtask

task main_loop;
    integer i, j, tmp, br;
    begin
        // wait
        wait (rstn === 1'b1);
        #500;
        // reg init
        br                      = 48;
        rx_en                   = 1;
        baud_rate               = br;
        word_len                = 1;        // 0: 8-bit, 1: 9-bit
        parity_en               = 1;        // 
        parity_type             = 0;        // 0: even, 1: odd
        stop_len                = 0;        // 00: 1, 01: 0.5, 10: 2, 11: 1.5
        // gen_byte
        cfg_start_err           = 0;
        cfg_parity_err          = 0;
        cfg_stop_err            = 0;
        cfg_start_noise         = 1;
        cfg_data_noise          = 1;
        cfg_parity_noise        = 1;
        cfg_stop_noise          = 0;
        cfg_byte_len            = 8;       // 7/8/9
        cfg_parity_en           = 1;
        cfg_parity_type         = 0;        // 0: even, 1: odd
        cfg_stop_len            = 0;       // 0: 1
        cfg_baud_rate           = br;
        for (i=0; i<10; i=i+1) begin
            gen_byte(8'h55, 1'b0);
            test_cnt = test_cnt + 1;
            #500;
        end
    end
endtask

task bit_gen;
    input tx_bit;
    input [15:0] tog_idx0;
    input [15:0] tog_idx1;
    input [15:0] tog_idx2;
    input [15:0] tog_idx3;
    reg [15:0] i;
    begin
        // 16 cycles
        for (i=0; i<16; i=i+1) begin
            if ((i == tog_idx0) || (i == tog_idx1) || (i == tog_idx2) || (i == tog_idx3))
                rx_din = ~tx_bit;
            else
                rx_din = tx_bit;
            repeat(cfg_baud_rate[15:4]) @(posedge clk);
        end
        // frac
        if (cfg_baud_rate[3:0] != 0)
            repeat(cfg_baud_rate[3:0]) @(posedge clk);
    end
endtask
       
    integer i, r, rm, r0, r1, r2, r3;
task gen_byte;
    input [7:0] data_byte;
    input data_8b;
    reg parity, td;
    begin
        // start
        @(posedge clk);
        rx_din = 1'b0;
        parity = 1'b0;
        r = {$random(`RAND_SEED + test_cnt*100 + 0)}%16;
        rm = r[3];
        if (cfg_start_err) begin
            if (rm) begin
                if (r%3 == 0) begin r0 = 5; r1 = 7; r2 = 20; r3 = 20; end
                if (r%3 == 1) begin r0 = 3; r1 = 7; r2 = 20; r3 = 20; end
                if (r%3 == 2) begin r0 = 3; r1 = 5; r2 = 20; r3 = 20; end
            end
            else begin
                if (r%3 == 0) begin r0 = 9; r1 = 10; r2 = 20; r3 = 20; end
                if (r%3 == 1) begin r0 = 8; r1 = 10; r2 = 20; r3 = 20; end
                if (r%3 == 2) begin r0 = 8; r1 =  9; r2 = 20; r3 = 20; end
            end
        end
        else if (cfg_start_noise) begin
            if (rm) begin
                if (r%3 == 0) begin r0 = 3; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 1) begin r0 = 5; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 2) begin r0 = 7; r1 = 20; r2 = 20; r3 = 20; end
            end
            else begin
                if (r%3 == 0) begin r0 = 8; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 1) begin r0 = 9; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 2) begin r0 = 10; r1 = 20; r2 = 20; r3 = 20; end
            end
        end
        else begin
            r0 = 20;
            r1 = 20;
            r2 = 20;
            r3 = 20;
        end
        bit_gen(0, r0, r1, r2, r3);
        // data byte
        for(i=0; i<cfg_byte_len; i=i+1) begin
            r = {$random(`RAND_SEED + test_cnt*100 + 1 + i)}%16;
            if (cfg_data_noise) begin
                if (r%3 == 0) begin r0 = 8; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 1) begin r0 = 9; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 2) begin r0 = 10; r1 = 20; r2 = 20; r3 = 20; end
            end
            else begin
                r0 = 20; r1 = 20; r2 = 20; r3 = 20;
            end
            if (i<8) // load data
                td = data_byte[i];
            else
                td = data_8b;
            parity = parity ^ td; // calc parity
            bit_gen(td, r0, r1, r2, r3);
        end
        // parity
        if (cfg_parity_en) begin
            r = {$random(`RAND_SEED + test_cnt*100 + 11)}%16;
            if (cfg_parity_noise) begin
                if (r%3 == 0) begin r0 = 8; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 1) begin r0 = 9; r1 = 20; r2 = 20; r3 = 20; end
                if (r%3 == 2) begin r0 = 10; r1 = 20; r2 = 20; r3 = 20; end
            end
            else begin
                r0 = 20; r1 = 20; r2 = 20; r3 = 20;
            end
            td = parity ^ cfg_parity_type;
            bit_gen(td, r0, r1, r2, r3);
        end
        // stop
        rx_din = 1'b1;
        case (cfg_stop_len)
            0: repeat(cfg_baud_rate) @(posedge clk);
            1: repeat(cfg_baud_rate[14:1]) @(posedge clk);
            2: repeat({cfg_baud_rate[15:0], 1'b0}) @(posedge clk);
            3: repeat(cfg_baud_rate + cfg_baud_rate[14:1]) @(posedge clk);
        endcase
    end
endtask

task disp_sum;
    begin
        $display("---------------------------------------------------");
        $display("---------------------------------------------------");
        $display("---------------------------------------------------");
        $display("  chk_cnt: %d", chk_cnt);
        if (err_cnt == 0) begin
            $display("      PASS.");
        end
        else begin
            $display("  err_cnt: %d", err_cnt);
            $display("      FAIL!");
        end
        $display("---------------------------------------------------");
    end
endtask

endmodule

