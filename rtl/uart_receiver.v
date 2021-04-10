
module uart_receiver (
    input                   rstn,
    input                   clk,
    input                   rx_din,
    input                   rx_en,
    input   [15:0]          baud_rate,
    input                   word_len,       // 0: 8-bit, 1: 9-bit
    input                   parity_en,      // 
    input                   parity_type,    // 0: even, 1: odd
    input   [1:0]           stop_len,       // 00: 1, 01: 0.5, 10: 2, 11: 1.5
    input                   samp_mode,      // 0: sample three times, 1: sample only one time
    input                   irda_mode,      // 0: normal uart mode, 1: 3/16 irda mode
    input                   ign_stop,       // 0: check stop, 1: ignore stop settings
    output                  rx_vld_p,
    output  [7:0]           rx_byte,
    output                  rx_parity,
    output                  start_noise_p,
    output                  data_noise_p,
    output                  parity_noise_p,
    output                  stop_noise_p,
    output                  parity_err_p,
    output                  stop_err_p,
    output  [7:0]           rx_state
);

// macro
localparam RX_IDLE          = 3'd0;
localparam RX_START         = 3'd1;
localparam RX_DATA          = 3'd2;
localparam RX_PARITY        = 3'd3;
localparam RX_STOP          = 3'd4;

// signal
reg [2:0] st_curr, st_next;
reg [11:0] cnt; reg [4:0] samp_cnt;
reg [1:0] bit0_num, bit1_num; reg [2:0] data_bit_cnt;
reg parity_buf; reg [7:0] data_buf;
wire [11:0] cnt_max; wire [2:0] data_bit_max;
wire cnt_start, cnt_end, bit_end, byte_end, stop_end;
reg stop_end_sel, stop_samp_en;
wire samp_point_pre, samp_point_mid;
wire start_judg, data_judg, stop_judg;
wire bit0_noise, bit1_noise, bit_noise, bit0_err, bit1_err;
wire start_samp_clr, data_samp_clr, stop_samp_clr;

// cnt
assign data_bit_max         = ~word_len & parity_en ? 3'd6 : 3'd7; // 9-bit mode, 9th receive in RX_PARITY state
assign cnt_max              = samp_cnt[4] ? ({8'h0, baud_rate[3:0]} - 12'd1) : (baud_rate[15:4] - 12'd1);
assign cnt_start            = cnt == 12'd0;
assign cnt_end              = cnt == cnt_max;
assign bit_end              = cnt_end && (samp_cnt == 5'd15);
assign byte_end             = data_bit_cnt == data_bit_max;
assign stop_end             = ign_stop ? (samp_cnt == 4) : stop_end_sel;
// sample
assign samp_point_pre       = irda_mode ? 1'b0 : samp_mode ? (samp_cnt == 5) : (samp_cnt == 3 || samp_cnt == 5 || samp_cnt == 7);
assign samp_point_mid       = irda_mode ? (samp_cnt == 2) : samp_mode ? (samp_cnt == 9) : (samp_cnt == 8 || samp_cnt == 9 || samp_cnt == 10);
assign start_samp_clr       = irda_mode ? samp_cnt == 1 && cnt_start : (samp_cnt == 1 || samp_cnt == 8) && cnt_start;
assign data_samp_clr        = irda_mode ? samp_cnt == 1 && cnt_start : samp_cnt == 8 && cnt_start;
assign stop_samp_clr        = data_bit_cnt == 3'd0 && samp_cnt == 1 && cnt_start;
assign start_judg           = irda_mode ? (samp_cnt == 11 && cnt == 12'd0) : (samp_cnt == 8 || samp_cnt == 11) && cnt == 12'd0;
assign data_judg            = samp_cnt == 11 && cnt == 12'd0;
assign stop_judg            = stop_end;
assign samp_val             = (irda_mode | samp_mode) ? bit1_num[0] : bit1_num[1];
// bit flag
assign bit0_noise           = (irda_mode | samp_mode) ? 1'b0 : (bit0_num[1] & bit1_num[0]); // two '0' and one '1'
assign bit1_noise           = (irda_mode | samp_mode) ? 1'b0 : (bit1_num[1] & bit0_num[0]); // two '1' and one '0'
assign bit_noise            = (irda_mode | samp_mode) ? 1'b0 : ((|bit0_num) & (|bit1_num));
assign bit0_err             = (irda_mode | samp_mode) ? 1'b0 : bit1_num[1]; // more than two '1'
assign bit1_err             = (irda_mode | samp_mode) ? 1'b0 : bit0_num[1]; // more than two '0'
assign parity_err           = parity_en & (^{parity_type, parity_buf, data_buf});
// err flag
assign start_fail           = start_judg && bit0_err;
assign start_noise_p        = (st_curr == RX_START)     && start_judg   && bit0_noise;
assign data_noise_p         = (st_curr == RX_DATA)      && data_judg    && bit_noise;
assign parity_noise_p       = (st_curr == RX_PARITY)    && data_judg    && bit_noise;
assign stop_noise_p         = (st_curr == RX_STOP)      && stop_judg    && bit1_noise;
assign parity_err_p         = (st_curr == RX_PARITY)    && data_judg    && parity_err;
assign stop_err_p           = (st_curr == RX_STOP)      && stop_judg    && bit1_err;
// dout
assign rx_vld_p             = st_curr == RX_DATA && st_next != RX_DATA;
assign rx_byte              = data_buf;
assign rx_parity            = parity_buf;
// state
assign rx_state[7:3]        = 5'h0; // rfu
assign rx_state[2:0]        = st_curr[2:0];

// stop sel
always @(*)
    case (stop_len)
        2'd0: begin // 1-bit
            stop_end_sel    = cnt_end && samp_cnt == 12;
            if (samp_mode)  stop_samp_en = cnt_end && samp_cnt == 9;
            else            stop_samp_en = cnt_end && (samp_cnt == 8 || samp_cnt == 9 || samp_cnt == 10);
        end
        2'd1: begin // 0.5 bit
            stop_end_sel    = cnt_end && samp_cnt == 5;
            if (samp_mode)  stop_samp_en = cnt_end && samp_cnt == 3;
            else            stop_samp_en = cnt_end && (samp_cnt == 2 || samp_cnt == 3 || samp_cnt == 4);
        end
        2'd2: begin // 2-bit
            stop_end_sel    = cnt_end && samp_cnt == 12 && data_bit_cnt == 3'd1;
            if (samp_mode)  stop_samp_en = cnt_end && samp_cnt == 13;
            else            stop_samp_en = cnt_end && (samp_cnt == 13 || samp_cnt == 14 || samp_cnt == 15);
        end
        default: begin
            stop_end_sel    = cnt_end && samp_cnt ==  5 && data_bit_cnt == 3'd1;
            if (samp_mode)  stop_samp_en = cnt_end && samp_cnt == 11;
            else            stop_samp_en = cnt_end && (samp_cnt == 10 || samp_cnt == 11 || samp_cnt == 12);
        end
    endcase

// fsm: sync
always @(posedge clk or negedge rstn)
    if (~rstn)
        st_curr <= RX_IDLE;
    else if (~rx_en)
        st_curr <= RX_IDLE;
    else
        st_curr <= st_next;

// fsm: comb
always @(*) begin
    // init
    st_next = st_curr;
    // trans
    case (st_curr)
        RX_IDLE: begin
            if (~rx_din)
                st_next = RX_START;
        end
        RX_START: begin
            if (start_fail)
                st_next = RX_IDLE;
            else if (bit_end)
                st_next = RX_DATA;
        end
        RX_DATA: begin
            if (bit_end & byte_end)
                if (parity_en | word_len)
                    st_next = RX_PARITY;
                else
                    st_next = RX_STOP;
        end
        RX_PARITY: begin
            if (bit_end)
                st_next = RX_STOP;
        end
        RX_STOP: begin
            if (stop_end)
                st_next = RX_IDLE;
        end
        default: begin
            st_next = RX_IDLE;
        end
    endcase
end

// cnt & samp_cnt
always @(posedge clk or negedge rstn)
    if (~rstn) begin
        cnt <= 0;
        samp_cnt <= 0;
    end
    else if (st_curr == RX_IDLE && st_next == RX_START) begin
        cnt <= 0;
        samp_cnt <= 0;
    end
    else if (st_curr == RX_START || st_curr == RX_DATA || st_curr == RX_PARITY || st_curr == RX_STOP) begin
        if (cnt == cnt_max) begin
            cnt <= 0;
            if (samp_cnt == 5'd16)
                samp_cnt <= 5'd0;
            else if (samp_cnt == 5'd15 && baud_rate[3:0] == 4'h0)
                samp_cnt <= 5'd0;
            else
                samp_cnt <= samp_cnt + 1;
        end
        else begin
            cnt <= cnt + 1;
        end
    end

// bit0_num, bit1_num
always @(posedge clk or negedge rstn)
    if (~rstn) begin
        bit0_num <= 0;
        bit1_num <= 0;
    end
    else if (st_curr == RX_START) begin
        if (start_samp_clr) begin
            bit0_num <= 0;
            bit1_num <= 0;
        end
        else if (cnt_end && (samp_point_pre || samp_point_mid)) begin
            bit0_num <= bit0_num + {1'b0, ~rx_din};
            bit1_num <= bit1_num + {1'b0, rx_din};
        end
    end
    else if (st_curr == RX_DATA || st_next == RX_PARITY) begin
        if (data_samp_clr) begin
            bit0_num <= 0;
            bit1_num <= 0;
        end
        else if (cnt_end && samp_point_mid) begin
            bit0_num <= bit0_num + {1'b0, ~rx_din};
            bit1_num <= bit1_num + {1'b0, rx_din};
        end
    end
    else if (st_curr == RX_STOP) begin
        if (stop_samp_clr) begin
            bit0_num <= 0;
            bit1_num <= 0;
        end
        else if (stop_samp_en) begin
            bit0_num <= bit0_num + {1'b0, ~rx_din};
            bit1_num <= bit1_num + {1'b0, rx_din};
        end
    end

// data_bit_cnt
always @(posedge clk or negedge rstn)
    if (~rstn) begin
        data_bit_cnt <= 0;
    end
    else if (st_curr == RX_START && st_next != RX_START) begin
        data_bit_cnt <= 0;
    end
    else if ((st_curr == RX_DATA || st_curr == RX_STOP) && bit_end) begin
        if (byte_end)
            data_bit_cnt <= 0;
        else
            data_bit_cnt <= data_bit_cnt + 1;
    end

// data_buf
always @(posedge clk)
    if (st_curr == RX_START && st_next != RX_START)
        data_buf <= 0;
    else if (st_curr == RX_DATA && data_judg)
        data_buf[data_bit_cnt] <= samp_val;

// parity_buf
always @(posedge clk)
    if (st_curr == RX_START && st_next != RX_START)
        parity_buf <= 0;
    else if (st_curr == RX_PARITY && data_judg)
        parity_buf <= samp_val;

endmodule

