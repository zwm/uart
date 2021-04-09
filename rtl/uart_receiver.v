
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
wire cnt_end, bit_end, byte_end; reg stop_end;

// cnt
assign data_bit_max         = ~word_len & parity_en ? 3'd6 : 3'd7; // 9-bit mode, 9th receive in RX_PARITY state
assign cnt_max              = samp_cnt[4] ? ({8'h0, baud_rate[3:0]} - 12'd1) : (baud_rate[15:4] - 12'd1);
assign cnt_end              = cnt == cnt_max;
assign bit_end              = cnt_end && (samp_cnt == 5'd15);
assign byte_end             = data_bit_cnt == data_bit_max;
// samp point
assign start_samp           = (samp_cnt == 8 || samp_cnt == 11) && cnt == 12'd0;
assign data_samp            = samp_cnt == 11 && cnt == 12'd0;
assign stop_samp            = start_samp;
// error
assign start_fail           = start_samp && bit1_num[1]; // more than two '1'
assign start_noise_p        = (st_curr == RX_START) && start_samp && bit0_num[1] && bit1_num[0]; // has two '0' and one '1'
assign data_noise_p         = (st_curr == RX_DATA) && data_samp && ((|bit0_num) & (|bit1_num));
assign parity_noise_p       = (st_curr == RX_PARITY) && data_samp && ((|bit0_num) & (|bit1_num));
assign stop_noise_p         = (st_curr == RX_STOP) && stop_samp && bit1_num[1] && bit0_num[0]; // has two '1' and one '0'
assign parity_err_p         = (st_curr == RX_PARITY) && parity_en && bit_end && (^{parity_type, parity_buf, data_buf});
assign stop_err_p           = (st_curr == RX_STOP) && stop_samp && bit0_num[1]; // more than two '0'
// dout
assign rx_vld_p             = st_curr == RX_DATA && st_next != RX_DATA;
assign rx_byte              = data_buf;
assign rx_parity            = parity_buf;
// state
assign rx_state[7:3]        = 5'h0; // rfu
assign rx_state[2:0]        = st_curr[2:0];

// stop_end
always @(*)
    case (stop_len)
        2'd0:       stop_end = bit_end;
        2'd1:       stop_end = cnt_end && samp_cnt == 7;
        2'd2:       stop_end = bit_end && data_bit_cnt == 3'd1;
        default:    stop_end = (data_bit_cnt == 3'd1) && cnt_end && (samp_cnt == 7);
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
    else if (st_curr == RX_START || st_curr == RX_STOP) begin // sample twice
        if (cnt_end && (samp_cnt == 3 || samp_cnt == 5 || samp_cnt == 7 || samp_cnt == 8 || samp_cnt == 9 || samp_cnt == 10)) begin
            bit0_num <= bit0_num + {1'b0, ~rx_din};
            bit1_num <= bit1_num + {1'b0, rx_din};
        end
        else if (cnt == 12'd0 && (samp_cnt == 1 || samp_cnt == 8)) begin
            bit0_num <= 0;
            bit1_num <= 0;
        end
    end
    else if (st_curr == RX_DATA || st_next == RX_PARITY) begin
        if (cnt_end && (samp_cnt == 8 || samp_cnt == 9 || samp_cnt == 10)) begin
            bit0_num <= bit0_num + {1'b0, ~rx_din};
            bit1_num <= bit1_num + {1'b0, rx_din};
        end
        else if (cnt == 12'd0 && samp_cnt == 8) begin
            bit0_num <= 0;
            bit1_num <= 0;
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
    else if (st_curr == RX_DATA && data_samp)
        data_buf[data_bit_cnt] <= bit1_num[1];

// parity_buf
always @(posedge clk)
    if (st_curr == RX_START && st_next != RX_START)
        parity_buf <= 0;
    else if (st_curr == RX_PARITY && data_samp)
        parity_buf <= bit1_num[1];

endmodule

