`timescale 1ns/1ps
import drone_pkg::*;

module drone_safety_monitor #(
  parameter int COORD_W = drone_pkg::COORD_W,
  parameter int VEL_W   = drone_pkg::VEL_W,
  parameter int Q1_MIN  = drone_pkg::Q1_MIN,
  parameter int Q1_MAX  = drone_pkg::Q1_MAX,
  parameter int Q2_MIN  = drone_pkg::Q2_MIN,
  parameter int Q2_MAX  = drone_pkg::Q2_MAX,
  parameter int Q3_MIN  = drone_pkg::Q3_MIN,
  parameter int Q3_MAX  = drone_pkg::Q3_MAX,
  parameter int DH      = drone_pkg::DH,
  parameter int D3_LOW  = drone_pkg::D3_LOW,
  parameter int D3_HIGH = drone_pkg::D3_HIGH,
  parameter longint unsigned VMAX_SQ = drone_pkg::VMAX_SQ,
  parameter int RECOVERY_BOUND = drone_pkg::RECOVERY_BOUND
) (
  input  logic clk,
  input  logic rst_n,
  input  logic valid,
  input  logic overflow,
  input  logic signed [COORD_W-1:0] q1,
  input  logic signed [COORD_W-1:0] q2,
  input  logic signed [COORD_W-1:0] q3,
  input  logic signed [VEL_W-1:0]   v1,
  input  logic signed [VEL_W-1:0]   v2,
  input  logic signed [VEL_W-1:0]   v3,
  output logic geofence_ok,
  output logic speed_ok,
  output logic inner_safe,
  output logic boundary_zone,
  output logic recovery_ok,
  output logic safety_ok
);

  localparam int REC_CNT_W = (RECOVERY_BOUND < 2) ? 2 : $clog2(RECOVERY_BOUND + 2);

  logic recovery_active;
  logic [REC_CNT_W-1:0] recovery_timer;
  logic recovery_fail;
  logic [2*VEL_W+4:0] speed_sq;

  function automatic logic [2*VEL_W+1:0] sqr(input logic signed [VEL_W-1:0] x);
    logic signed [VEL_W:0] sx;
    begin
      sx = {x[VEL_W-1], x};
      sqr = sx * sx;
    end
  endfunction

  always_comb begin
    geofence_ok =
      (q1 >= $signed(Q1_MIN)) && (q1 <= $signed(Q1_MAX)) &&
      (q2 >= $signed(Q2_MIN)) && (q2 <= $signed(Q2_MAX)) &&
      (q3 >= $signed(Q3_MIN)) && (q3 <= $signed(Q3_MAX));

    inner_safe =
      (q1 >= $signed(Q1_MIN + DH))      && (q1 <= $signed(Q1_MAX - DH)) &&
      (q2 >= $signed(Q2_MIN + DH))      && (q2 <= $signed(Q2_MAX - DH)) &&
      (q3 >= $signed(Q3_MIN + D3_LOW))  && (q3 <= $signed(Q3_MAX - D3_HIGH));

    boundary_zone = geofence_ok && !inner_safe;

    speed_sq = sqr(v1) + sqr(v2) + sqr(v3);
    speed_ok = (speed_sq <= VMAX_SQ[2*VEL_W+4:0]);

    recovery_ok = !recovery_fail;
    safety_ok = valid && !overflow && geofence_ok && speed_ok && recovery_ok;
  end

  // Synthesizable bounded-recovery watchdog. It approximates the unbounded LTL
  // eventual-recovery property with a configurable cycle bound.
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      recovery_active <= 1'b0;
      recovery_timer  <= '0;
      recovery_fail   <= 1'b0;
    end else if (valid && !overflow) begin
      if (inner_safe) begin
        recovery_active <= 1'b0;
        recovery_timer  <= '0;
      end else if (boundary_zone) begin
        if (!recovery_active) begin
          recovery_active <= 1'b1;
          recovery_timer  <= '0;
        end else if (recovery_timer >= RECOVERY_BOUND[REC_CNT_W-1:0]) begin
          recovery_fail <= 1'b1;
        end else begin
          recovery_timer <= recovery_timer + 1'b1;
        end
      end else begin
        recovery_active <= 1'b0;
        recovery_timer  <= '0;
      end
    end
  end

`ifndef SYNTHESIS
`ifndef NO_SVA
  property p_geofence_invariant;
    @(posedge clk) disable iff (!rst_n)
      valid && !overflow |-> geofence_ok;
  endproperty

  property p_speed_invariant;
    @(posedge clk) disable iff (!rst_n)
      valid && !overflow |-> speed_ok;
  endproperty

  property p_bounded_recovery;
    @(posedge clk) disable iff (!rst_n)
      valid && !overflow && boundary_zone |-> ##[1:RECOVERY_BOUND] (inner_safe && !overflow);
  endproperty

  a_geofence_invariant: assert property (p_geofence_invariant)
    else $error("Geofence invariant failed at t=%0t: q=(%0d,%0d,%0d)", $time, q1, q2, q3);

  a_speed_invariant: assert property (p_speed_invariant)
    else $error("Speed invariant failed at t=%0t: v=(%0d,%0d,%0d), speed_sq=%0d", $time, v1, v2, v3, speed_sq);

  a_bounded_recovery: assert property (p_bounded_recovery)
    else $error("Bounded recovery failed at t=%0t: q=(%0d,%0d,%0d), v=(%0d,%0d,%0d)", $time, q1, q2, q3, v1, v2, v3);
`endif
`endif

endmodule

