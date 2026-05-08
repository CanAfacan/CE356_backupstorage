`timescale 1ns/1ps
import drone_pkg::*;

module drone_closed_loop #(
  parameter int COORD_W = drone_pkg::COORD_W,
  parameter int VEL_W   = drone_pkg::VEL_W,
  parameter int CTRL_W  = drone_pkg::CTRL_W,
  parameter int ACC_W   = drone_pkg::ACC_W,
  parameter int SCALE   = drone_pkg::SCALE,
  parameter int Q1_MIN  = drone_pkg::Q1_MIN,
  parameter int Q1_MAX  = drone_pkg::Q1_MAX,
  parameter int Q2_MIN  = drone_pkg::Q2_MIN,
  parameter int Q2_MAX  = drone_pkg::Q2_MAX,
  parameter int Q3_MIN  = drone_pkg::Q3_MIN,
  parameter int Q3_MAX  = drone_pkg::Q3_MAX,
  parameter int DH      = drone_pkg::DH,
  parameter int D3_LOW  = drone_pkg::D3_LOW,
  parameter int D3_HIGH = drone_pkg::D3_HIGH,
  parameter int U_H     = drone_pkg::U_H,
  parameter int U_HOV   = drone_pkg::U_HOV,
  parameter int U3_MIN  = drone_pkg::U3_MIN,
  parameter int U3_MAX  = drone_pkg::U3_MAX,
  parameter int V_MIN   = drone_pkg::V_MIN,
  parameter int V_MAX   = drone_pkg::V_MAX
) (
  input  logic clk,
  input  logic rst_n,
  input  logic step_en,
  input  logic init_load,
  input  logic signed [COORD_W-1:0] init_q1,
  input  logic signed [COORD_W-1:0] init_q2,
  input  logic signed [COORD_W-1:0] init_q3,
  input  logic signed [VEL_W-1:0]   init_v1,
  input  logic signed [VEL_W-1:0]   init_v2,
  input  logic signed [VEL_W-1:0]   init_v3,
  output logic signed [COORD_W-1:0] q1,
  output logic signed [COORD_W-1:0] q2,
  output logic signed [COORD_W-1:0] q3,
  output logic signed [VEL_W-1:0]   v1,
  output logic signed [VEL_W-1:0]   v2,
  output logic signed [VEL_W-1:0]   v3,
  output logic signed [CTRL_W-1:0]  u1,
  output logic signed [CTRL_W-1:0]  u2,
  output logic signed [CTRL_W-1:0]  u3,
  output logic                      overflow,
  output logic                      candidate_overflow
);

  logic signed [COORD_W-1:0] q1_next, q2_next, q3_next;
  logic signed [VEL_W-1:0]   v1_next, v2_next, v3_next;

  drone_controller #(
    .COORD_W(COORD_W), .VEL_W(VEL_W), .CTRL_W(CTRL_W),
    .Q1_MIN(Q1_MIN), .Q1_MAX(Q1_MAX), .Q2_MIN(Q2_MIN), .Q2_MAX(Q2_MAX),
    .Q3_MIN(Q3_MIN), .Q3_MAX(Q3_MAX), .DH(DH), .D3_LOW(D3_LOW), .D3_HIGH(D3_HIGH),
    .U_H(U_H), .U_HOV(U_HOV), .U3_MIN(U3_MIN), .U3_MAX(U3_MAX)
  ) u_controller (
    .q1(q1), .q2(q2), .q3(q3),
    .v1(v1), .v2(v2), .v3(v3),
    .u1(u1), .u2(u2), .u3(u3)
  );

  drone_dynamics_update #(
    .COORD_W(COORD_W), .VEL_W(VEL_W), .CTRL_W(CTRL_W), .ACC_W(ACC_W), .SCALE(SCALE),
    .Q1_MIN(Q1_MIN), .Q1_MAX(Q1_MAX), .Q2_MIN(Q2_MIN), .Q2_MAX(Q2_MAX),
    .Q3_MIN(Q3_MIN), .Q3_MAX(Q3_MAX), .V_MIN(V_MIN), .V_MAX(V_MAX)
  ) u_dynamics (
    .q1(q1), .q2(q2), .q3(q3),
    .v1(v1), .v2(v2), .v3(v3),
    .u1(u1), .u2(u2), .u3(u3),
    .q1_next(q1_next), .q2_next(q2_next), .q3_next(q3_next),
    .v1_next(v1_next), .v2_next(v2_next), .v3_next(v3_next),
    .domain_overflow(candidate_overflow)
  );

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      q1 <= '0;
      q2 <= '0;
      q3 <= $signed(32);  // midpoint altitude of [0,64]
      v1 <= '0;
      v2 <= '0;
      v3 <= '0;
      overflow <= 1'b0;
    end else if (init_load) begin
      q1 <= init_q1;
      q2 <= init_q2;
      q3 <= init_q3;
      v1 <= init_v1;
      v2 <= init_v2;
      v3 <= init_v3;
      overflow <= 1'b0;
    end else if (step_en) begin
      if (!overflow) begin
        if (candidate_overflow) begin
          // Match the project/TLC-style behavior: hold physical state and expose escape.
          overflow <= 1'b1;
        end else begin
          q1 <= q1_next;
          q2 <= q2_next;
          q3 <= q3_next;
          v1 <= v1_next;
          v2 <= v2_next;
          v3 <= v3_next;
        end
      end
    end
  end

endmodule

