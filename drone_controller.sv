`timescale 1ns/1ps
import drone_pkg::*;

module drone_controller #(
  parameter int COORD_W = drone_pkg::COORD_W,
  parameter int VEL_W   = drone_pkg::VEL_W,
  parameter int CTRL_W  = drone_pkg::CTRL_W,
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
  parameter int U3_MAX  = drone_pkg::U3_MAX
) (
  input  logic signed [COORD_W-1:0] q1,
  input  logic signed [COORD_W-1:0] q2,
  input  logic signed [COORD_W-1:0] q3,
  input  logic signed [VEL_W-1:0]   v1,
  input  logic signed [VEL_W-1:0]   v2,
  input  logic signed [VEL_W-1:0]   v3,
  output logic signed [CTRL_W-1:0]  u1,
  output logic signed [CTRL_W-1:0]  u2,
  output logic signed [CTRL_W-1:0]  u3
);

  // Guard-band controller with zero-velocity recovery.
  // Corrects when the drone is moving outward OR stalled in the guard band.
  // Does not keep pushing if the drone is already moving inward.

  always_comb begin
    u1 = '0;

    if ((q1 <= $signed(Q1_MIN + DH)) && (v1 <= 0)) begin
      u1 = $signed(U_H);
    end else if ((q1 >= $signed(Q1_MAX - DH)) && (v1 >= 0)) begin
      u1 = -$signed(U_H);
    end
  end

  always_comb begin
    u2 = '0;

    if ((q2 <= $signed(Q2_MIN + DH)) && (v2 <= 0)) begin
      u2 = $signed(U_H);
    end else if ((q2 >= $signed(Q2_MAX - DH)) && (v2 >= 0)) begin
      u2 = -$signed(U_H);
    end
  end

  always_comb begin
    u3 = $signed(U_HOV);

    if ((q3 <= $signed(Q3_MIN + D3_LOW)) && (v3 <= 0)) begin
      u3 = $signed(U3_MAX);
    end else if ((q3 >= $signed(Q3_MAX - D3_HIGH)) && (v3 >= 0)) begin
      u3 = $signed(U3_MIN);
    end
  end

endmodule
