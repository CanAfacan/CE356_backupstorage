`timescale 1ns/1ps
import drone_pkg::*;

module drone_dynamics_update #(
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
  parameter int V_MIN   = drone_pkg::V_MIN,
  parameter int V_MAX   = drone_pkg::V_MAX
) (
  input  logic signed [COORD_W-1:0] q1,
  input  logic signed [COORD_W-1:0] q2,
  input  logic signed [COORD_W-1:0] q3,
  input  logic signed [VEL_W-1:0]   v1,
  input  logic signed [VEL_W-1:0]   v2,
  input  logic signed [VEL_W-1:0]   v3,
  input  logic signed [CTRL_W-1:0]  u1,
  input  logic signed [CTRL_W-1:0]  u2,
  input  logic signed [CTRL_W-1:0]  u3,
  output logic signed [COORD_W-1:0] q1_next,
  output logic signed [COORD_W-1:0] q2_next,
  output logic signed [COORD_W-1:0] q3_next,
  output logic signed [VEL_W-1:0]   v1_next,
  output logic signed [VEL_W-1:0]   v2_next,
  output logic signed [VEL_W-1:0]   v3_next,
  output logic                      domain_overflow
);

  logic signed [ACC_W-1:0] q1_ext, q2_ext, q3_ext;
  logic signed [ACC_W-1:0] v1_ext, v2_ext, v3_ext;
  logic signed [ACC_W-1:0] u1_ext, u2_ext, u3_ext;
  logic signed [ACC_W-1:0] a1, a2, a3;
  logic signed [ACC_W-1:0] q1_cand, q2_cand, q3_cand;
  logic signed [ACC_W-1:0] v1_cand, v2_cand, v3_cand;

  always_comb begin
    q1_ext = {{(ACC_W-COORD_W){q1[COORD_W-1]}}, q1};
    q2_ext = {{(ACC_W-COORD_W){q2[COORD_W-1]}}, q2};
    q3_ext = {{(ACC_W-COORD_W){q3[COORD_W-1]}}, q3};

    v1_ext = {{(ACC_W-VEL_W){v1[VEL_W-1]}}, v1};
    v2_ext = {{(ACC_W-VEL_W){v2[VEL_W-1]}}, v2};
    v3_ext = {{(ACC_W-VEL_W){v3[VEL_W-1]}}, v3};

    u1_ext = {{(ACC_W-CTRL_W){u1[CTRL_W-1]}}, u1};
    u2_ext = {{(ACC_W-CTRL_W){u2[CTRL_W-1]}}, u2};
    u3_ext = {{(ACC_W-CTRL_W){u3[CTRL_W-1]}}, u3};

    // Net acceleration a = u - F*e3.
    a1 = u1_ext;
    a2 = u2_ext;
    a3 = u3_ext - $signed(SCALE);

    // Exact sampled-data update for dt = 1/4 and F = 32.
    // Arithmetic shifts are exact when controls are in Uctrl and velocities are on 8Z.
    v1_cand = v1_ext + (a1 >>> 2);
    v2_cand = v2_ext + (a2 >>> 2);
    v3_cand = v3_ext + (a3 >>> 2);

    q1_cand = q1_ext + (v1_ext >>> 2) + (a1 >>> 5);
    q2_cand = q2_ext + (v2_ext >>> 2) + (a2 >>> 5);
    q3_cand = q3_ext + (v3_ext >>> 2) + (a3 >>> 5);

    q1_next = q1_cand[COORD_W-1:0];
    q2_next = q2_cand[COORD_W-1:0];
    q3_next = q3_cand[COORD_W-1:0];
    v1_next = v1_cand[VEL_W-1:0];
    v2_next = v2_cand[VEL_W-1:0];
    v3_next = v3_cand[VEL_W-1:0];

    domain_overflow =
      (q1_cand < $signed(Q1_MIN)) || (q1_cand > $signed(Q1_MAX)) ||
      (q2_cand < $signed(Q2_MIN)) || (q2_cand > $signed(Q2_MAX)) ||
      (q3_cand < $signed(Q3_MIN)) || (q3_cand > $signed(Q3_MAX)) ||
      (v1_cand < $signed(V_MIN))  || (v1_cand > $signed(V_MAX))  ||
      (v2_cand < $signed(V_MIN))  || (v2_cand > $signed(V_MAX))  ||
      (v3_cand < $signed(V_MIN))  || (v3_cand > $signed(V_MAX));
  end

endmodule

