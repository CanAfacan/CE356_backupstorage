package drone_pkg;
  // Fixed-point convention from the project writeup:
  // F = 32, so real_value = fixed_value / 32.
  // Sampling period dt = 1/4, giving exact integer updates:
  //   v_next = v + (u - F*e3)/4
  //   q_next = q + v/4 + (u - F*e3)/32
  parameter int COORD_W = 16;
  parameter int VEL_W   = 16;
  parameter int CTRL_W  = 16;
  parameter int ACC_W   = 32;

  parameter int SCALE = 32;

  // Baseline fixed-point geofence: q1,q2 in [-32,32], q3 in [0,64].
  parameter int Q1_MIN = -32;
  parameter int Q1_MAX =  32;
  parameter int Q2_MIN = -32;
  parameter int Q2_MAX =  32;
  parameter int Q3_MIN =   0;
  parameter int Q3_MAX =  64;

  // Guard band widths: delta = 8 fixed-point units = 1/4 real unit.
  parameter int DH      = 8;
  parameter int D3_LOW  = 8;
  parameter int D3_HIGH = 8;

  // Fixed-point control alphabet values.
  parameter int U_H    = 32;  // horizontal +/- 1.0
  parameter int U_HOV  = 32;  // vertical hover 1.0
  parameter int U3_MIN = 0;   // vertical downward corrective thrust 0.0
  parameter int U3_MAX = 64;  // vertical upward corrective thrust 2.0

  // Velocity finite modeling domain. Tune to match a specific model sweep.
  parameter int V_MIN = -128;
  parameter int V_MAX =  128;

  // Default speed limit in fixed-point-squared units.
  // Example: VMAX = 64 fixed-point units = 2.0 real units, so VMAX_SQ = 4096.
  parameter longint unsigned VMAX_SQ = 64'd4096;

  // Bounded simulation/formal approximation of the unbounded recovery liveness property.
  parameter int RECOVERY_BOUND = 16;
endpackage

