`timescale 1ns/1ps
import drone_pkg::*;

module tb_drone_closed_loop;
  localparam int RANDOM_TRIALS_DEFAULT = 100;
  localparam int RANDOM_STEPS_DEFAULT  = 32;

  logic clk;
  logic rst_n;
  logic step_en;
  logic init_load;

  logic signed [COORD_W-1:0] init_q1, init_q2, init_q3;
  logic signed [VEL_W-1:0]   init_v1, init_v2, init_v3;
  logic signed [COORD_W-1:0] q1, q2, q3;
  logic signed [VEL_W-1:0]   v1, v2, v3;
  logic signed [CTRL_W-1:0]  u1, u2, u3;
  logic overflow, candidate_overflow;
  logic geofence_ok, speed_ok, inner_safe, boundary_zone, recovery_ok, safety_ok;

  int errors;
  int random_trials;
  int random_steps;
  int seed;

  drone_closed_loop dut (
    .clk(clk), .rst_n(rst_n), .step_en(step_en), .init_load(init_load),
    .init_q1(init_q1), .init_q2(init_q2), .init_q3(init_q3),
    .init_v1(init_v1), .init_v2(init_v2), .init_v3(init_v3),
    .q1(q1), .q2(q2), .q3(q3), .v1(v1), .v2(v2), .v3(v3),
    .u1(u1), .u2(u2), .u3(u3),
    .overflow(overflow), .candidate_overflow(candidate_overflow)
  );

  drone_safety_monitor monitor (
    .clk(clk), .rst_n(rst_n), .valid(1'b1), .overflow(overflow),
    .q1(q1), .q2(q2), .q3(q3), .v1(v1), .v2(v2), .v3(v3),
    .geofence_ok(geofence_ok), .speed_ok(speed_ok), .inner_safe(inner_safe),
    .boundary_zone(boundary_zone), .recovery_ok(recovery_ok), .safety_ok(safety_ok)
  );

  always #5 clk = ~clk;

  task automatic reset_dut();
    begin
      clk = 1'b0;
      rst_n = 1'b0;
      step_en = 1'b0;
      init_load = 1'b0;
      init_q1 = '0; init_q2 = '0; init_q3 = 32;
      init_v1 = '0; init_v2 = '0; init_v3 = '0;
      repeat (3) @(posedge clk);
      rst_n = 1'b1;
      @(posedge clk);
    end
  endtask

  task automatic load_state(
    input int q1_i, input int q2_i, input int q3_i,
    input int v1_i, input int v2_i, input int v3_i
  );
    begin
      @(negedge clk);
      init_q1 = q1_i[COORD_W-1:0];
      init_q2 = q2_i[COORD_W-1:0];
      init_q3 = q3_i[COORD_W-1:0];
      init_v1 = v1_i[VEL_W-1:0];
      init_v2 = v2_i[VEL_W-1:0];
      init_v3 = v3_i[VEL_W-1:0];
      init_load = 1'b1;
      @(negedge clk);
      init_load = 1'b0;
      #1;
    end
  endtask

  task automatic step_once();
    begin
      @(negedge clk);
      step_en = 1'b1;
      @(negedge clk);
      step_en = 1'b0;
      #1;
    end
  endtask

  task automatic check(input bit cond, input string msg);
    begin
      if (!cond) begin
        errors++;
        $error("CHECK FAILED: %s", msg);
      end
    end
  endtask

  task automatic run_center_hover_test();
    begin
      $display("\n[DIRECTED] center hover");
      load_state(0, 0, 32, 0, 0, 0);
      repeat (8) step_once();
      check(q1 == 0 && q2 == 0 && q3 == 32, "hover should preserve position");
      check(v1 == 0 && v2 == 0 && v3 == 0, "hover should preserve velocity");
      check(!overflow && geofence_ok && speed_ok, "center hover should remain safe");
    end
  endtask

  task automatic run_controller_threshold_test();
    begin
      $display("\n[DIRECTED] controller thresholds");
      load_state(Q1_MIN + DH, 0, 32, -8, 0, 0);
      check(u1 == U_H, "lower q1 guard moving outward should command +U_H");

      load_state(Q1_MAX - DH, 0, 32, 8, 0, 0);
      check(u1 == -U_H, "upper q1 guard moving outward should command -U_H");

      load_state(0, 0, Q3_MIN + D3_LOW, 0, 0, -8);
      check(u3 == U3_MAX, "lower q3 guard moving downward should command U3_MAX");

      load_state(0, 0, Q3_MAX - D3_HIGH, 0, 0, 8);
      check(u3 == U3_MIN, "upper q3 guard moving upward should command U3_MIN");
    end
  endtask

  task automatic run_inward_recovery_test();
    int i;
    begin
      $display("\n[DIRECTED] inward recovery from guard region");
      load_state(Q1_MIN + 4, 0, 32, 8, 0, 0);
      for (i = 0; i < RECOVERY_BOUND; i++) begin
        if (inner_safe) break;
        step_once();
      end
      check(inner_safe, "state should reach inner safe set when already moving inward");
      check(!overflow && geofence_ok && speed_ok, "inward recovery should not overflow or violate safety");
    end
  endtask

  task automatic run_outward_braking_negative_test();
    int i;
    begin
      $display("\n[NEGATIVE] outward braking can stall in guard region");
      load_state(Q1_MIN + 4, 0, 32, -8, 0, 0);
      for (i = 0; i < RECOVERY_BOUND + 4; i++) begin
        step_once();
      end
      check(!recovery_ok || overflow || !inner_safe,
            "negative test expected bounded recovery not to be established");
    end
  endtask

  function automatic int rand_int(input int lo, input int hi);
    int span;
    begin
      span = hi - lo + 1;
      rand_int = lo + int'($urandom_range(span - 1));
    end
  endfunction

  function automatic int rand_vel_multiple_of_8(input int lo_mul, input int hi_mul);
    begin
      rand_vel_multiple_of_8 = 8 * rand_int(lo_mul, hi_mul);
    end
  endfunction

  task automatic run_randomized_tests();
    int t, s;
    int start_errors;
    begin
      $display("\n[RANDOM] constrained randomized tests: trials=%0d steps=%0d seed=%0d",
               random_trials, random_steps, seed);
      start_errors = errors;
      for (t = 0; t < random_trials; t++) begin
        // Start in the inner safe box with lattice-compatible velocities.
        load_state(
          rand_int(Q1_MIN + DH, Q1_MAX - DH),
          rand_int(Q2_MIN + DH, Q2_MAX - DH),
          rand_int(Q3_MIN + D3_LOW, Q3_MAX - D3_HIGH),
          rand_vel_multiple_of_8(-1, 1),
          rand_vel_multiple_of_8(-1, 1),
          rand_vel_multiple_of_8(-1, 1)
        );

        for (s = 0; s < random_steps; s++) begin
          step_once();
          if (!geofence_ok || !speed_ok || overflow) begin
            errors++;
            $display("  random failure: trial=%0d step=%0d q=(%0d,%0d,%0d) v=(%0d,%0d,%0d) overflow=%0b geo=%0b speed=%0b recovery=%0b",
                     t, s, q1, q2, q3, v1, v2, v3, overflow, geofence_ok, speed_ok, recovery_ok);
            break;
          end
        end
      end
      $display("[RANDOM] new failures recorded by TB checks: %0d", errors - start_errors);
    end
  endtask

  initial begin
    errors = 0;
    random_trials = RANDOM_TRIALS_DEFAULT;
    random_steps  = RANDOM_STEPS_DEFAULT;
    seed = 32'hCE356;

    void'($value$plusargs("RANDOM_TRIALS=%d", random_trials));
    void'($value$plusargs("RANDOM_STEPS=%d", random_steps));
    void'($value$plusargs("SEED=%d", seed));
    void'($urandom(seed));

    reset_dut();
    run_center_hover_test();
    run_controller_threshold_test();
    run_inward_recovery_test();

    if ($test$plusargs("RUN_NEGATIVE_TESTS")) begin
      run_outward_braking_negative_test();
    end

    run_randomized_tests();

    $display("\nSimulation completed with %0d TB check error(s).", errors);
    if (errors == 0) begin
      $display("TB_STATUS: PASS");
    end else begin
      $display("TB_STATUS: FAIL");
    end
    $finish;
  end

endmodule

