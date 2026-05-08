setenv LMC_TIMEUNIT -9

# Clean old compiled library
if {[file exists work]} {
    vdel -lib work -all
}

vlib work
vmap work work

# Compile package first
vlog -sv -work work ../rtl/drone_pkg.sv

# Compile synthesizable RTL
vlog -sv -work work ../rtl/drone_controller.sv
vlog -sv -work work ../rtl/drone_dynamics_update.sv
vlog -sv -work work ../rtl/drone_closed_loop.sv

# Compile simulation/formal monitor
vlog -sv -work work ../rtl/drone_safety_monitor.sv

# Compile testbench last
vlog -sv -work work ../tb/tb_drone_closed_loop.sv

# Run simulation
vsim -classdebug -voptargs="+acc" +notimingchecks -L work work.tb_drone_closed_loop -wlf drone.wlf +RUN_NEGATIVE_TESTS

# Waves
add wave -noupdate -group TB -radix hexadecimal /tb_drone_closed_loop/*
add wave -noupdate -group DUT -radix hexadecimal /tb_drone_closed_loop/dut/*
add wave -noupdate -group MONITOR -radix hexadecimal /tb_drone_closed_loop/monitor/*

run 10000ns
