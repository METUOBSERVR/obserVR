# FILE INCLUDING 
source file.tcl 

# FILE MANIPULATION
set SOURCE_VERILOG_FILES_STR        [join $SOURCE_VERILOG_FILES " "]
set SOURCE_IP_SIM_NETLIST_FILE_STR  [join $SOURCE_IP_SIM_NETLIST_FILE " "]
set SOURCE_INCLUDE_FILES_STR        [join $SOURCE_INCLUDE_FILES " "]


puts "---"
puts "Compiling verilog files with xvlog commnad"
puts "---"
puts "xvlog commnad running"
puts "---"
set cmd_xvlog "xvlog --incr --relax --sv --log ./log_files/xvlog_log.log -i $SOURCE_INCLUDE_FILES_STR --work ./library_generated $SOURCE_VERILOG_GLOBAL_FILE $SOURCE_TOP_TESTBENCH_FILE $SOURCE_TOP_FILE $SOURCE_VERILOG_FILES_STR $SOURCE_IP_SIM_NETLIST_FILE_STR"
eval exec $cmd_xvlog
puts "---"
puts "xvlog command done"
puts "---"

puts "---"
puts "Eloborated design generation with xelab commnad"
puts "---"
puts "xelab commnad running"
puts "---"
set cmd_xelab "xelab --incr --debug typical --relax -log ./log_files/xelab_log.log -i $SOURCE_INCLUDE_FILES_STR -L xil_defaultlib -L unisims_ver -L unimacro_ver -L secureip -L xpm --snapshot $SNAPSHOT_ROOT library_generated.tb_i2c library_generated.glbl"
eval exec $cmd_xelab
puts "---"
puts "xelab command done"
puts "---"

puts "---"
puts "Simulation of the eloborated design with xsim commnad"
puts "---"
puts "xsim command running"
set cmd_xsim "xsim $SNAPSHOT_ROOT -log ./log_files/xsim_log.log -gui -tclbatch wave_config.tcl"
eval exec $cmd_xsim
puts "---"
puts "xsim command done"
puts "---"










