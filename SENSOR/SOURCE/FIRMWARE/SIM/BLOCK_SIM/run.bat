@echo off
REM DELETE UNNECESSARRY FILES 
REM rd is specified for deleting directories and erase adn del for files
rd /Q /S log_files
rd /Q /S .Xil
rd /Q /S xsim.dir
rd /Q /S xsim.jou
rd /Q /S xvlog.pb
rd /Q /S xelab.pb
rd /Q /S vivado*
rd /Q /S snapshot*
REM ADDING XLINX FEATURES
call C:\Xilinx\Vivado\2023.2\settings64.bat

mkdir log_files
vivado -mode tcl -nojournal -source ./param.tcl -log ./log_files/vivado_log.log 