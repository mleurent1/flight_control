
# PlanAhead Launch Script for Post-Synthesis floorplanning, created by Project Navigator

create_project -name flight_control_mimas -dir "C:/Users/mleurent/Documents/perso/modelisme/flight_control/hdl/flight_control_mimas/planAhead_run_1" -part xc6slx9tqg144-3
set_property design_mode GateLvl [get_property srcset [current_run -impl]]
set_property edif_top_file "C:/Users/mleurent/Documents/perso/modelisme/flight_control/hdl/flight_control_mimas/FLIGHT_CTRL_TOP_FPGA_MIMAS.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {C:/Users/mleurent/Documents/perso/modelisme/flight_control/hdl/flight_control_mimas} }
set_property target_constrs_file "FLIGHT_CTRL_TOP_FPGA_MIMAS.ucf" [current_fileset -constrset]
add_files [list {FLIGHT_CTRL_TOP_FPGA_MIMAS.ucf}] -fileset [get_property constrset [current_run]]
link_design
