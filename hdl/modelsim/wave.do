onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_CLK
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_MOSI
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_MISO
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_CSN
add wave -noupdate -radix unsigned {/tb_FLIGHT_CTRL_TOP/spi_data[0]}
add wave -noupdate -radix unsigned {/tb_FLIGHT_CTRL_TOP/spi_data[1]}
add wave -noupdate -radix unsigned {/tb_FLIGHT_CTRL_TOP/spi_data[2]}
add wave -noupdate -radix unsigned {/tb_FLIGHT_CTRL_TOP/spi_data[3]}
add wave -noupdate -radix unsigned {/tb_FLIGHT_CTRL_TOP/spi_data[4]}
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/bit_cnt
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/byte_cnt
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_in
add wave -noupdate -radix unsigned -childformat {{{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[7]} -radix unsigned} {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[6]} -radix unsigned} {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[5]} -radix unsigned} {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[4]} -radix unsigned} {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[3]} -radix unsigned} {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[2]} -radix unsigned} {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[1]} -radix unsigned} {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[0]} -radix unsigned}} -subitemconfig {{/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[7]} {-height 15 -radix unsigned} {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[6]} {-height 15 -radix unsigned} {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[5]} {-height 15 -radix unsigned} {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[4]} {-height 15 -radix unsigned} {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[3]} {-height 15 -radix unsigned} {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[2]} {-height 15 -radix unsigned} {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[1]} {-height 15 -radix unsigned} {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out[0]} {-height 15 -radix unsigned}} /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/sr_out
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/rnw
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/ADDR
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/REN
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/RD
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/WEN
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/WD
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/SPI_SLAVE_inst/AUTO_INC_EN
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/CLK_DIV
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/DEVICE_ADDR
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/REG_ADDR
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/WRITE_DATA
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/READ_SIZE
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/READ_DATA
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/READ_VALID
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/READ_EN
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/WRITE_EN
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/NACK
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/BUSY
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/SCL
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/SDA_OUT
add wave -noupdate /tb_FLIGHT_CTRL_TOP/sda_in
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/SDA_IN
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/SDA_DIR
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/state
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/next_state
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/f_cnt
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/c_cnt
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/b_cnt
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/c_pulse
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/b_pulse
add wave -noupdate {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/c_pulse_dl[3]}
add wave -noupdate {/tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/b_pulse_dl[1]}
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/bytes
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/sr_in
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/I2C_MASTER_inst/sr_out
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/MPU_INIT_inst/I2C_ADDR
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/MPU_INIT_inst/I2C_WRITE_DATA
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/MPU_INIT_inst/I2C_WRITE_EN
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/MPU_INIT_inst/DONE
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/ICU_INT
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/GYRO_X
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/GYRO_Y
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/GYRO_Z
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/ACCEL_X
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/ACCEL_Y
add wave -noupdate -radix unsigned /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/ACCEL_Z
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/GYRO_X_VALID
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/GYRO_Y_VALID
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/GYRO_Z_VALID
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/ACCEL_X_VALID
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/ACCEL_Y_VALID
add wave -noupdate /tb_FLIGHT_CTRL_TOP/FLIGHT_CTRL_TOP_inst/COLLECT_SENSOR_inst/ACCEL_Z_VALID
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {114867901 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 195
configure wave -valuecolwidth 74
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ps
update
WaveRestoreZoom {110295403 ps} {126701653 ps}
