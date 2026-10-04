Vendored copy of https://github.com/boschsensortec/BMI270_SensorAPI (bmi2.c, bmi2.h, bmi2_defs.h,
bmi270.c, bmi270.h), BSD-3-Clause, see LICENSE. Only chip initialisation (including the 8 KiB
configuration blob that the BMI270 needs after every power-up) and raw register access are used;
the FIFO is parsed by components/imu_fifo.
