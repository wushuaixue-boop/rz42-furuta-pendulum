# Validation summary

All required tests passed: **True**

## Servo model on the real coupled plant (0.1 rad step)

| | rise 10-90 % [ms] | overshoot [%] | settling 2 % [ms] |
|---|---|---|---|
| ideal_model | 26.2 | 1.5 | 40 |
| actual | 28.9 | 0.3 | 53 |

RMS difference actual vs. ideal: 1.2 % of the step.

## Closed-loop tests

| test | required | result | first catch [s] | longest balance [s] | balance RMS [deg] | max arm [deg] | peak torque [N m] | switches | notes |
|---|---|---|---|---|---|---|---|---|---|
| demo | yes | PASS | 3.46 | 18.4 | 1.23 | 91.8 | 0.247 | 3 |  |
| init_hanging_offset_30deg | yes | PASS | 2.85 | 27.1 | 0.12 | 35.0 | 0.222 | 1 |  |
| init_hanging_arm_offset | yes | PASS | 2.44 | 27.4 | 0.12 | 35.0 | 0.197 | 1 |  |
| init_moving_pendulum | yes | PASS | 1.84 | 28.0 | 0.12 | 35.0 | 0.206 | 1 |  |
| init_near_upright | yes | PASS | 0.0 | 30.0 | 0.12 | 17.8 | 0.114 | 1 |  |
| init_near_upright_other_side_wrapped | yes | PASS | 0.0 | 29.8 | 0.12 | 25.0 | 0.128 | 1 |  |
| servo_bw_actual_8Hz_assumed_15Hz | no | PASS | 4.86 | 16.4 | 2.01 | 86.3 | 0.223 | 3 |  |
| servo_bw_actual_10Hz_assumed_15Hz | yes | PASS | 3.27 | 18.7 | 1.62 | 91.9 | 0.252 | 3 |  |
| servo_bw_actual_25Hz_assumed_15Hz | yes | PASS | 11.02 | 10.9 | 1.29 | 94.6 | 0.350 | 3 |  |
| servo_bw_10Hz_matched | yes | PASS | 3.27 | 18.7 | 1.53 | 94.2 | 0.241 | 3 |  |
| latency_3ms | yes | PASS | 3.28 | 18.6 | 1.26 | 93.8 | 0.276 | 3 |  |
| control_rate_100Hz | no | PASS | 5.56 | 16.4 | 1.63 | 94.1 | 0.328 | 3 |  |
| sensor_noise_seed_1 | yes | PASS | 3.47 | 18.4 | 1.27 | 91.8 | 0.247 | 3 |  |
| sensor_noise_seed_2 | yes | PASS | 3.47 | 18.5 | 1.23 | 91.9 | 0.246 | 3 |  |
| plant_mismatch_added_mass_10g | no | PASS | 3.16 | 26.7 | 0.12 | 35.2 | 0.228 | 1 |  |
