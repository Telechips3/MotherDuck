#pragma once
#define STEER_GAIN  1.5f

int steer_from_aruco_q15(int16 aruco_x_norm_q15,
                           uint16 aruco_dist_mm,
                           float *steer_angle_rad); 
