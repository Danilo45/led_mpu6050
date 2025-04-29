/*
 * kalman_filter.h
 *
 *  Created on: Apr 26, 2025
 *      Author: danilo
 */

#ifndef SRC_KALMAN_FILTER_H_
#define SRC_KALMAN_FILTER_H_


typedef struct {
    float q_angle;
    float r_measure;
    float angle;
    float p[2][2];
} KalmanFilter;

void kalman_filter_init(KalmanFilter* kf) ;
float kalman_filter_get_angle(KalmanFilter* kf, float new_angle, float dt);

#endif /* SRC_KALMAN_FILTER_H_ */
