#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>
#include <stdio.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28
#define WHEEL_RADIUS 0.0205
#define WINDOW_SIZE 2 
#define UMBRAL_SEGURIDAD 0.14 

double ps_to_meters(double sensor_value) {
  if (sensor_value < 72) return 0.30; 
  return 10.0 / sensor_value; 
}

int main(int argc, char **argv) {
  wb_robot_init();

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);

  WbDeviceTag ds_fl = wb_robot_get_device("ps7");
  WbDeviceTag ds_fr = wb_robot_get_device("ps0");
  WbDeviceTag ds_l = wb_robot_get_device("ps5");
  WbDeviceTag ds_r = wb_robot_get_device("ps2");
  wb_distance_sensor_enable(ds_fl, TIME_STEP);
  wb_distance_sensor_enable(ds_fr, TIME_STEP);
  wb_distance_sensor_enable(ds_l, TIME_STEP);
  wb_distance_sensor_enable(ds_r, TIME_STEP);

  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_encoder = wb_robot_get_device("right wheel sensor");
  wb_position_sensor_enable(left_encoder, TIME_STEP);
  wb_position_sensor_enable(right_encoder, TIME_STEP);

  double prev_left_enc = 0.0;
  double prev_right_enc = 0.0;
  
  double d_estimated = 0.3;
  double P = 0.1;
  double Q = 0.02;
  double R = 0.01;
  
  double history_front[WINDOW_SIZE] = {0.0};
  int history_index = 0;
  int history_count = 0;

  int evasive_maneuver_timer = 0;
  double current_left_speed = 0.0;
  double current_right_speed = 0.0;

  while (wb_robot_step(TIME_STEP) != -1) {
    
    double raw_fl = wb_distance_sensor_get_value(ds_fl);
    double raw_fr = wb_distance_sensor_get_value(ds_fr);
    double raw_l = wb_distance_sensor_get_value(ds_l);
    double raw_r = wb_distance_sensor_get_value(ds_r);
    
    double max_raw_front = (raw_fl > raw_fr) ? raw_fl : raw_fr;
    double z_raw = ps_to_meters(max_raw_front);
    
    history_front[history_index] = z_raw;
    history_index = (history_index + 1) % WINDOW_SIZE;
    if (history_count < WINDOW_SIZE) history_count++;
    
    double sum = 0.0;
    for (int i = 0; i < history_count; i++) {
      sum += history_front[i];
    }
    double z_filtered = sum / history_count;
    
    double curr_left_enc = wb_position_sensor_get_value(left_encoder);
    double curr_right_enc = wb_position_sensor_get_value(right_encoder);
    
    double d_theta_l = curr_left_enc - prev_left_enc;
    double d_theta_r = curr_right_enc - prev_right_enc;
    double ds_l = WHEEL_RADIUS * d_theta_l;
    double ds_r = WHEEL_RADIUS * d_theta_r;
    double delta_s = (ds_l + ds_r) / 2.0;
    
    prev_left_enc = curr_left_enc;
    prev_right_enc = curr_right_enc;
    
    // Filtro de Kalman reactivo
    double d_pred = d_estimated - delta_s;
    double P_pred = P + Q;
    
    double K = P_pred / (P_pred + R);
    d_estimated = d_pred + K * (z_filtered - d_pred);
    P = (1.0 - K) * P_pred;
    
    if (d_estimated < 0.0) d_estimated = 0.0;
    
    printf("Crudo: %.3fm | Filtrado: %.3fm | Kalman: %.3fm\n", z_raw, z_filtered, d_estimated);
    
    // Lógica de Navegación
    if (evasive_maneuver_timer > 0) {
        evasive_maneuver_timer--;
    } else {
        if (d_estimated > UMBRAL_SEGURIDAD) {
          current_left_speed = MAX_SPEED * 0.4;
          current_right_speed = MAX_SPEED * 0.4;
        } else {
          evasive_maneuver_timer = 15; 
          
          if (raw_l > raw_r) { 
            current_left_speed = MAX_SPEED * 0.5;
            current_right_speed = -MAX_SPEED * 0.5;
          } else {             
            current_left_speed = -MAX_SPEED * 0.5;
            current_right_speed = MAX_SPEED * 0.5;
          }
        }
    }
    
    wb_motor_set_velocity(left_motor, current_left_speed);
    wb_motor_set_velocity(right_motor, current_right_speed);
  }

  wb_robot_cleanup();
  return 0;
}
