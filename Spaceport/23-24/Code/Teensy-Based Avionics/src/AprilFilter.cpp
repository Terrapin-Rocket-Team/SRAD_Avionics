#include "AprilFilter.h"

Matrix get_F(int dt){
  double* result = new double[36]{1, 0, 0, dt, 0, 0,
      0, 1, 0, 0, dt, 0,
      0, 0, 1, 0, 0, dt,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1};
  return Matrix(6, 6, result);
}

Matrix get_G(int dt){
  double* result = new double[18]{0.5 * dt * dt, 0, 0,
      0, 0.5 * dt * dt, 0,
      0, 0, 0.5 * dt * dt,
      dt, 0, 0,
      0, dt, 0,
      0, 0, dt};
  return Matrix(6, 3, result);
}

Matrix get_H(int has_gps, int has_barometer){
  int sum = (has_barometer + has_gps) == 0 ? 1 : (has_barometer + has_gps);
  double* result = new double[24]{1.0 * has_gps, 0, 0, 0, 0, 0,
      0, 1.0 * has_gps, 0, 0, 0, 0,
      0, 0, 1.0 / sum * has_gps, 0, 0, 0,
      0, 0, 1.0 / sum * has_barometer, 0, 0, 0};
  return Matrix(3, 6, result);
}

LinearKalmanFilter *initializeFilter(){
  double* x = new double[6] {0, 0, 0, 0, 0, 0};
  Matrix* X = new Matrix(6, 1, x);
  double* u = new double[3] {0, 0, 0};
  Matrix* U = new Matrix(3, 1, u);
  double *p = new double[36]{1, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 1,
      1, 0, 0, 1, 0, 0,
      0, 1, 0, 0, 1, 0,
      0, 0, 1, 0, 0, 1};
  Matrix* P = new Matrix(6, 6, p);
  double *r = new double[16] {1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1};
  Matrix* R = new Matrix(4, 4, r);
  
  return new LinearKalmanFilter(*X, *U, *P, get_F(0), get_G(0), *R);
}

double* iterateFilter(LinearKalmanFilter kf, int dt, double* input, double* measurement, int has_gps, int has_barometer){
  Matrix meas = Matrix(4, 1, measurement);
  Matrix inp = Matrix(3, 1, input);
  Matrix state = kf.iterate(meas, inp, get_F(dt), get_G(dt), get_H(has_gps, has_barometer));
  double* ret = new double[6];
  double* st = state.getArr();
  for(int i = 0; i < 6; ++i){
    ret[i] = st[i];
  }
  return ret;
}