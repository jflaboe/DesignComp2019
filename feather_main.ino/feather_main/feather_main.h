#ifndef FEATHER_H_
#define FEATHER_H_

float vector_angle(float ivec,float jvec){
  float angle;
  if (jvec>0){
  angle = math.arcos(ivec);
  }
  else  if (jvec<0){
  angle = 360 - math.arcos(ivec);
  }
  return angle;
}

float dot_product(float x_1,float y_1,float x_2,float y_2){
    float cos_x;
    sin_x = x_1*x_2-y_1*y_2;
    return cos_x;
}
float cross_product(float x_1,float y_1,float x_2,float y_2){
    float sin_x;
    sin_x = x_1*y_2-x_2*y_1;
    return sin_x;
}
#endif