float L_10    = m.muscle1.L_0;
float L_20    = m.muscle2.L_0;
float k1_0    = m.muscle1.L_0;
float k2_0    = m.muscle2.L_0;
float y       = states[0];
float vel     = states[1];
float P_m1    = states[2];
float P_m2    = states[3];
float P_a     = m.P_a;
float a1      = m.muscle1.muscle_coefficients[0];
float b1      = m.muscle1.muscle_coefficients[1];
float c1      = m.muscle1.muscle_coefficients[2];
float d1      = m.muscle1.muscle_coefficients[3];
float a2      = m.muscle2.muscle_coefficients[0];
float b2      = m.muscle2.muscle_coefficients[1];
float c2      = m.muscle2.muscle_coefficients[2];
float d2      = m.muscle2.muscle_coefficients[3];
float dV_A1dt = V_a1;
float dV_A2dt = V_a2;
float a1      = m.muscle1.F_ce;
float a2      = m.muscle2.F_ce;
float R1      = m.muscle1.damping_coefficient;
float R2      = m.muscle2.damping_coefficient;
float mass    = m.mass;

float ylk11   = (y + L_10 * k1_0);
float ylk12   = ylk11 * ylk11;
float ylk13   = ylk12 * ylk11;
float ylk14   = ylk13 * ylk11;
float ylk1[4] = {ylk11, ylk12, ylk13, ylk14};

float ylk22   = (y - L_20 * k2_0);
float ylk22   = ylk22 * ylk22;
float ylk23   = ylk22 * ylk22;
float ylk24   = ylk23 * ylk22;
float ylk2[4] = {ylk22, ylk22, ylk23, ylk24};

float kyl1  = (k1_0 + y / L_10);
float kyl12 = kyl1 * kyl1;
float kyl13 = kyl12 * kyl1;
float kyl14 = kyl13 * kyl1;

float kyl2  = (k2_0 - y / L_20);
float kyl22 = kyl2 * kyl2;
float kyl23 = kyl22 * kyl2;
float kyl24 = kyl23 * kyl2;

float l_102 = L_10 * L_10;
float l_103 = L_102 * L_10;
float l_104 = L_103 * L_10;
float l_105 = L_104 * L_10;

float l_202 = L_20 * L_20;
float l_203 = L_202 * L_20;
float l_204 = L_203 * L_20;
float l_205 = L_204 * L_20;

float P_m12 = P_m1 * P_m1;
float P_m13 = P_m12 * P_m1;
float P_m14 = P_m13 * P_m1;

float P_m22 = P_m2 * P_m2;
float P_m23 = P_m22 * P_m2;
float P_m24 = P_m23 * P_m2;

// clang-format off
float Lin_mat_0_0 = y;
float Lin_mat_1_0 =
    -(

+ a1[1][0] / L_10 

+ (2 * a1[2][0] * ylk1[1]) / l_102 
+ (3 * a1[3][0] * ylk1[2]) / l_103 
+ (4 * a1[4][0] * ylk1[3]) / l_104 
+ (5 * a1[5][0] * ylk1[4]) / l_105

+ P_m1 * 
    (a1[1][1] / L_10 
  + (2 * a1[2][1] * ylk1[1]) / l_102 
  + (3 * a1[3][1] * ylk1[2]) / l_103 
  + (4 * a1[4][1] * ylk1[3]) / l_104)

+ (P_m12 * (
          + a1[1][2] * l_102
      + 2 * a1[2][2] * L_10 * y 
      + 2 * a1[2][2] * l_102 * k1_0 
      + 3 * a1[3][2] * std::pow(y, 2))
      + 3 * a1[3][2] * l_102 * std::pow(k1_0, 2) 
      + 6 * a1[3][2] * L_10 * k1_0 * y 
  ) / l_103

+ (P_m13 * 
  (L_10 * a1[1][3] 
+ 2 * a1[2][3] * y 
+ 2 * L_10 * a1[2][3] * k1_0)
  ) / l_102

+ (P_m14 * a1[1][4]) / L_10 





    + P_m2 * (a2[1][1] / L_20 
      - (2 * a2[2][1] * ylk2[2]) / l_202 
      + (3 * a2[3][1] * ylk2[2]) / l_203 
      - (4 * a2[4][1] * ylk2[3]) / l_204) 

             

             + a2[1][0] / L_20 

    

    + (P_m24 * a2[1][4]) / L_20 

    

    - (2 * a2[2][0] * ylk2[2]) / l_202

    

    + (P_m22 * 
      (3 * a2[3][2] * l_202 * std::pow(k2_0, 2) 
     + 2 * a2[2][2] * l_202 * k2_0 + a2[1][2] * l_202 
     - 6 * a2[3][2] * L_20 * k2_0 * y 
     - 2 * a2[2][2] * L_20 * y 
     + 3 * a2[3][2] * std::pow(y, 2))
      ) / l_203

      

      + (3 * a2[3][0] * ylk2[2]) / l_203 
      - (4 * a2[4][0] * ylk2[3]) / l_204 
      + (5 * a2[5][0] * ylk2[4]) / l_205

      

      + (P_m23 * 
          (L_20 * a2[1][3] 
        - 2 * a2[2][3] * y 
        + 2 * L_20 * a2[2][3] * k2_0)
          ) / l_202

      )/ mass;

float Lin_mat_2_0 =
    (P_a * 
      (c1 * kyl1 
        + (3 * a1 * kyl12) / L_10 
        + (2 * b1 * kyl1) / L_10)
     * 
     (c1 / L_10 
      + (3 * a1 * kyl12) / L_10 
      + (2 * b1 * kyl1) / L_10))
        / std::pow((d1 + c1 * kyl1 + a1 * kyl13 + b1 * kyl12), 2)
    - (P_a * 
       ((2 * b1) / l_102 
      + c1 / L_10 
      + (6 * a1 * kyl1) / l_102)) 
    / (d1 + c1 * kyl1 + a1 * kyl13 + b1 * kyl12)
    - (P_a * 
      dV_A1dt * (c1 / L_10 
        + (3 * a1 * kyl12) / L_10 
        + (2 * b1 * kyl1) / L_10))
          / std::pow((d1 + c1 * kyl1 + a1 * kyl13 + b1 * kyl12), 2);

float Lin_mat_3_0 =
    (P_a * 
      ((2 * b2) / l_202 
        + c2 / L_20 
        + (6 * a2 * kyl2) / l_202)) 
    / (d2 + c2 * kyl2 + a2 * kyl23 + b2 * kyl22)
    - (P_a * 
      (c2 * kyl2 
        + (3 * a2 * kyl22) / L_20 
        + (2 * b2 * kyl2) / L_20)
       * (c2 / L_20 
        + (3 * a2 * kyl22) / L_20 
        + (2 * b2 * kyl2) / L_20))
          / std::pow((d2 + c2 * kyl2 + a2 * kyl23 + b2 * kyl22), 2)
    + (P_a * dV_A2dt * (c2 / L_20 + (3 * a2 * kyl22) / L_20 + (2 * b2 * kyl2) / L_20))
          / std::pow((d2 + c2 * kyl2 + a2 * kyl23 + b2 * kyl22), 2);
float Lin_mat_0_1 = 0;
float Lin_mat_1_1 = 0;
float Lin_mat_2_1 = 0;
float Lin_mat_3_1 = 0;
float Lin_mat_0_2 = 0;
float Lin_mat_1_2 =
    -(a1[0][1] + a1[1][1] * kyl1 + 3 * P_m12 * (a1[0][3] + a1[1][3] * kyl1 + a1[2][3] * kyl12) - R1 / L_10
      + 2 * P_m1 * (a1[0][2] + a1[1][2] * kyl1 + a1[2][2] * kyl12 + a1[3][2] * kyl13) + a1[2][1] * kyl12
      + a1[3][1] * kyl13 + a1[4][1] * kyl14 + 5 * P_m14 * a1[0][5] + 4 * P_m13 * (a1[0][4] + a1[1][4] * kyl1))
    / mass;
float Lin_mat_2_2 = 0;
float Lin_mat_3_2 = 0;
float Lin_mat_0_3 = 0;
float Lin_mat_1_3 =
    (a2[0][1] + a2[1][1] * kyl2 + 3 * P_m22 * (a2[0][3] + a2[1][3] * kyl2 + a2[2][3] * kyl22) - R2 / L_20
     + 2 * P_m2 * (a2[0][2] + a2[1][2] * kyl2 + a2[2][2] * kyl22 + a2[3][2] * kyl23) + a2[2][1] * kyl22
     + a2[3][1] * kyl23 + a2[4][1] * kyl24 + 5 * P_m24 * a2[0][5] + 4 * P_m23 * (a2[0][4] + a2[1][4] * kyl2))
    / mass;
float Lin_mat_2_3 = 0;
float Lin_mat_3_3 = 0;