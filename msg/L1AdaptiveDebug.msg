uint64 timestamp # time since system start (microseconds)

float32 v_pred_x
float32 v_pred_y
float32 v_pred_z

float32 omega_pred_x
float32 omega_pred_y
float32 omega_pred_z

float32 v_pred_error_x
float32 v_pred_error_y
float32 v_pred_error_z

float32 omega_pred_error_x
float32 omega_pred_error_y
float32 omega_pred_error_z

float32 u_ad_0 # thrust
float32 u_ad_1 # moment x
float32 u_ad_2 # moment y
float32 u_ad_3 # moment z

float32 u_b_0 # thrust
float32 u_b_1 # moment x
float32 u_b_2 # moment y
float32 u_b_3 # moment z

float32 sigma_m_0 # uncentainty thrust in z
float32 sigma_m_1 # uncentainty moment x
float32 sigma_m_2 # uncentainty moment y
float32 sigma_m_3 # uncentainty moment z

float32 sigma_um_0 # uncentainty thrust in x
float32 sigma_um_1 # uncentainty thrust in y



bool l1enable
bool l1ctrl_enable

float32 dt

# TOPICS l1_adaptive_debug