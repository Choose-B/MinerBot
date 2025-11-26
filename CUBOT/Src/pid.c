# include "pid.h"

uint8_t PID_Init( PID* pid,
                float k_p,
                float k_i,
                float k_d,
                float target,
                int frequency,
                float max_output,
                float i_limit,
                float episilon
            ) {
    if (pid == NULL || frequency <= 0 || max_output < 0 || i_limit < 0) {
        return 0;
    }

    pid->k_p = k_p;
    pid->k_i = k_i;
    pid->k_d = k_d;
    for (int i = 0;i < HIGHEST_DERIVATIVE;i++){
        pid->target[i] = target;
    }
    pid->frequency = frequency;
    pid->max_output = max_output;
    pid->i_limit = i_limit;
    pid->episilon = episilon;

    pid->error = 0.0f;
    pid->p = 0.0f;
    pid->i = 0.0f;
    pid->d = 0.0f;
    pid->beta = 0;
    pid->last_measure = 0.0f;

    return 1;
}

float PID_Calc(PID* pid, float measure,int n, ...) {
    pid->last_measure = pid->measure;
    pid->measure = pid->last_measure * 0.8f + measure * 0.2f;// 低通滤波
    pid->error = pid->target[0] - measure;

    // Proportional term
    pid->p = pid->k_p * pid->error;

    // Integral term
    // 积分分离
    if ( fabs(pid->error) < pid->episilon ) {
        pid->beta = 1;
    }
    else {
        pid->beta = 0;
        // 不置0容易过调
        pid->i = 0;
    }

    if ( pid->beta ) {
        pid->i += pid->k_i * pid->error / pid->frequency ;
        //积分限幅
        if ( fabs(pid->i) > pid->i_limit ){
            pid->i = pid->i > 0 ? pid->i_limit : -pid->i_limit;
        }
    }
    

    // Derivative term
    // 使用微分先行
    pid->d = (measure - pid->last_measure) * pid->k_d * pid->frequency;

    // 前馈
    pid->feedforward = 0;
    if (n > 0) {
        va_list args;
        va_start(args, n);
        for (int i = 0; i < n; i++) {
            float (*feedforward_func)(PID*) = va_arg(args, float(*)(PID*));
            pid->feedforward += feedforward_func(pid);
        }
        va_end(args);
    }
    // 前馈限幅，防一手没有做限幅的前馈函数  
    // 前馈函数如果不做限幅会导致PID算法无效化
    if ( fabs(pid->feedforward) > pid->max_output ){
        if (pid->feedforward < 0){
            pid->feedforward = -pid->max_output;
        }
        else {
            pid->feedforward = pid->max_output;
        }
    }

    // 总输出
    float output = pid->p + pid->i * pid->beta + pid->d + pid->feedforward;

    // 输出限幅
    return fabs(output) > pid->max_output ? 
            output > 0 ? pid->max_output : -pid->max_output : 
            output;
}

void PID_target(PID* pid, int target_0){
    for (int i = 0;i < HIGHEST_DERIVATIVE-1;i++){
        pid->target[i+1] = pid->target[i];
    }
    pid->target[0] = target_0;
}

void PID_Copy(PID* dest, PID* src) {
    if (dest == NULL || src == NULL) {
        return;
    }

    dest->frequency = src->frequency;
    for (int i = 0;i < HIGHEST_DERIVATIVE;i++){
        dest->target[i] = src->target[i];
    }
    dest->measure = src->measure;
    dest->last_measure = src->last_measure;
    dest->error = src->error;
    dest->p = src->p;
    dest->k_p = src->k_p;
    dest->i = src->i;
    dest->k_i = src->k_i;
    dest->i_limit = src->i_limit;
    dest->episilon = src->episilon;
    dest->beta = src->beta;
    dest->d = src->d;
    dest->k_d = src->k_d;
    dest->max_output = src->max_output;
}
