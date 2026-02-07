void SetMotorSpeedMPS(float speed_mps)
{
    // Clamp speed to [0, MAX_SPEED_MPS]
    if (speed_mps < 0.0f)
    {
        speed_mps = 0.0f;
    }

    if (speed_mps > MAX_SPEED_MPS) speed_mps = MAX_SPEED_MPS;

    // Convert to ratio (0..1)
    float ratio = speed_mps / MAX_SPEED_MPS;

    // Convert ratio to CCR (0..PWM_MAX_CCR)
    uint32_t ccr = (uint32_t) (ratio * (float)PWM_MAX_CCR);

    if (ccr > PWM_MAX_CCR) ccr = PWM_MAX_CCR;

    TIM2->CCR1 = ccr;
}




