package frc.lib.util;

public class KalmanFilter_s {
    private double x_hat; // 状态估计值
    private double p; // 状态估计协方差

    private double q; // 过程噪声协方差
    private double r; // 测量噪声协方差
    private double k; // 卡尔曼增益

    public KalmanFilter_s(double initialState, double processNoiseVariance, double measurementNoiseVariance) {
        this.x_hat = initialState;
        this.p = 0.01;
        this.q = processNoiseVariance;
        this.r = measurementNoiseVariance;
    }

    public double filter(double measurement) {
        // 预测阶段
        double x_hat_prime = x_hat;
        double p_prime = p + q;

        // 更新阶段
        k = p_prime / (p_prime + r);
        x_hat = x_hat_prime + k * (measurement - x_hat_prime);
        p = (1 - k) * p_prime;
        return x_hat;
    }
}