package frc.robot;
//PIDF (For shooter wheel, lever wheel, intake wheel), PID
public class Pidf {
    private double kP, kI, kD, kF;
    private double integralSum = 0;
    private double lastError = 0;
    private double maxOutput = 1.0;
    private double minOutput = 0.0;

    public Pidf(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public double calculate(double currentRPM, double targetRPM, double dt) {
        double error = targetRPM - currentRPM;

        double proportional = kP * error;

        // Integral with anti-windup
        integralSum += error * dt;
        double integral = kI * integralSum;

        double derivative = kD * (error - lastError) / dt;
        lastError = error;

        double feedforward = kF * targetRPM;

        double output = proportional + integral + derivative + feedforward;

        // Anti-windup: Eğer çıkış limitleri aşıyorsa, integral birikimini geri al
        if (output > maxOutput || output < minOutput) {
            integralSum -= error * dt;  // son eklenen integrali geri al
        }

        return output;
    }

    public void setPIDF(double p, double i, double d, double f) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }


    public void reset() {
        integralSum = 0;
        lastError = 0;
    }

    // İsteğe bağlı: Çıkış limitlerini değiştirmek için
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }
}