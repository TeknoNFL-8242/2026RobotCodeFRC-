package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Intakeencoder: küçük wrapper, ortak Encoder.java'daki fonksiyonları kullanır.
 * Bu sayede enkoderla ilgili tekil merkez dosya Encoder.java olur.
 */
public class Intakeencoder {
    // ortak Encoder sınıfı (Encoder.java)
    private final DriveEncoder impl;

    public Intakeencoder() {
        this.impl = new DriveEncoder();
    }

    public double getRPM() {
        return impl.getRPM();
    }

    public void setRPM(double rpm) {
        impl.setRPM(rpm);
    }

    public double getwheelspeed(){

        double rpm = getRPM();
        double wheelCircumference = Math.PI * RobotConfig.Encoder.WHEEL_DIAMETER_METERS;
        double wheelSpeedMetersPerSecond = (rpm / 60.0) * wheelCircumference;
        SmartDashboard.putNumber(RobotConfig.Encoder.WHEEL_SPEED_KEY, wheelSpeedMetersPerSecond);
        return wheelSpeedMetersPerSecond;
    }
}
