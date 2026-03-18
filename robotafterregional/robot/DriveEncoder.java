package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Merkezi enkoder stub'u. İhtiyaç halinde bu sınıf WPILib Encoder ile sarmalanıp
 * gerçek donanıma bağlanır.
 *
 * Kullanım: new Encoder() -> getRPM(), setRPM(...)
 */
public class DriveEncoder {

    private double rpm = 0.0;

    /** Gerçek encoder'dan RPM okunacak. Şu an stub. */
    public double getRPM() {
        //TODO: encoder.getRate() * 60.0 vb. ile bağla
        return rpm;
    }

    public void setRPM(double rpm) { this.rpm = rpm; }

    public double getwheelspeed(){

        double rpm = getRPM();
        double wheelCircumference = Math.PI * RobotConfig.Encoder.WHEEL_DIAMETER_METERS;
        double wheelSpeedMetersPerSecond = (rpm / 60.0) * wheelCircumference;
        SmartDashboard.putNumber(RobotConfig.Encoder.WHEEL_SPEED_KEY, wheelSpeedMetersPerSecond);
        return wheelSpeedMetersPerSecond;
    }
}
