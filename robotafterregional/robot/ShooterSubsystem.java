package frc.robot;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterSubsystem
 * Tek dosyada:
 * - Fizik: mesafe -> gerekli cikis hizi -> RPM
 * - Kontrol: Pidf ile hedef RPM sabitleme (P + F ana etki)
 * - Donanim: SparkMax + encoder
 */
public class ShooterSubsystem extends SubsystemBase {

    private final SparkMax shooterMotor;
    private final Encoder wheelEncoder;
    private final Pidf pidfController;

    // State
    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private double targetDistanceMeters = 0.0;
    private double targetExitVelocityMs = 0.0;
    private boolean enabled = false;
    private boolean usePidfControl = RobotConfig.Shooter.DEFAULT_USE_PIDF;

    public ShooterSubsystem() {
        shooterMotor = new SparkMax(RobotConfig.Ports.SHOOTER_SPARK_CAN, MotorType.kBrushed);
        configureSparkMax();

        wheelEncoder = new Encoder(RobotConfig.Ports.SHOOTER_ENCODER_DIO_A, RobotConfig.Ports.SHOOTER_ENCODER_DIO_B);
        wheelEncoder.setDistancePerPulse(1.0 / RobotConfig.Shooter.ENCODER_COUNTS_PER_REV);
        wheelEncoder.setSamplesToAverage(32);

        pidfController = new Pidf(
            RobotConfig.Shooter.DEFAULT_KP,
            RobotConfig.Shooter.DEFAULT_KI,
            RobotConfig.Shooter.DEFAULT_KD,
            RobotConfig.Shooter.DEFAULT_KF
        );
        pidfController.setOutputLimits(0.0, 1.0);

        SmartDashboard.putNumber("Shooter/Tune_kP", RobotConfig.Shooter.DEFAULT_KP);
        SmartDashboard.putNumber("Shooter/Tune_kI", RobotConfig.Shooter.DEFAULT_KI);
        SmartDashboard.putNumber("Shooter/Tune_kD", RobotConfig.Shooter.DEFAULT_KD);
        SmartDashboard.putNumber("Shooter/Tune_kFF", RobotConfig.Shooter.DEFAULT_KF);
        SmartDashboard.putBoolean("Shooter/UsePIDF", RobotConfig.Shooter.DEFAULT_USE_PIDF);

        System.out.println("✓ ShooterSubsystem initialized");
    }

    private void configureSparkMax() {
        try {
            shooterMotor.clearFaults();
            SparkMaxConfig config = new SparkMaxConfig();
            config.voltageCompensation(RobotConfig.BrownoutProtection.SPARK_SHOOTER_VOLTAGE_COMP_SATURATION_V);
            config.smartCurrentLimit(RobotConfig.BrownoutProtection.SPARK_SHOOTER_SMART_CURRENT_LIMIT_A);
            config.secondaryCurrentLimit(RobotConfig.BrownoutProtection.SPARK_SHOOTER_SECONDARY_CURRENT_LIMIT_A);
            config.openLoopRampRate(RobotConfig.BrownoutProtection.SPARK_SHOOTER_OPEN_LOOP_RAMP_SEC);
            config.closedLoopRampRate(RobotConfig.BrownoutProtection.SPARK_SHOOTER_CLOSED_LOOP_RAMP_SEC);
            config.inverted(true);
            shooterMotor.configure(
                config,
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters
            );
        } catch (Exception e) {
            System.err.println("❌ SparkMax configuration failed: " + e.getMessage());
        }
    }

    public void shootAtDistance(double distanceMeters) {
        if (distanceMeters < RobotConfig.Shooter.MIN_DISTANCE_M || distanceMeters > RobotConfig.Shooter.MAX_DISTANCE_M) {
            stop();
            SmartDashboard.putString("Shooter/Error", "Distance out of range");
            return;
        }

        targetDistanceMeters = distanceMeters;
        targetExitVelocityMs = requiredExitVelocity(distanceMeters);
        if (targetExitVelocityMs <= 0) {
            stop();
            SmartDashboard.putString("Shooter/Error", "No valid trajectory");
            return;
        }

        setTargetRPM(velocityToRPM(targetExitVelocityMs));
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = clamp(rpm, 0.0, RobotConfig.Shooter.MAX_RPM);
        this.enabled = targetRPM > 0.0;
        pidfController.reset();
        if (!enabled) {
            shooterMotor.set(0);
        }
    }

    public void stop() {
        targetRPM = 0.0;
        currentRPM = 0.0;
        targetDistanceMeters = 0.0;
        targetExitVelocityMs = 0.0;
        enabled = false;
        pidfController.reset();
        shooterMotor.set(0);
    }

    public boolean isAtSpeed() {
        return enabled
            && Math.abs(currentRPM - targetRPM) <= RobotConfig.Shooter.RPM_AT_SPEED_TOLERANCE;
    }

    public double getCurrentRPM() {
        return Math.abs(wheelEncoder.getRate() * 60.0);
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isEnabled() {
        return enabled;
    }

    @Override
    public void periodic() {
        currentRPM = getCurrentRPM();

        if (enabled && targetRPM > 0.0) {
            updatePIDFromDashboard();
            double output;
            if (usePidfControl) {
                output = pidfController.calculate(currentRPM, targetRPM, RobotConfig.Shooter.LOOP_DT_SEC);
            } else {
                double kFF = SmartDashboard.getNumber("Shooter/Tune_kFF", RobotConfig.Shooter.DEFAULT_KF);
                output = kFF * targetRPM;
            }
            shooterMotor.set(clamp(output, 0.0, 1.0));
        } else {
            shooterMotor.set(0);
        }

        updateTelemetry();
    }

    public double requiredExitVelocity(double distanceMeters) {
        double deltaH = RobotConfig.Shooter.TARGET_HEIGHT_M - RobotConfig.Shooter.SHOOTER_HEIGHT_M;
        double launchAngleRad = Math.toRadians(RobotConfig.Shooter.LAUNCH_ANGLE_DEG);
        double cosTheta = Math.cos(launchAngleRad);
        double tanTheta = Math.tan(launchAngleRad);
        double denominator = distanceMeters * tanTheta - deltaH;
        if (denominator <= 0.0) {
            return -1.0;
        }

        double v0Squared = (RobotConfig.Shooter.GRAVITY * distanceMeters * distanceMeters)
            / (2.0 * cosTheta * cosTheta * denominator);
        if (v0Squared <= 0.0) {
            return -1.0;
        }

        return Math.sqrt(v0Squared) * RobotConfig.Shooter.DRAG_CORRECTION_FACTOR;
    }

    public double velocityToRPM(double velocityMs) {
        double wheelVel = velocityMs / RobotConfig.Shooter.TRANSFER_EFFICIENCY;
        double radPerSec = wheelVel / RobotConfig.Shooter.WHEEL_RADIUS_M;
        return radPerSec / RobotConfig.Shooter.RPM_TO_RAD_PER_SEC;
    }

    public double rpmToVelocity(double rpm) {
        double radPerSec = rpm * RobotConfig.Shooter.RPM_TO_RAD_PER_SEC;
        return radPerSec * RobotConfig.Shooter.WHEEL_RADIUS_M * RobotConfig.Shooter.TRANSFER_EFFICIENCY;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Shooter/CurrentRPM", currentRPM);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/RPM_Error", targetRPM - currentRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
        SmartDashboard.putBoolean("Shooter/Enabled", enabled);
        SmartDashboard.putBoolean("Shooter/UsePIDF_Active", usePidfControl);
        SmartDashboard.putNumber("Shooter/DistanceTarget", targetDistanceMeters);
        SmartDashboard.putNumber("Shooter/TargetExitVelocity_ms", targetExitVelocityMs);
        SmartDashboard.putNumber("Shooter/MeasuredExitVelocity_ms", rpmToVelocity(currentRPM));
        SmartDashboard.putString("Shooter/Status", enabled ? (isAtSpeed() ? "Ready" : "Spinning Up") : "Disabled");
        SmartDashboard.putNumber("Shooter/MotorOutput_percent", shooterMotor.getAppliedOutput() * 100.0);
        SmartDashboard.putNumber("Shooter/MotorTemp", shooterMotor.getMotorTemperature());
    }

    public void updatePIDFromDashboard() {
        double kP = SmartDashboard.getNumber("Shooter/Tune_kP", RobotConfig.Shooter.DEFAULT_KP);
        double kI = SmartDashboard.getNumber("Shooter/Tune_kI", RobotConfig.Shooter.DEFAULT_KI);
        double kD = SmartDashboard.getNumber("Shooter/Tune_kD", RobotConfig.Shooter.DEFAULT_KD);
        double kFF = SmartDashboard.getNumber("Shooter/Tune_kFF", RobotConfig.Shooter.DEFAULT_KF);
        usePidfControl = SmartDashboard.getBoolean("Shooter/UsePIDF", RobotConfig.Shooter.DEFAULT_USE_PIDF);

        pidfController.setPIDF(kP, kI, kD, kFF);

        SmartDashboard.putString(
            "Shooter/TuneStatus",
            usePidfControl ? "PIDF Active - Reading values" : "FF Only - PIDF Bypassed"
        );
        SmartDashboard.putNumber("Shooter/Tune_kP_Actual", kP);
        SmartDashboard.putNumber("Shooter/Tune_kI_Actual", kI);
        SmartDashboard.putNumber("Shooter/Tune_kD_Actual", kD);
        SmartDashboard.putNumber("Shooter/Tune_kFF_Actual", kFF);
        SmartDashboard.putBoolean("Shooter/UsePIDF_Active", usePidfControl);
    }

    public void resetEncoder() {
        wheelEncoder.reset();
    }
}
