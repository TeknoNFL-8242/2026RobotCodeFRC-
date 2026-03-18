package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * IntakeElevatorSubsystem
 * NEO + Spark MAX integrated encoder ile pozisyon kontrolü.
 * Varsayılan hedefler:
 * - close: 0 tur
 * - open: 2.6
 *  tur
 */

 
public class IntakeElevatorSubsystem {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController openPositionPid;
    private final PIDController closePositionPid;

    private double zeroRot = 0.0;
    private double targetRot = 0.0;
    private boolean closedLoopEnabled = true;
    private boolean emergencyStopped = false;

    public IntakeElevatorSubsystem(SparkMax motor) {
        this.motor = motor;
        this.encoder = motor.getEncoder();
        this.openPositionPid = new PIDController(
            RobotConfig.IntakeElevator.OPEN_KP,
            RobotConfig.IntakeElevator.OPEN_KI,
            RobotConfig.IntakeElevator.OPEN_KD
        );
        this.closePositionPid = new PIDController(
            RobotConfig.IntakeElevator.CLOSE_KP,
            RobotConfig.IntakeElevator.CLOSE_KI,
            RobotConfig.IntakeElevator.CLOSE_KD
        );
        this.openPositionPid.setTolerance(RobotConfig.IntakeElevator.POSITION_TOLERANCE_TURNS);
        this.closePositionPid.setTolerance(RobotConfig.IntakeElevator.POSITION_TOLERANCE_TURNS);

        setZeroHere();
        setClosePosition();
    }

    public void setZeroHere() {
        zeroRot = encoder.getPosition();
    }

    public void setOpenPosition() {
        setTargetTurnsFromZero(RobotConfig.IntakeElevator.OPEN_TURNS_FROM_ZERO);
    }

    public void setCloseDownPosition() {
        setTargetTurnsFromZero(RobotConfig.IntakeElevator.CLOSE_DOWN_TURNS_FROM_ZERO);
    }

    public void setClosePosition() {
        setTargetTurnsFromZero(0.0);
    }

    public void setTargetTurnsFromZero(double turns) {
        if (emergencyStopped) {
            return;
        }
        targetRot = clamp(
            zeroRot + turns,
            zeroRot + RobotConfig.IntakeElevator.MIN_TURNS_FROM_ZERO,
            zeroRot + RobotConfig.IntakeElevator.MAX_TURNS_FROM_ZERO
        );
        closedLoopEnabled = true;
    }

    public void stop() {
        closedLoopEnabled = false;
        motor.set(0.0);
    }

    public void setEmergencyStop(boolean enabled) {
        emergencyStopped = enabled;
        if (emergencyStopped) {
            stop();
        } else {
            closedLoopEnabled = true;
        }
    }

    public boolean isEmergencyStopped() {
        return emergencyStopped;
    }

    public boolean isAtTarget() {
        return Math.abs(targetRot - encoder.getPosition()) <= RobotConfig.IntakeElevator.POSITION_TOLERANCE_TURNS;
    }

    public double getPositionTurnsFromZero() {
        return encoder.getPosition() - zeroRot;
    }

    public double getTargetTurnsFromZero() {
        return targetRot - zeroRot;
    }

    public void periodic() {
        double currentRot = encoder.getPosition();
        double errorTurns = targetRot - currentRot;

        if (emergencyStopped) {
            motor.set(0.0);
        } else if (closedLoopEnabled) {
            if (Math.abs(errorTurns) <= RobotConfig.IntakeElevator.POSITION_TOLERANCE_TURNS) {
                motor.set(0.0);
                SmartDashboard.putNumber("IntakeElevator/ActiveProfileLimit", 0.0);
                SmartDashboard.putNumber("IntakeElevator/PIDErrorTurns", errorTurns);
            } else {
                PIDController activePid = (errorTurns > 0.0) ? openPositionPid : closePositionPid;
                double pidOut = activePid.calculate(currentRot, targetRot);
                double output = pidOut + RobotConfig.IntakeElevator.HOLD_FEEDFORWARD;

            // Directional speed profiles (linear):
            // close: 0.50 -> 0.05, open: 0.30 -> 0.01
                if (errorTurns < 0.0) {
                    double closeMagLimit = getCloseProfileMagnitude(Math.abs(errorTurns));
                    output = clamp(output, -closeMagLimit, closeMagLimit);
                    SmartDashboard.putNumber("IntakeElevator/ActiveProfileLimit", closeMagLimit);
                } else if (errorTurns > 0.0) {
                    double openMagLimit = getOpenProfileMagnitude(Math.abs(errorTurns));
                    output = clamp(output, -openMagLimit, openMagLimit);
                    SmartDashboard.putNumber("IntakeElevator/ActiveProfileLimit", openMagLimit);
                } else {
                    output = clamp(
                        output,
                        -RobotConfig.IntakeElevator.MAX_OUTPUT,
                        RobotConfig.IntakeElevator.MAX_OUTPUT
                    );
                    SmartDashboard.putNumber("IntakeElevator/ActiveProfileLimit", RobotConfig.IntakeElevator.MAX_OUTPUT);
                }
                motor.set(output);
                SmartDashboard.putNumber("IntakeElevator/PIDErrorTurns", errorTurns);
            }
        }

        SmartDashboard.putNumber("IntakeElevator/PositionTurns", getPositionTurnsFromZero());
        SmartDashboard.putNumber("IntakeElevator/TargetTurns", getTargetTurnsFromZero());
        SmartDashboard.putBoolean("IntakeElevator/AtTarget", isAtTarget());
        SmartDashboard.putBoolean("IntakeElevator/ClosedLoopEnabled", closedLoopEnabled);
        SmartDashboard.putBoolean("IntakeElevator/EmergencyStopped", emergencyStopped);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double getCloseProfileMagnitude(double absErrorTurns) {
        double window = Math.max(1e-6, RobotConfig.IntakeElevator.CLOSE_PROFILE_WINDOW_TURNS);
        double alpha = clamp(absErrorTurns / window, 0.0, 1.0);
        // Late-cut close profile: keep power longer, then drop near target.
        double curved = Math.pow(alpha, 0.65);
        return RobotConfig.IntakeElevator.CLOSE_PROFILE_END_OUTPUT
            + (RobotConfig.IntakeElevator.CLOSE_PROFILE_START_OUTPUT
                - RobotConfig.IntakeElevator.CLOSE_PROFILE_END_OUTPUT) * curved;
    }

    private static double getOpenProfileMagnitude(double absErrorTurns) {
        double window = Math.max(1e-6, RobotConfig.IntakeElevator.OPEN_PROFILE_WINDOW_TURNS);
        double alpha = clamp(absErrorTurns / window, 0.0, 1.0);
        return RobotConfig.IntakeElevator.OPEN_PROFILE_END_OUTPUT
            + (RobotConfig.IntakeElevator.OPEN_PROFILE_START_OUTPUT
                - RobotConfig.IntakeElevator.OPEN_PROFILE_END_OUTPUT) * alpha;
    }
}
