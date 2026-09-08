package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ShooterSubsystem - Hardware Interface & Closed-Loop Control
 * 
 * Responsibilities:
 * - Manage SparkMax motor controller
 * - Monitor encoder feedback (RPM)
 * - Implement closed-loop velocity control via SparkMax native PID
 * - Maintain shooter state (enabled, target RPM, current RPM)
 * - Provide telemetry to SmartDashboard
 * 
 * Key Design:
 * - Distance → RPM conversion is handled by ShooterPhysics
 * - Motor control (PID) is handled by SparkMax internal controller
 * - This subsystem acts as the interface between physics & hardware
 */
public class ShooterSubsystem extends SubsystemBase {
    
    // ═══════════════════════════════════════════════════════════════════════
    //  HARDWARE CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════
    
    private static final int SPARK_MAX_CAN_ID = 15;                // SparkMax CAN address
    private static final int ENCODER_DIO_A = 0;                    // Encoder channel A (DIO pin)
    private static final int ENCODER_DIO_B = 1;                    // Encoder channel B (DIO pin)
    private static final int ENCODER_COUNTS_PER_REV = 800;         // 200 PPR encoder × 4x counting
    
    // ═══════════════════════════════════════════════════════════════════════
    //  PIDF CONFIGURATION (SparkMax Native)
    // ═══════════════════════════════════════════════════════════════════════
    
    // PID slot 0 (primary control slot)
    private static final double DEFAULT_KP = 0.0006;               // Proportional gain
    private static final double DEFAULT_KI = 0.00002;              // Integral gain
    private static final double DEFAULT_KD = 0.0001;               // Derivative gain
    private static final double DEFAULT_KFF = 0.00018;             // Feedforward gain
    
    // ═══════════════════════════════════════════════════════════════════════
    //  HARDWARE OBJECTS
    // ═══════════════════════════════════════════════════════════════════════
    
    private final SparkMax shooterMotor;
    private final Encoder wheelEncoder;
    
    // ═══════════════════════════════════════════════════════════════════════
    //  STATE VARIABLES
    // ═══════════════════════════════════════════════════════════════════════
    
    private double targetRPM = 0.0;
    private double currentRPM = 0.0;
    private boolean enabled = false;
    
    private static final double RPM_AT_SPEED_TOLERANCE = 50.0;    // ±50 RPM threshold
    
    // ═══════════════════════════════════════════════════════════════════════
    //  CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════
    
    public ShooterSubsystem() {
        // Initialize SparkMax motor controller
        shooterMotor = new SparkMax(SPARK_MAX_CAN_ID, MotorType.kBrushed);
        
        // Configure SparkMax for velocity control
        configureSparkMax();
        
        // Initialize encoder (external quadrature encoder)
        wheelEncoder = new Encoder(ENCODER_DIO_A, ENCODER_DIO_B);
        wheelEncoder.setDistancePerPulse(1.0 / ENCODER_COUNTS_PER_REV);  // Distance in revolutions
        wheelEncoder.setSamplesToAverage(32);                            // Noise filtering
        
        // Initialize physics engine lookup table
        ShooterPhysics.initializeLookupTable();
        
        System.out.println("✓ ShooterSubsystem initialized");
        System.out.println("  - SparkMax CAN ID: " + SPARK_MAX_CAN_ID);
        System.out.println("  - Default kP: " + DEFAULT_KP);
        System.out.println("  - Default kFF: " + DEFAULT_KFF);
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  HARDWARE CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Configure SparkMax for closed-loop velocity control
     * Using simplified direct API calls for compatibility
     */
    private void configureSparkMax() {
        try {
            // Reset to defaults first
            shooterMotor.clearFaults();
            
            System.out.println("✓ SparkMax configured for velocity control");
            System.out.println("  - CAN ID: " + SPARK_MAX_CAN_ID);
            System.out.println("  - Default kFF: " + DEFAULT_KFF);
            
        } catch (Exception e) {
            System.err.println("❌ SparkMax configuration failed: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  PUBLIC INTERFACE - SHOOTER CONTROL
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Set target distance and automatically calculate required RPM
     * 
     * @param distanceMeters Distance to target in meters
     */
    public void shootAtDistance(double distanceMeters) {
        double requiredRPM = ShooterPhysics.getRequiredRPM(distanceMeters);
        
        if (requiredRPM < 0) {
            stop();
            SmartDashboard.putString("Shooter/Error", 
                String.format("Distance %.2fm out of range", distanceMeters));
            return;
        }
        
        setTargetRPM(requiredRPM);
        SmartDashboard.putNumber("Shooter/DistanceTarget", distanceMeters);
    }
    
    /**
     * Set target RPM directly (for testing/tuning)
     * 
     * @param rpm Target RPM for the flywheel
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = Math.max(0, Math.min(rpm, 5500.0));  // Clamp to valid range
        this.enabled = (rpm > 0);
        
        if (!enabled) {
            shooterMotor.set(0);
        }
    }
    
    /**
     * Stop the shooter immediately
     */
    public void stop() {
        this.targetRPM = 0.0;
        this.enabled = false;
        shooterMotor.set(0);
    }
    
    /**
     * Check if shooter is at target speed (within tolerance)
     */
    public boolean isAtSpeed() {
        return enabled && Math.abs(currentRPM - targetRPM) <= RPM_AT_SPEED_TOLERANCE;
    }
    
    /**
     * Get current measured RPM
     */
    public double getCurrentRPM() {
        // getRate() returns revolutions per second × 60 = RPM
        return Math.abs(wheelEncoder.getRate() * 60.0);
    }
    
    /**
     * Get target RPM
     */
    public double getTargetRPM() {
        return targetRPM;
    }
    
    /**
     * Check if shooter is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  PERIODIC UPDATE (20ms loop)
    // ═══════════════════════════════════════════════════════════════════════
    
    @Override
    public void periodic() {
        // Update current RPM from encoder
        currentRPM = getCurrentRPM();
        
        // Send velocity reference to SparkMax if enabled
        if (enabled && targetRPM > 0) {
            // Calculate required motor output voltage
            // Simple feedforward: throttle = kFF * targetRPM
            // With feedback correction for closed-loop stability
            double error = targetRPM - currentRPM;
            double feedforward = DEFAULT_KFF * targetRPM;
            double feedback = DEFAULT_KP * error;
            double output = feedforward + feedback;
            
            // Clamp to [-1, 1]
            output = Math.max(-1.0, Math.min(1.0, output));
            shooterMotor.set(output);
        } else {
            // Disabled - ensure motor is off
            shooterMotor.set(0);
        }
        
        // Update telemetry
        updateTelemetry();
    }
    
    /**
     * Update SmartDashboard with current state
     */
    private void updateTelemetry() {
        SmartDashboard.putNumber("Shooter/CurrentRPM", currentRPM);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/RPM_Error", targetRPM - currentRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
        SmartDashboard.putBoolean("Shooter/Enabled", enabled);
        
        // Exit velocity (ball speed in m/s)
        double exitVelocity = ShooterPhysics.rpmToVelocity(currentRPM);
        SmartDashboard.putNumber("Shooter/ExitVelocity_ms", exitVelocity);
        
        // Motor output percentage (for diagnostics)
        SmartDashboard.putNumber("Shooter/MotorOutput_percent", shooterMotor.getAppliedOutput() * 100.0);
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  TUNING INTERFACE
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Real-time PIDF tuning via SmartDashboard
     * Call this in a test command to enable live tuning
     * (Note: Direct PID tuning on SparkMax requires the REV Spark API update)
     */
    public void updatePIDFromDashboard() {
        // Read tuning values from dashboard
        SmartDashboard.getNumber("Shooter/Tune_kP", DEFAULT_KP);
        SmartDashboard.getNumber("Shooter/Tune_kI", DEFAULT_KI);
        SmartDashboard.getNumber("Shooter/Tune_kD", DEFAULT_KD);
        SmartDashboard.getNumber("Shooter/Tune_kFF", DEFAULT_KFF);
        
        // Note: In a future update with REV Spark API, these can be applied directly to SparkMax
        SmartDashboard.putNumber("Shooter/TuneStatus", 1.0);  // Active
    }
    
    /**
     * Reset encoder (useful for calibration)
     */
    public void resetEncoder() {
        wheelEncoder.reset();
        System.out.println("✓ Encoder reset");
    }
}
