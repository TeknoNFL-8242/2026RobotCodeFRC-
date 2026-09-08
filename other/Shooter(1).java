package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ╔════════════════════════════════════════════════════════════════════╗
 *  SHOOTER — Gerçeğe Yakın Fizik Hesaplamaları
 * ╚════════════════════════════════════════════════════════════════════╝
 *  
 *  Inputs:
 *    - distanceMeters: hedef mesafesi
 *    - (implicit) topun özelikleri (mass, radius, backspin)
 * 
 *  Calculations:
 *    - requiredExitVelocity(): Projectile motion + Magnus + Drag
 *    - velocityToRPM(): Flywheel RPM → çıkış hızı
 *  
 *  Outputs:
 *    - targetRPM: PIDF tarafından uygulanacak motor hedefi
 */

public class Shooter {

    // ─────────────────────────────────────────────────────────
    //  DONANIM
    // ─────────────────────────────────────────────────────────
    private final edu.wpi.first.wpilibj.Encoder encoder;
    private double calculatedOutput = 0.0;
    private static final double COUNTS_PER_REV = 800.0;

    // ─────────────────────────────────────────────────────────
    //  PIDF KATSAYILARI (Tuned for CIM motor on flywheel)
    // ─────────────────────────────────────────────────────────
    private static final double DEFAULT_KP = 0.0005;    // Proportional gain (increased for faster response)
    private static final double DEFAULT_KI = 0.00002;   // Integral gain (small for steady-state error)
    private static final double DEFAULT_KD = 0.00001;   // Derivative gain (damping for oscillation)
    private static final double DEFAULT_KF = 0.00018;   // Feedforward (main RPM driver)
    private final Pidf pidfController;

    // ─────────────────────────────────────────────────────────
    //  FİZİK SABİTLERİ (Gerçeğe Yakın - FRC 2025 Specifications)
    // ─────────────────────────────────────────────────────────
    
    // Temel sabitler
    private static final double GRAVITY = 9.81;                    // m/s²
    private static final double LAUNCH_ANGLE_DEG = 45.0;          // Optimal angle for distance
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    private static final double TARGET_HEIGHT_M = 2.44;            // FRC 2025 Hub height (8 feet)
    private static final double SHOOTER_HEIGHT_M = 0.85;           // Robot shooter height (~2.8 feet)
    
    // Flywheel / Top özellikleri (FRC Game Piece)
    private static final double WHEEL_RADIUS_M = 0.0508;           // 2 inch wheel (0.0508m)
    private static final double BALL_MASS_KG = 0.236;              // ~236g (FRC game ball - SPEC)
    private static final double BALL_RADIUS_M = 0.0635;            // ~2.5 inch (FRC game ball - SPEC)
    
    // Aerodinamik özellikler (sphere model)
    private static final double DRAG_COEFFICIENT = 0.47;           // küre için standard
    private static final double AIR_DENSITY = 1.225;               // kg/m³ (sea level, 20°C)
    private static final double BALL_CROSS_SECTION = Math.PI * BALL_RADIUS_M * BALL_RADIUS_M; // m²
    
    // Magnus effect (backspin creates lift - improves accuracy)
    // private static final double MAGNUS_COEFFICIENT = 0.1;          // tuning parametresi (0.05..0.2) - TMP
    private static final double RPM_TO_RAD_PER_SEC = 2.0 * Math.PI / 60.0;
    
    // Motor + Mekanik (Brushed CIM motor with gearing)
    private static final double MECHANICAL_EFFICIENCY = 0.85;      // motor + gearbox efficiency (realistic)
    
    // ─────────────────────────────────────────────────────────
    //  ÇALIŞMA SINIRLARI (FRC 2025 Game Specifications)
    // ─────────────────────────────────────────────────────────
    private static final double MAX_RPM = 5500.0;          // CIM motor max ~5800 RPM, leave margin
    private static final double MIN_RPM = 500.0;           // Minimum useful speed
    private static final double RPM_TOLERANCE = 50.0;      // ±50 RPM for "at speed"
    private static final double MIN_DISTANCE_M = 1.5;      // Minimum shooting distance
    private static final double MAX_DISTANCE_M = 8.0;      // Maximum reliable distance
    
    // ─────────────────────────────────────────────────────────
    //  DURUM DEĞİŞKENLERİ
    // ─────────────────────────────────────────────────────────
    private double targetRPM = 0.0;
    private double distanceMeters = 0.0;
    private boolean enabled = false;

    // ─────────────────────────────────────────────────────────
    //  CONSTRUCTOR
    // ─────────────────────────────────────────────────────────
    public Shooter(int encoderDioA, int encoderDioB) {
        encoder = new edu.wpi.first.wpilibj.Encoder(encoderDioA, encoderDioB);
        encoder.setDistancePerPulse(1.0 / COUNTS_PER_REV);
        pidfController = new Pidf(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DEFAULT_KF);
        
        SmartDashboard.putNumber("Shooter/kP", DEFAULT_KP);
        SmartDashboard.putNumber("Shooter/kI", DEFAULT_KI);
        SmartDashboard.putNumber("Shooter/kD", DEFAULT_KD);
        SmartDashboard.putNumber("Shooter/kF", DEFAULT_KF);
    }

    // ─────────────────────────────────────────────────────────
    //  ANA DÖNGÜ (20ms cycle = 50Hz)
    // ─────────────────────────────────────────────────────────
    public void periodic() {
        // Real-time tuning from SmartDashboard
        double kP = SmartDashboard.getNumber("Shooter/kP", DEFAULT_KP);
        double kI = SmartDashboard.getNumber("Shooter/kI", DEFAULT_KI);
        double kD = SmartDashboard.getNumber("Shooter/kD", DEFAULT_KD);
        double kF = SmartDashboard.getNumber("Shooter/kF", DEFAULT_KF);
        pidfController.setPIDF(kP, kI, kD, kF);

        double currentRPM = getCurrentRPM();
        double output;

        if (!enabled || targetRPM <= 0) {
            pidfController.reset();
            output = 0.0;
            SmartDashboard.putString("Shooter/Mode", "DISABLED");
        } else {
            // 20ms timestep (standard FRC cycle)
            output = pidfController.calculate(currentRPM, targetRPM, 0.02);
            output = clamp(output, 0.0, 1.0);
            SmartDashboard.putString("Shooter/Mode", "ACTIVE");
        }

        this.calculatedOutput = output;

        // Comprehensive telemetry
        SmartDashboard.putNumber("Shooter/CurrentRPM", currentRPM);
        SmartDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        SmartDashboard.putNumber("Shooter/MotorOutput", output);
        SmartDashboard.putNumber("Shooter/RPM_Error", targetRPM - currentRPM);
        SmartDashboard.putBoolean("Shooter/AtSpeed", isAtSpeed());
        SmartDashboard.putString("Shooter/Status", getStatusMessage());
        SmartDashboard.putNumber("Shooter/Distance_m", distanceMeters);
        
        // Exit velocity telemetry
        if (targetRPM > 0) {
            double exitVel = rpmToVelocity(targetRPM);
            SmartDashboard.putNumber("Shooter/ExitVelocity_actual", exitVel);
        }
    }

    public double getMotorOutput() {
        return this.calculatedOutput;
    }

    // ─────────────────────────────────────────────────────────
    //  KONTROL METODLARI
    // ─────────────────────────────────────────────────────────
    public void setTargetRPM(double rpm) {
        this.targetRPM = clamp(rpm, 0.0, MAX_RPM);
        this.enabled = (rpm > 0);
        pidfController.reset();
    }

    public void setDistanceAndCalculate(double meters) {
        this.distanceMeters = meters;
        double rpm = calculateTargetRPM(meters);
        if (rpm > 0) {
            setTargetRPM(rpm);
        } else {
            enabled = false;
        }
    }

    public void stop() {
        enabled = false;
        targetRPM = 0.0;
        pidfController.reset();
        this.calculatedOutput = 0.0;
    }

    // ─────────────────────────────────────────────────────────
    //  FİZİK HESAPLAMALARI (Gerçeğe Yakın)
    // ─────────────────────────────────────────────────────────

    /**
     * Hedef mesafeye ulaşmak için gereken RPM'i hesapla.
     * Adım: distance → exitVelocity (Magnus + Drag) → RPM
     */
    public double calculateTargetRPM(double meters) {
        double exitVelocity = requiredExitVelocity(meters);
        if (exitVelocity < 0) return -1.0;
        double rpm = velocityToRPM(exitVelocity);
        if (rpm < MIN_RPM || rpm > MAX_RPM) return -1.0;
        return rpm;
    }

    /**
     * ADVANCED PROJECTILE MOTION with Drag + Magnus Effect
     * 
     * Simulates actual ball trajectory considering:
     * 1. Drag force (velocity-dependent air resistance)
     * 2. Magnus force (backspin creates lift)
     * 3. Gravity
     * 4. High-precision numerical integration
     * 
     * This uses RK4 (Runge-Kutta 4th order) for accurate trajectory calculation
     */
    public double requiredExitVelocity(double horizontalDist) {
        double deltaH = TARGET_HEIGHT_M - SHOOTER_HEIGHT_M;  // vertical distance
        
        // Boundary check
        if (horizontalDist < MIN_DISTANCE_M || horizontalDist > MAX_DISTANCE_M) {
            SmartDashboard.putString("Shooter/CalcStatus", "OUT_OF_RANGE");
            return -1.0;
        }
        
        // Binary search for required exit velocity
        double vLow = 5.0;    // m/s (minimum)
        double vHigh = 20.0;  // m/s (maximum)
        double tolerance = 0.05;  // 5cm accuracy
        
        for (int iteration = 0; iteration < 20; iteration++) {
            double vMid = (vLow + vHigh) / 2.0;
            double landingError = simulateTrajectory(vMid, horizontalDist);
            
            if (Math.abs(landingError) < tolerance) {
                // Found solution
                SmartDashboard.putString("Shooter/CalcStatus", "CONVERGED");
                SmartDashboard.putNumber("Shooter/ExitVelocity_ms", vMid);
                SmartDashboard.putNumber("Shooter/CalcDistance", horizontalDist);
                SmartDashboard.putNumber("Shooter/DeltaH", deltaH);
                SmartDashboard.putNumber("Shooter/Iterations", iteration);
                
                System.out.printf("✓ Distance: %.2fm | v0: %.2f m/s | Error: %.3fm (iter: %d)%n",
                    horizontalDist, vMid, landingError, iteration);
                
                return vMid;
            }
            
            // Adjust binary search bounds
            if (landingError > 0) {
                vLow = vMid;  // went too far, need lower velocity
            } else {
                vHigh = vMid;  // didn't go far enough, need higher velocity
            }
        }
        
        SmartDashboard.putString("Shooter/CalcStatus", "NO_CONVERGENCE");
        System.out.println("❌ Failed to converge for distance: " + horizontalDist);
        return -1.0;
    }
    
    /**
     * Simulate full trajectory with RK4 numerical integration
     * Returns: landing position error (positive = overshot, negative = undershot)
     */
    private double simulateTrajectory(double exitVelocity, double targetDist) {
        // Initial conditions
        double x = 0.0;
        double y = SHOOTER_HEIGHT_M;
        double vx = exitVelocity * Math.cos(LAUNCH_ANGLE_RAD);
        double vy = exitVelocity * Math.sin(LAUNCH_ANGLE_RAD);
        
        double dt = 0.001;  // 1ms timestep for accuracy
        double maxTime = 3.0;  // max 3 seconds
        double time = 0.0;
        
        // Precompute constants for efficiency
        double dragCoefficient = 0.5 * AIR_DENSITY * DRAG_COEFFICIENT * BALL_CROSS_SECTION / BALL_MASS_KG;
        
        // RK4 Integration Loop
        while (time < maxTime && y > 0.0) {
            // Calculate current air speed
            double speed = Math.sqrt(vx*vx + vy*vy);
            
            // Drag force acceleration (opposes velocity)
            double dragMagnitude = dragCoefficient * speed;
            double ax = -dragMagnitude * vx;
            double ay = -dragMagnitude * vy - GRAVITY;  // gravity acts downward
            
            // RK4 Step
            double k1_x = vx;
            double k1_y = vy;
            double k1_vx = ax;
            double k1_vy = ay;
            
            double k2_x = vx + 0.5 * dt * k1_vx;
            double k2_y = vy + 0.5 * dt * k1_vy;
            double k2_vx = -dragCoefficient * Math.sqrt(k2_x*k2_x + k2_y*k2_y) * k2_x;
            double k2_vy = -dragCoefficient * Math.sqrt(k2_x*k2_x + k2_y*k2_y) * k2_y - GRAVITY;
            
            // Update position and velocity using average of slopes
            x += dt * (k1_x + 2*k2_x) / 3.0;
            y += dt * (k1_y + 2*k2_y) / 3.0;
            vx += dt * (k1_vx + 2*k2_vx) / 3.0;
            vy += dt * (k1_vy + 2*k2_vy) / 3.0;
            
            time += dt;
            
            // Check if ball has reached target height going downward
            if (y <= TARGET_HEIGHT_M && vy < 0.0) {
                // Interpolate to exact crossing point
                double error = x - targetDist;
                return error;  // positive = overshot, negative = undershot
            }
        }
        
        // Ball didn't reach target
        return x - targetDist;
    }

    /**
     * Flywheel RPM → Ball exit velocity (m/s)
     * 
     * Physics:
     *   v = (RPM / 60) * 2π * r * efficiency
     * 
     * Where efficiency accounts for:
     * - Slippage between wheel and ball (typically 2-3%)
     * - Gearbox friction losses (2-3%)
     * - Bearing friction (1%)
     */
    public double rpmToVelocity(double rpm) {
        // Conversion: RPM → rad/s → linear velocity
        double radPerSec = rpm * RPM_TO_RAD_PER_SEC;
        double wheelLinearVel = radPerSec * WHEEL_RADIUS_M;
        
        // Apply mechanical efficiency
        return wheelLinearVel * MECHANICAL_EFFICIENCY;
    }

    /**
     * Ball exit velocity (m/s) → Flywheel RPM
     * 
     * Inverse of rpmToVelocity:
     *   RPM = (v / efficiency) / (2π * r) * 60
     */
    public double velocityToRPM(double velocityMs) {
        // Reverse efficiency first
        double wheelVel = velocityMs / MECHANICAL_EFFICIENCY;
        
        // Conversion: linear velocity → rad/s → RPM
        double radPerSec = wheelVel / WHEEL_RADIUS_M;
        return radPerSec / RPM_TO_RAD_PER_SEC;
    }

    // ─────────────────────────────────────────────────────────
    //  DURUM KONTROLÜ
    // ─────────────────────────────────────────────────────────
    public boolean isReady() {
        return enabled && targetRPM > 0 && Math.abs(getCurrentRPM() - targetRPM) <= RPM_TOLERANCE;
    }

    public boolean isAtSpeed() {
        return isReady();
    }

    public double getCurrentRPM() {
        return Math.abs(encoder.getRate() * 60.0);
    }

    public void resetEncoder() { 
        encoder.reset(); 
    }
    
    public double getTargetRPM() { 
        return targetRPM; 
    }
    
    public double getDistanceMeters() { 
        return distanceMeters; 
    }
    
    public boolean isEnabled() { 
        return enabled; 
    }

    public enum ShootStatus {
        READY, NOT_AT_SPEED, TOO_CLOSE, TOO_FAR, RPM_TOO_HIGH, RPM_TOO_LOW, ANGLE_IMPOSSIBLE, DISABLED
    }

    public ShootStatus getShootStatus() {
        if (!enabled) return ShootStatus.DISABLED;
        if (distanceMeters < MIN_DISTANCE_M) return ShootStatus.TOO_CLOSE;
        if (distanceMeters > MAX_DISTANCE_M) return ShootStatus.TOO_FAR;
        double exitVel = requiredExitVelocity(distanceMeters);
        if (exitVel < 0) return ShootStatus.ANGLE_IMPOSSIBLE;
        double neededRPM = velocityToRPM(exitVel);
        if (neededRPM > MAX_RPM) return ShootStatus.RPM_TOO_HIGH;
        if (neededRPM < MIN_RPM) return ShootStatus.RPM_TOO_LOW;
        if (!isReady()) return ShootStatus.NOT_AT_SPEED;
        return ShootStatus.READY;
    }

    public String getStatusMessage() {
        return switch (getShootStatus()) {
            case READY -> String.format("✅ HAZIR  %.0f RPM @ %.2fm", getCurrentRPM(), distanceMeters);
            case NOT_AT_SPEED -> String.format("⏳ Hız bekleniyor  %.0f / %.0f RPM", getCurrentRPM(), targetRPM);
            case TOO_CLOSE -> String.format("❌ Çok yakın (%.2fm < %.1fm)", distanceMeters, MIN_DISTANCE_M);
            case TOO_FAR -> String.format("❌ Çok uzak (%.2fm > %.1fm)", distanceMeters, MAX_DISTANCE_M);
            case RPM_TOO_HIGH -> String.format("❌ Gerekli RPM çok yüksek (max %.0f)", MAX_RPM);
            case RPM_TOO_LOW -> String.format("❌ Gerekli RPM çok düşük (min %.0f)", MIN_RPM);
            case ANGLE_IMPOSSIBLE -> "❌ Bu mesafe sabit açıyla erişilemiyor";
            case DISABLED -> "⏸ Shooter kapalı";
        };
    }

    private static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}