package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.TreeMap;

/**
 * ShooterPhysics - Analytic Physics Engine for FRC Shooter Subsystem
 * 
 * Handles:
 * - Analytic projectile motion calculations (no iterative integration)
 * - Unit conversions (RPM ↔ Ball Velocity)
 * - Precomputed RPM lookup table with linear interpolation
 * - Drag correction via simplified linear factor
 * 
 * Target Performance: <1ms calculation time for lookup-based control
 */
public class ShooterPhysics {

    // ═══════════════════════════════════════════════════════════════════════
    //  HARDWARE & GEOMETRY CONSTANTS (FRC 2026)
    // ═══════════════════════════════════════════════════════════════════════
    
    // Shooter geometry
    private static final double SHOOTER_HEIGHT_M = 0.85;           // Robot shooter height (meters)
    private static final double TARGET_HEIGHT_M = 2.44;            // Target height (8 feet - official spec)
    private static final double LAUNCH_ANGLE_DEG = 45.0;           // Fixed launch angle (degrees)
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(LAUNCH_ANGLE_DEG);
    
    // Ball properties (FRC 2026 Game Piece)
    // private static final double BALL_MASS_KG = 0.216;              // Game piece mass (kg) - TMP
    // private static final double BALL_RADIUS_M = 0.0635;            // Game piece radius (2.5 inches) - TMP
    
    // Flywheel properties
    private static final double WHEEL_RADIUS_M = 0.0508;           // 4-inch wheel (meters)
    
    // Physics constants
    private static final double GRAVITY = 9.81;                    // Gravitational acceleration (m/s²)
    
    // Transfer efficiency (wheel speed → ball speed)
    // Accounts for: grip friction, compression, slip (~10-15% loss)
    private static final double TRANSFER_EFFICIENCY = 0.88;        // 88% of wheel surface speed transfers to ball
    
    // Aerodynamics (simplified for <8m range)
    private static final double DRAG_CORRECTION_FACTOR = 1.02;     // +2% velocity to compensate for drag (linear approximation)
    // private static final double AIR_DENSITY = 1.225;               // kg/m³ at sea level - TMP
    // private static final double DRAG_COEFFICIENT = 0.47;           // Sphere drag coefficient - TMP
    // private static final double BALL_CROSS_SECTION = Math.PI * BALL_RADIUS_M * BALL_RADIUS_M; // TMP
    
    // RPM conversion constants
    private static final double RPM_TO_RAD_PER_SEC = 2.0 * Math.PI / 60.0;
    
    // Lookup table bounds
    private static final double MIN_DISTANCE_M = 1.5;
    private static final double MAX_DISTANCE_M = 8.0;
    private static final double DISTANCE_STEP_M = 0.1;
    
    // Operating limits
    private static final double MAX_RPM = 5500.0;
    private static final double MIN_RPM = 500.0;
    
    // ═══════════════════════════════════════════════════════════════════════
    //  LOOKUP TABLE
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Precomputed RPM lookup table: Distance (m) → Required RPM
     * Generated during initialization to avoid real-time computation
     */
    private static TreeMap<Double, Double> rpmLookupTable = null;
    
    /**
     * Initialize the lookup table (call once during robotInit)
     */
    public static void initializeLookupTable() {
        if (rpmLookupTable != null) {
            return;  // Already initialized
        }
        
        rpmLookupTable = new TreeMap<>();
        
        System.out.println("═══════════════════════════════════════════════════════");
        System.out.println("  SHOOTER PHYSICS - RPM LOOKUP TABLE INITIALIZATION");
        System.out.println("═══════════════════════════════════════════════════════");
        
        for (double distance = MIN_DISTANCE_M; distance <= MAX_DISTANCE_M; distance += DISTANCE_STEP_M) {
            double requiredRPM = calculateRequiredRPM(distance);
            rpmLookupTable.put(distance, requiredRPM);
            
            // Log to console and dashboard
            System.out.printf("Distance: %.1fm → RPM: %.0f (v_exit: %.2f m/s)%n",
                distance, requiredRPM, rpmToVelocity(requiredRPM));
        }
        
        System.out.println("═══════════════════════════════════════════════════════");
        System.out.println("Lookup table initialized. Range: " + MIN_DISTANCE_M + "m to " + MAX_DISTANCE_M + "m");
        System.out.println("═══════════════════════════════════════════════════════");
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  PUBLIC INTERFACE - DISTANCE → RPM CONVERSION
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Get required RPM for a given distance using lookup table + interpolation
     * 
     * @param distanceMeters Target distance in meters
     * @return Required wheel RPM, or -1 if distance is out of range
     */
    public static double getRequiredRPM(double distanceMeters) {
        // Validate range
        if (distanceMeters < MIN_DISTANCE_M || distanceMeters > MAX_DISTANCE_M) {
            SmartDashboard.putString("Shooter/Error", 
                String.format("Distance %.2fm out of range [%.1f, %.1f]m",
                    distanceMeters, MIN_DISTANCE_M, MAX_DISTANCE_M));
            return -1.0;
        }
        
        // Find bounding entries in lookup table
        Double floorKey = rpmLookupTable.floorKey(distanceMeters);
        Double ceilKey = rpmLookupTable.ceilingKey(distanceMeters);
        
        // Exact match
        if (floorKey != null && Math.abs(floorKey - distanceMeters) < 0.001) {
            return rpmLookupTable.get(floorKey);
        }
        
        // Linear interpolation between floor and ceiling
        if (floorKey != null && ceilKey != null && !floorKey.equals(ceilKey)) {
            double rpmFloor = rpmLookupTable.get(floorKey);
            double rpmCeil = rpmLookupTable.get(ceilKey);
            
            // Linear interpolation: RPM = RPM_floor + (RPM_ceil - RPM_floor) * alpha
            double alpha = (distanceMeters - floorKey) / (ceilKey - floorKey);
            double interpolatedRPM = rpmFloor + (rpmCeil - rpmFloor) * alpha;
            
            return clamp(interpolatedRPM, MIN_RPM, MAX_RPM);
        }
        
        // Fallback to nearest
        if (ceilKey != null) {
            return rpmLookupTable.get(ceilKey);
        }
        if (floorKey != null) {
            return rpmLookupTable.get(floorKey);
        }
        
        return -1.0;
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  ANALYTIC PHYSICS - PROJECTILE MOTION
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Calculate required RPM for a given distance using analytic projectile motion
     * 
     * Formula (no drag):
     *   v0 = sqrt( (g * D^2) / (2 * cos²θ * (D*tanθ - ΔH)) )
     * 
     * Where:
     *   D = horizontal distance
     *   ΔH = height difference (target - shooter)
     *   θ = launch angle
     *   g = gravity
     * 
     * Drag is compensated via a simple linear correction factor
     */
    private static double calculateRequiredRPM(double distanceMeters) {
        double deltaH = TARGET_HEIGHT_M - SHOOTER_HEIGHT_M;
        
        // Solve for exit velocity using analytic projectile motion
        double cosTheta = Math.cos(LAUNCH_ANGLE_RAD);
        double tanTheta = Math.tan(LAUNCH_ANGLE_RAD);
        double cos2Theta = cosTheta * cosTheta;
        
        // Denominator check: D*tan(θ) - ΔH must be positive
        double denominator = distanceMeters * tanTheta - deltaH;
        if (denominator <= 0) {
            System.err.println("❌ Projectile motion impossible at distance " + distanceMeters + "m");
            return -1.0;
        }
        
        // v0² = (g * D²) / (2 * cos²θ * (D*tanθ - ΔH))
        double v0Squared = (GRAVITY * distanceMeters * distanceMeters) 
                         / (2.0 * cos2Theta * denominator);
        
        if (v0Squared < 0) {
            return -1.0;  // No real solution
        }
        
        double exitVelocity = Math.sqrt(v0Squared);
        
        // Apply drag correction (simplified linear factor for <8m)
        exitVelocity *= DRAG_CORRECTION_FACTOR;
        
        // Convert exit velocity to wheel RPM
        double requiredRPM = velocityToRPM(exitVelocity);
        
        // Clamp to operational limits
        return clamp(requiredRPM, MIN_RPM, MAX_RPM);
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  UNIT CONVERSIONS - RPM ↔ BALL VELOCITY
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Convert wheel RPM to ball exit velocity
     * 
     * Physics:
     *   Wheel_linvel = (RPM / 60) × 2π × r_wheel
     *   Ball_velocity = Wheel_linvel × transfer_efficiency
     */
    public static double rpmToVelocity(double rpm) {
        // Wheel linear velocity (m/s)
        double wheelLinearVel = rpm * RPM_TO_RAD_PER_SEC * WHEEL_RADIUS_M;
        
        // Apply transfer efficiency (grip/compression losses)
        return wheelLinearVel * TRANSFER_EFFICIENCY;
    }
    
    /**
     * Convert desired ball exit velocity to required wheel RPM
     * 
     * Inverse of rpmToVelocity:
     *   RPM = (Ball_velocity / transfer_efficiency) / (2π × r_wheel) × 60
     */
    public static double velocityToRPM(double ballVelocity) {
        // Reverse transfer efficiency
        double requiredWheelVel = ballVelocity / TRANSFER_EFFICIENCY;
        
        // Convert linear velocity to RPM
        return requiredWheelVel / (2.0 * Math.PI * WHEEL_RADIUS_M) * 60.0;
    }
    
    // ═══════════════════════════════════════════════════════════════════════
    //  UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════
    
    /**
     * Clamp a value between min and max
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Get lookup table size (for debugging)
     */
    public static int getLookupTableSize() {
        return rpmLookupTable != null ? rpmLookupTable.size() : 0;
    }
    
    /**
     * Get min/max distance range
     */
    public static double getMinDistance() { return MIN_DISTANCE_M; }
    public static double getMaxDistance() { return MAX_DISTANCE_M; }
    
    /**
     * Export lookup table to SmartDashboard (for tuning/diagnostics)
     */
    public static void exportLookupTable() {
        if (rpmLookupTable == null) {
            return;
        }
        
        int index = 0;
        for (var entry : rpmLookupTable.entrySet()) {
            SmartDashboard.putNumber("Shooter/LUT_Distance_" + index, entry.getKey());
            SmartDashboard.putNumber("Shooter/LUT_RPM_" + index, entry.getValue());
            index++;
        }
    }
}
