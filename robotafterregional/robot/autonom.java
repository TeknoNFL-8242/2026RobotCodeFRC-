package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * autonom.java
 *
 * Tüm otonom rutinleri bu sınıfta toplanmıştır.
 * Robot.java autonomousPeriodic() içinden step() çağrılır.
 *
 * Mevcut modlar:
 *  ┌──────────────────────────────────────────────────────────────────────┐
 *  │  "doNothing"      → hiçbir hareket yok                               │
 *  │  "ileriGit"        → 2 saniye düz ileri                             │
 *  │  "geriGit"         → 2 saniye düz geri                              │
 *  │  "roll"            → Yerinde döner (0.5 sn)                         │
 *  │  "visionAlign"     → AprilTag'e göre hizalan + yaklaş               │
 *  │  "visionDrive"     → Görünen tag'e doğru git, X metre yakın dur     │
 *  │  "visionScore"     → Tag'e hizalan → yaklaş → dur (3 aşamalı)       │
 *  │  "tagAssistShoot"  → ID 9/10/25/26 tag'a hizalan + mesafede atış     │
 *  │  "TowerPreload"    → tower preload atışı + tag'e 1m yaklaşma         │
 *  └──────────────────────────────────────────────────────────────────────┘
 *
 *  Her mod STATE MACHINE ile yönetilir.
 *  Yeni mod eklemek için:
 *    1) run() switch bloğuna yeni case ekle
 *    2) Yeni private runXxx() metodu yaz
 */
public class autonom {

    // ═══════════════════════════════════════════════════════════════════════
    // AYARLAR
    // ═══════════════════════════════════════════════════════════════════════

    // ═══════════════════════════════════════════════════════════════════════
    // BAĞIMLILIKLAR — Robot.java constructor'dan enjekte edilir
    // ═══════════════════════════════════════════════════════════════════════
    private final DifferentialDrive drive;
    private final VisionSubsystem   vision;
    private final DoubleSupplier    yawSupplierDeg;
    private final ShooterSubsystem shooter;
    private final IntakeElevatorSubsystem intakeElevator;
    private final WPI_VictorSPX vcLever;
    private final SparkMax intakeL;
    private final SparkMax intakeR;
    private final BooleanSupplier blueTeamSupplier;
    private final BooleanSupplier redTeamSupplier;

    private enum AllianceSide {
        RED,
        BLUE
    }

    private static final int[] RED_SHOT_TAGS = {9, 10, 8, 5, 4, 3, 11, 2};
    private static final int[] BLUE_SHOT_TAGS = {25, 26, 18, 27, 19, 20, 21, 24};
    private static final int[] RED_HOME_TAGS = {12, 1, 11, 2, 9, 10, 7, 6};
    private static final int[] BLUE_HOME_TAGS = {22, 23, 21, 24, 25, 26, 17, 28};
    private static final int[] RED_HELPER_TAGS = {8, 5, 4, 3, 11, 2, 12, 1, 7, 6, 14, 13, 16, 15};
    private static final int[] BLUE_HELPER_TAGS = {18, 27, 19, 20, 21, 24, 22, 23, 17, 28, 29, 30, 31, 32};
    private static final int[] ALL_FIELD_TAGS = {
        1, 2, 3, 4, 5, 6, 7, 8,
        9, 10, 11, 12, 13, 14, 15, 16,
        17, 18, 19, 20, 21, 22, 23, 24,
        25, 26, 27, 28, 29, 30, 31, 32
    };
    private static final int[] TAG_ASSIST_TARGET_TAGS =
        RobotConfig.Autonomous.TAG_ASSIST_TARGET_IDS.clone();
    private static final int[] TOWE_PRELOAD_TARGET_TAGS =
        RobotConfig.Autonomous.TOWE_PRELOAD_TARGET_IDS.clone();

    // Gyro PID
    private final PIDController headingHoldPid = new PIDController(
        RobotConfig.Autonomous.GYRO_HEADING_KP,
        RobotConfig.Autonomous.GYRO_HEADING_KI,
        RobotConfig.Autonomous.GYRO_HEADING_KD
    );
    private final PIDController turnPid = new PIDController(
        RobotConfig.Autonomous.GYRO_TURN_KP,
        RobotConfig.Autonomous.GYRO_TURN_KI,
        RobotConfig.Autonomous.GYRO_TURN_KD
    );

    // ═══════════════════════════════════════════════════════════════════════
    // İÇ DURUM
    // ═══════════════════════════════════════════════════════════════════════
    private final Timer timer = new Timer();
    private final Timer totalTimer = new Timer();
    private int   phase       = 0;   // çok fazlı mod geçişi için
    private String activeMode = "";
    private double headingHoldSetpointDeg = 0.0;
    private double turnTargetYawDeg = 0.0;
    private boolean rollTargetInitialized = false;
    private double lastYawDeg = 0.0;
    private AllianceSide activeAlliance = AllianceSide.RED;
    private int tagAssistTargetId = -1;
    private double tagAssistStableStartSec = -1.0;
    private double tagAssistFeedStartSec = -1.0;
    private int towePreloadTargetId = -1;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * @param drive  Robot.java'daki m_drive nesnesi
     * @param vision VisionSubsystem nesnesi
     */
    public autonom(
        DifferentialDrive drive,
        VisionSubsystem vision,
        DoubleSupplier yawSupplierDeg,
        ShooterSubsystem shooter,
        IntakeElevatorSubsystem intakeElevator,
        WPI_VictorSPX vcLever,
        SparkMax intakeL,
        SparkMax intakeR,
        BooleanSupplier blueTeamSupplier,
        BooleanSupplier redTeamSupplier
    ) {
        this.drive  = drive;
        this.vision = vision;
        this.yawSupplierDeg = yawSupplierDeg;
        this.shooter = shooter;
        this.intakeElevator = intakeElevator;
        this.vcLever = vcLever;
        this.intakeL = intakeL;
        this.intakeR = intakeR;
        this.blueTeamSupplier = blueTeamSupplier;
        this.redTeamSupplier = redTeamSupplier;

        headingHoldPid.enableContinuousInput(-180.0, 180.0);
        headingHoldPid.setTolerance(RobotConfig.Autonomous.GYRO_HEADING_TOLERANCE_DEG);
        turnPid.enableContinuousInput(-180.0, 180.0);
        turnPid.setTolerance(RobotConfig.Autonomous.ROLL_TOLERANCE_DEG);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // INIT — Her otonom başlangıcında çağrılır
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Seçilen modu başlatır. Robot.java autonomousInit() içinden çağrılır.
     * @param mode Dashboard'dan seçilen mod stringi
     */
    public void init(String mode) {
        activeMode = (mode == null) ? RobotConfig.Autonomous.DEFAULT_MODE : mode;
        phase      = 0;
        timer.restart();
        totalTimer.restart();
        headingHoldSetpointDeg = getYawDeg();
        headingHoldPid.reset();
        turnPid.reset();
        rollTargetInitialized = false;
        activeAlliance = resolveAllianceSide();
        tagAssistTargetId = -1;
        tagAssistStableStartSec = -1.0;
        tagAssistFeedStartSec = -1.0;
        towePreloadTargetId = -1;
        setIntakeRollers(0.0);
        vcLever.set(0.0);
        shooter.stop();
        System.out.println("🤖 Otonom başladı → " + activeMode);
        
        // Auto mode status initialization
        SmartDashboard.putString("Auto/ActiveMode", activeMode);
        SmartDashboard.putNumber("Auto/Phase", phase);
        SmartDashboard.putString("Auto/Status", "Starting");
        SmartDashboard.putNumber("Auto/Timer", 0.0);
        SmartDashboard.putNumber("Auto/TotalTimer", 0.0);
        SmartDashboard.putNumber("Auto/GyroHeadingTarget_deg", headingHoldSetpointDeg);
        SmartDashboard.putString("Auto/Alliance", activeAlliance.name());
    }

    // ═══════════════════════════════════════════════════════════════════════
    // STEP — Her otonom periyodunda çağrılır (50 Hz / ~20 ms)
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Aktif modu bir adım ilerletir. Robot.java autonomousPeriodic() içinden çağrılır.
     */
    public void step() {
        SmartDashboard.putNumber("Auto/Timer",   timer.get());
        SmartDashboard.putNumber("Auto/TotalTimer", totalTimer.get());
        SmartDashboard.putNumber("Auto/Phase",   phase);
        SmartDashboard.putNumber("Auto/GyroYaw_deg", getYawDeg());
        
        // Update auto status based on mode
        SmartDashboard.putString("Auto/Mode", activeMode);

        switch (activeMode) {
            case RobotConfig.Autonomous.MODE_COLLECT_RETURN_SHOOT: runCollectReturnShoot(); break;
            case RobotConfig.Autonomous.MODE_DO_NOTHING:   runDoNothing();  break;
            case RobotConfig.Autonomous.MODE_FORWARD:     runIleri();       break;
            case RobotConfig.Autonomous.MODE_BACKWARD:    runGeri();        break;
            case RobotConfig.Autonomous.MODE_ROLL:        runRoll();        break;
            case RobotConfig.Autonomous.MODE_VISION_ALIGN:runVisionAlign(); break;
            case RobotConfig.Autonomous.MODE_VISION_DRIVE:runVisionDrive(); break;
            case RobotConfig.Autonomous.MODE_VISION_SCORE:runVisionScore(); break;
            case RobotConfig.Autonomous.MODE_TAG_ASSIST_SHOOT: runTagAssistShoot(); break;
            case RobotConfig.Autonomous.MODE_TOWE_PRELOAD: runTowePreload(); break;
            default:
                drive.tankDrive(0, 0);
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TEMEL SÜRÜŞ RUTINLERI
    // ═══════════════════════════════════════════════════════════════════════

    /** Hiçbir şey yapma (default). */
    private void runDoNothing() {
        drive.tankDrive(0, 0);
        SmartDashboard.putString("Auto/Status", "Do Nothing");
    }

    /** 2 saniye düz ileri */
    private void runIleri() {
        if (timer.get() < RobotConfig.Autonomous.FORWARD_DURATION_SEC) {
            driveStraightWithGyro(RobotConfig.Autonomous.FORWARD_POWER);
            SmartDashboard.putString("Auto/Status", "Forward (gyro heading hold)");
        } else {
            stop();
        }
    }

    /** 2 saniye düz geri */
    private void runGeri() {
        if (timer.get() < RobotConfig.Autonomous.BACKWARD_DURATION_SEC) {
            driveStraightWithGyro(RobotConfig.Autonomous.BACKWARD_POWER);
            SmartDashboard.putString("Auto/Status", "Backward (gyro heading hold)");
        } else {
            stop();
        }
    }

    /** Gyro PID ile hedef açıya yerinde dönüş */
    private void runRoll() {
        if (!rollTargetInitialized) {
            turnTargetYawDeg = wrapAngleDeg(getYawDeg() + RobotConfig.Autonomous.ROLL_TARGET_DELTA_DEG);
            rollTargetInitialized = true;
            turnPid.reset();
            timer.restart();
            SmartDashboard.putNumber("Auto/RollTargetYaw_deg", turnTargetYawDeg);
        }

        if (timer.get() > RobotConfig.Autonomous.ROLL_TIMEOUT_SEC) {
            System.out.println("⚠ roll: gyro turn timeout");
            stop();
            return;
        }

        double yaw = getYawDeg();
        double turnOutput = clamp(
            turnPid.calculate(yaw, turnTargetYawDeg),
            -RobotConfig.Autonomous.MAX_TURN_POWER,
            RobotConfig.Autonomous.MAX_TURN_POWER
        );

        SmartDashboard.putString("Auto/Status", "Roll (gyro turn PID)");
        SmartDashboard.putNumber("Auto/RollYaw_deg", yaw);
        SmartDashboard.putNumber("Auto/RollTurnOutput", turnOutput);
        SmartDashboard.putBoolean("Auto/RollAtSetpoint", turnPid.atSetpoint());

        if (turnPid.atSetpoint()) {
            System.out.println("✓ roll tamamlandı (gyro açıya ulaşıldı)");
            stop();
            return;
        }

        drive.tankDrive(-turnOutput, turnOutput);
    }

    /**
     * collectReturnShoot (default)
     * 0) Intake hazırlık
     * 1) Kısa top toplama sürüşü
     * 2) Alliance tarafına dönüş (ID + helper + dead-reckoning fallback)
     * 3) Kısa stabilizasyon
     * 4) 3 saniyede 3 farklı RPM şut denemesi
     * 5) Bekleme
     */
    private void runCollectReturnShoot() {
        if (totalTimer.get() >= RobotConfig.Autonomous.ROUTINE_MAX_TOTAL_SEC) {
            forceCompleteWait("CRS-GlobalTimeout");
            return;
        }

        activeAlliance = resolveAllianceSide();
        int[] shotTags = getShotTagsForAlliance(activeAlliance);
        int[] homeTags = getHomeTagsForAlliance(activeAlliance);
        int[] helperTags = getHelperTagsForAlliance(activeAlliance);
        int[] fallbackTags = buildFallbackTagPriority(homeTags, helperTags, shotTags);

        SmartDashboard.putString("Auto/Alliance", activeAlliance.name());
        SmartDashboard.putNumber("Auto/CRS/TotalElapsed_s", totalTimer.get());
        SmartDashboard.putNumber("Auto/CRS/PhaseElapsed_s", timer.get());

        switch (phase) {
            case 0:
                intakeElevator.setOpenPosition();
                setIntakeRollers(RobotConfig.Autonomous.SHOT_INTAKE_POWER);
                vcLever.set(0.0);
                shooter.stop();
                drive.tankDrive(0.0, 0.0);
                SmartDashboard.putString("Auto/Status", "CRS-Phase0 Prep Intake");
                boolean prepReady = intakeElevator.isAtTarget()
                    && timer.get() >= RobotConfig.Autonomous.COLLECT_PREP_SEC;
                boolean prepTimeout = timer.get() >= RobotConfig.Autonomous.COLLECT_PREP_MAX_SEC;
                SmartDashboard.putBoolean("Auto/CRS/PrepReady", prepReady);
                SmartDashboard.putBoolean("Auto/CRS/PrepTimeout", prepTimeout);
                if (prepReady || prepTimeout) {
                    phase = 1;
                    timer.restart();
                    headingHoldSetpointDeg = getYawDeg();
                }
                break;

            case 1:
                intakeElevator.setOpenPosition();
                setIntakeRollers(RobotConfig.Autonomous.SHOT_INTAKE_POWER);
                vcLever.set(0.0);
                shooter.stop();
                driveStraightWithGyro(RobotConfig.Autonomous.COLLECT_DRIVE_POWER);
                SmartDashboard.putString("Auto/Status", "CRS-Phase1 Collect Drive");
                if (timer.get() >= RobotConfig.Autonomous.COLLECT_DRIVE_SEC) {
                    phase = 2;
                    timer.restart();
                    headingHoldSetpointDeg = getYawDeg();
                }
                break;

            case 2:
                intakeElevator.setOpenPosition();
                setIntakeRollers(RobotConfig.Autonomous.SHOT_INTAKE_POWER);
                vcLever.set(0.0);
                shooter.stop();

                int returnTargetTag = pickPreferredVisibleTag(homeTags);
                TagAssistResult returnAssist = solveTagAssist(returnTargetTag, helperTags, fallbackTags);
                SmartDashboard.putNumber("Auto/CRS/ReturnTargetTag", returnTargetTag);
                SmartDashboard.putString("Auto/CRS/ReturnSource", returnAssist.source);
                SmartDashboard.putNumber("Auto/CRS/ReturnYawError_deg", returnAssist.yawErrorDeg);
                SmartDashboard.putNumber("Auto/CRS/ReturnDistance_m", returnAssist.distanceM);
                SmartDashboard.putBoolean("Auto/CRS/ReturnTagVisible", returnAssist.targetVisible);

                double leftOut = RobotConfig.Autonomous.RETURN_DRIVE_POWER;
                double rightOut = RobotConfig.Autonomous.RETURN_DRIVE_POWER;

                if (returnAssist.hasSolution) {
                    double correction = clamp(
                        returnAssist.yawErrorDeg * RobotConfig.Autonomous.RETURN_ALIGN_KP,
                        -RobotConfig.Autonomous.RETURN_ALIGN_MAX,
                        RobotConfig.Autonomous.RETURN_ALIGN_MAX
                    );
                    double baseReturnPower = RobotConfig.Autonomous.RETURN_DRIVE_POWER;
                    if (returnAssist.distanceM > 0.0
                        && returnAssist.distanceM < RobotConfig.Autonomous.RETURN_STOP_DISTANCE_M + 0.6) {
                        baseReturnPower = RobotConfig.Autonomous.RETURN_DRIVE_POWER * 0.72;
                    }
                    leftOut = clamp(baseReturnPower - correction, -1.0, 1.0);
                    rightOut = clamp(baseReturnPower + correction, -1.0, 1.0);
                    SmartDashboard.putString("Auto/CRS/ReturnDriveMode", "VisionAlignDrive");
                } else {
                    double[] blindDrive = getBlindReturnDrive(timer.get(), activeAlliance);
                    leftOut = blindDrive[0];
                    rightOut = blindDrive[1];
                    SmartDashboard.putString("Auto/CRS/ReturnDriveMode", "BlindFallback");
                }
                drive.tankDrive(leftOut, rightOut);

                boolean atReturnDistance = returnAssist.distanceM > 0.0
                    && returnAssist.distanceM <= RobotConfig.Autonomous.RETURN_STOP_DISTANCE_M;
                boolean returnTimedOut = timer.get() >= RobotConfig.Autonomous.RETURN_DRIVE_TIMEOUT_SEC;

                SmartDashboard.putBoolean("Auto/CRS/AtReturnDistance", atReturnDistance);
                SmartDashboard.putBoolean("Auto/CRS/ReturnTimedOut", returnTimedOut);

                if (atReturnDistance || returnTimedOut) {
                    phase = 3;
                    timer.restart();
                    drive.tankDrive(0.0, 0.0);
                    setIntakeRollers(0.0);
                    headingHoldSetpointDeg = getYawDeg();
                }
                break;

            case 3:
                drive.tankDrive(0.0, 0.0);
                setIntakeRollers(0.0);
                vcLever.set(0.0);
                shooter.stop();
                SmartDashboard.putString("Auto/Status", "CRS-Phase3 Settle");
                if (timer.get() >= RobotConfig.Autonomous.RETURN_WAIT_SETTLE_SEC) {
                    phase = 4;
                    timer.restart();
                    headingHoldSetpointDeg = getYawDeg();
                }
                break;

            case 4:
                runThreeStageShotSeries(shotTags, helperTags, fallbackTags);
                break;

            default:
                forceCompleteWait("CRS-Complete");
                break;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // VİZYON RUTINLERI
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * visionAlign
     * ─────────────────────────────────────────────────────────────────────
     * Ön kameradaki AprilTag'e hizalanır (yaw = 0).
     * Tag yoksa bekler; timeout 3 saniye.
     *
     *  Phase 0 → Hedef aranıyor / hizalanıyor
     *  Phase 1 → Hizalandı, dur
     */
    private void runVisionAlign() {
        if (phase == 1) { stop(); return; }

        // 3 saniye içinde bulamazsa iptal
        if (timer.get() > RobotConfig.Autonomous.ALIGN_TIMEOUT_SEC) {
            System.out.println("⚠ visionAlign: timeout, hedef bulunamadı");
            stop();
            phase = 1;
            return;
        }

        if (!vision.frontHasTarget()) {
            // Tag yok — yerinde bekle
            drive.tankDrive(0, 0);
            SmartDashboard.putString("Auto/Status", "Searching for target...");
            SmartDashboard.putBoolean("Auto/TargetFound", false);
            return;
        }
        
        SmartDashboard.putBoolean("Auto/TargetFound", true);

        double yaw    = vision.getFrontYaw();
        double error  = -yaw;                       // Pozitif hata = sola dön
        double output = clamp(
            error * RobotConfig.Autonomous.KP_ALIGN,
            -RobotConfig.Autonomous.MAX_TURN_POWER,
            RobotConfig.Autonomous.MAX_TURN_POWER
        );

        SmartDashboard.putNumber("Auto/AlignError_deg", yaw);
        SmartDashboard.putNumber("Auto/AlignOutput",    output);
        SmartDashboard.putString("Auto/Status",         "Aligning to target...");
        SmartDashboard.putBoolean("Auto/Aligned", Math.abs(yaw) < RobotConfig.Autonomous.ALIGN_TOLERANCE_DEG);

        if (Math.abs(yaw) < RobotConfig.Autonomous.ALIGN_TOLERANCE_DEG) {
            System.out.println("✓ visionAlign tamamlandı (yaw=" + String.format("%.2f", yaw) + "°)");
            stop();
            phase = 1;
        } else {
            // Yerinde dön
            drive.tankDrive(-output, output);
        }
    }

    /**
     * visionDrive
     * ─────────────────────────────────────────────────────────────────────
     * Ön kameradaki AprilTag'e hizalan, sonra TARGET_STOP_DISTANCE_M kadar
     * yaklaşana kadar ilerle.
     *
     *  Phase 0 → Hizalanıyor (yaw düzeltmesi)
     *  Phase 1 → İlerliyor (mesafe kapatılıyor)
     *  Phase 2 → Tamamlandı
     */
    private void runVisionDrive() {
        if (phase == 2) { stop(); return; }

        // Genel zaman aşımı: 6 saniye
        if (timer.get() > RobotConfig.Autonomous.DRIVE_TIMEOUT_SEC) {
            System.out.println("⚠ visionDrive: timeout");
            stop();
            phase = 2;
            return;
        }

        if (!vision.frontHasTarget()) {
            drive.tankDrive(0, 0);
            SmartDashboard.putString("Auto/Status", "Waiting for target...");
            SmartDashboard.putBoolean("Auto/TargetFound", false);
            return;
        }
        
        SmartDashboard.putBoolean("Auto/TargetFound", true);

        double yaw      = vision.getFrontYaw();
        double distance = vision.getFrontDistance();

        SmartDashboard.putNumber("Auto/Yaw_deg",      yaw);
        SmartDashboard.putNumber("Auto/Distance_m",   distance);
        SmartDashboard.putNumber("Auto/TargetDist_m", RobotConfig.Autonomous.TARGET_STOP_DISTANCE_M);
        SmartDashboard.putNumber("Auto/DistanceError", distance - RobotConfig.Autonomous.TARGET_STOP_DISTANCE_M);

        // ─── Phase 0: Hizalan ───
        if (phase == 0) {
            double turnOutput = clamp(
                -yaw * RobotConfig.Autonomous.KP_ALIGN,
                -RobotConfig.Autonomous.MAX_TURN_POWER,
                RobotConfig.Autonomous.MAX_TURN_POWER
            );
            SmartDashboard.putString("Auto/Status", "Phase1: Aligning to target");
            SmartDashboard.putBoolean("Auto/Aligned", Math.abs(yaw) < RobotConfig.Autonomous.ALIGN_TOLERANCE_DEG);

            if (Math.abs(yaw) < RobotConfig.Autonomous.ALIGN_TOLERANCE_DEG) {
                System.out.println("→ visionDrive: hizalandı, ilerlemeye başlanıyor");
                phase = 1;
                timer.restart();   // Phase 1 için timer'ı sıfırla
            } else {
                drive.tankDrive(-turnOutput, turnOutput);
            }
            return;
        }

        // ─── Phase 1: İlerle ───
        if (phase == 1) {
            double distError  = distance - RobotConfig.Autonomous.TARGET_STOP_DISTANCE_M;
            double driveOut   = clamp(
                distError * RobotConfig.Autonomous.KP_DRIVE,
                -RobotConfig.Autonomous.MAX_DRIVE_POWER,
                RobotConfig.Autonomous.MAX_DRIVE_POWER
            );
            double turnOutput = clamp(
                -yaw * RobotConfig.Autonomous.KP_ALIGN * RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_GAIN,
                -RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT,
                RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT
            ); // Hafif düzeltme

            SmartDashboard.putString("Auto/Status", "Phase2: Driving to target");
            SmartDashboard.putNumber("Auto/DistError_m", distError);
            SmartDashboard.putBoolean("Auto/AtDistance", distError < RobotConfig.Autonomous.DRIVE_DEADZONE_M);

            if (distError < RobotConfig.Autonomous.DRIVE_DEADZONE_M) {
                System.out.println("✓ visionDrive tamamlandı (mesafe=" + String.format("%.2f", distance) + " m)");
                stop();
                phase = 2;
            } else {
                // Sol/sağ farklı güç ile hem ilerle hem hizayı koru
                drive.tankDrive(driveOut - turnOutput, driveOut + turnOutput);
            }
        }
    }

    /**
     * visionScore
     * ─────────────────────────────────────────────────────────────────────
     * Tam skor sekansı: Hizalan → Yaklaş → Dur (oyun piyesi bırak)
     *
     *  Phase 0 → 1 sn ileri sürerek tag'e yaklaş (kör yaklaşım)
     *  Phase 1 → visionAlign ile hizalan
     *  Phase 2 → visionDrive ile yaklaş
     *  Phase 3 → Tamamlandı, dur
     */
    private void runVisionScore() {
        if (phase == 3) { stop(); return; }

        SmartDashboard.putNumber("Auto/Phase", phase);

        switch (phase) {

            // ─── Phase 0: Kör yaklaşım (1 saniye ileri) ───
            case 0:
                SmartDashboard.putString("Auto/Status", "Phase0: Initial approach");
                SmartDashboard.putBoolean("Auto/PhaseComplete", false);
                if (timer.get() < RobotConfig.Autonomous.SCORE_PHASE0_DURATION_SEC) {
                    drive.tankDrive(RobotConfig.Autonomous.SCORE_PHASE0_POWER, RobotConfig.Autonomous.SCORE_PHASE0_POWER);
                } else {
                    System.out.println("→ visionScore Phase 1: hizalama");
                    phase = 1;
                    timer.restart();
                }
                break;

            // ─── Phase 1: Hizala ───
            case 1:
                SmartDashboard.putString("Auto/Status", "Phase1: Aligning to target");
                SmartDashboard.putBoolean("Auto/PhaseComplete", false);
                if (timer.get() > RobotConfig.Autonomous.SCORE_PHASE1_TIMEOUT_SEC) {
                    System.out.println("→ visionScore Phase 1 timeout, Phase 2'ye geç");
                    phase = 2;
                    timer.restart();
                    break;
                }
                if (!vision.frontHasTarget()) {
                    drive.tankDrive(0, 0);
                    break;
                }
                double yaw1 = vision.getFrontYaw();
                if (Math.abs(yaw1) < RobotConfig.Autonomous.ALIGN_TOLERANCE_DEG) {
                    System.out.println("→ visionScore Phase 2: yaklaşım");
                    phase = 2;
                    timer.restart();
                } else {
                    double t1 = clamp(
                        -yaw1 * RobotConfig.Autonomous.KP_ALIGN,
                        -RobotConfig.Autonomous.MAX_TURN_POWER,
                        RobotConfig.Autonomous.MAX_TURN_POWER
                    );
                    drive.tankDrive(-t1, t1);
                }
                break;

            // ─── Phase 2: Yaklaş ───
            case 2:
                SmartDashboard.putString("Auto/Status", "Phase2: Approaching target");
                SmartDashboard.putBoolean("Auto/PhaseComplete", false);
                if (timer.get() > RobotConfig.Autonomous.SCORE_PHASE2_TIMEOUT_SEC) {
                    System.out.println("→ visionScore Phase 2 timeout, tamamlandı");
                    stop();
                    phase = 3;
                    break;
                }
                if (!vision.frontHasTarget()) {
                    drive.tankDrive(0, 0);
                    break;
                }
                double dist2   = vision.getFrontDistance();
                double yaw2    = vision.getFrontYaw();
                double dErr    = dist2 - RobotConfig.Autonomous.TARGET_STOP_DISTANCE_M;
                double dOut    = clamp(
                    dErr * RobotConfig.Autonomous.KP_DRIVE,
                    -RobotConfig.Autonomous.MAX_DRIVE_POWER,
                    RobotConfig.Autonomous.MAX_DRIVE_POWER
                );
                double tCorr   = clamp(
                    -yaw2 * RobotConfig.Autonomous.KP_ALIGN * RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_GAIN,
                    -RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT,
                    RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT
                );

                SmartDashboard.putNumber("Auto/Distance_m", dist2);
                SmartDashboard.putNumber("Auto/DistanceError", dErr);
                SmartDashboard.putBoolean("Auto/AtDistance", dErr < RobotConfig.Autonomous.DRIVE_DEADZONE_M);

                if (dErr < RobotConfig.Autonomous.DRIVE_DEADZONE_M) {
                    System.out.println("✓ visionScore tamamlandı!");
                    stop();
                    phase = 3;
                } else {
                    drive.tankDrive(dOut - tCorr, dOut + tCorr);
                }
                break;

            default:
                stop();
                phase = 3;
                break;
        }
    }

    /**
     * tagAssistShoot
     * ─────────────────────────────────────────────────────────────────────
     * Sadece 9/10/25/26 tag'lerinden birini gördüğünde hedefe hizalanır,
     * mesafeyi dashboard'a yazar ve mesafe aralığı uygunsa otomatik atış yapar.
     *
     * Phase 0 → Hedef tag bul / hizalan / mesafeyi ayarla
     * Phase 1 → Shooter hazırken feed yap
     * Phase 2 → Tamamlandı
     */
    private void runTagAssistShoot() {
        if (phase >= 2) {
            stop();
            stopScoringActuators();
            return;
        }

        if (totalTimer.get() > RobotConfig.Autonomous.TAG_ASSIST_TIMEOUT_SEC) {
            stop();
            stopScoringActuators();
            phase = 2;
            SmartDashboard.putString("Auto/Status", "TagAssistShoot Timeout");
            SmartDashboard.putString("Auto/TagAssistShoot/Result", "Timeout");
            return;
        }

        int visibleTarget = pickFirstVisibleTag(TAG_ASSIST_TARGET_TAGS);
        if (visibleTarget > 0) {
            tagAssistTargetId = visibleTarget;
        } else if (tagAssistTargetId > 0 && !vision.isTagVisible(tagAssistTargetId)) {
            tagAssistTargetId = -1;
        }

        if (tagAssistTargetId <= 0) {
            stopScoringActuators();
            tagAssistStableStartSec = -1.0;
            tagAssistFeedStartSec = -1.0;
            drive.tankDrive(
                -RobotConfig.Autonomous.TAG_ASSIST_SEARCH_SPIN_POWER,
                RobotConfig.Autonomous.TAG_ASSIST_SEARCH_SPIN_POWER
            );

            SmartDashboard.putString("Auto/Status", "TagAssistShoot Searching");
            SmartDashboard.putNumber("Auto/TagAssistShoot/TargetId", -1);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/Visible", false);
            SmartDashboard.putNumber("Auto/TagAssistShoot/Distance_m", -1.0);
            SmartDashboard.putNumber("Auto/TagAssistShoot/YawError_deg", 0.0);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/InRange", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/Aligned", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/ShooterAtSpeed", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/CanShoot", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/FeedActive", false);
            return;
        }

        double distanceM = vision.getDistanceToTag(tagAssistTargetId);
        double yawErrorDeg = vision.getYawToTag(tagAssistTargetId);
        if (distanceM < 0.0) {
            stopScoringActuators();
            tagAssistStableStartSec = -1.0;
            tagAssistFeedStartSec = -1.0;
            drive.tankDrive(
                -RobotConfig.Autonomous.TAG_ASSIST_SEARCH_SPIN_POWER,
                RobotConfig.Autonomous.TAG_ASSIST_SEARCH_SPIN_POWER
            );
            SmartDashboard.putString("Auto/Status", "TagAssistShoot Lost Target");
            SmartDashboard.putNumber("Auto/TagAssistShoot/TargetId", tagAssistTargetId);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/Visible", false);
            SmartDashboard.putNumber("Auto/TagAssistShoot/Distance_m", -1.0);
            SmartDashboard.putNumber("Auto/TagAssistShoot/YawError_deg", 0.0);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/InRange", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/Aligned", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/ShooterAtSpeed", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/CanShoot", false);
            SmartDashboard.putBoolean("Auto/TagAssistShoot/FeedActive", false);
            return;
        }

        double distanceErrorM = 0.0;
        if (distanceM > RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MAX_DISTANCE_M) {
            distanceErrorM = distanceM - RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MAX_DISTANCE_M;
        } else if (distanceM < RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MIN_DISTANCE_M) {
            distanceErrorM = distanceM - RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MIN_DISTANCE_M;
        }

        double driveOutput = clamp(
            distanceErrorM * RobotConfig.Autonomous.TAG_ASSIST_DRIVE_KP,
            -RobotConfig.Autonomous.TAG_ASSIST_MAX_DRIVE_POWER,
            RobotConfig.Autonomous.TAG_ASSIST_MAX_DRIVE_POWER
        );
        double turnOutput = clamp(
            -yawErrorDeg * RobotConfig.Autonomous.KP_ALIGN,
            -RobotConfig.Autonomous.MAX_TURN_POWER,
            RobotConfig.Autonomous.MAX_TURN_POWER
        );
        double leftOut = clamp(driveOutput - turnOutput, -1.0, 1.0);
        double rightOut = clamp(driveOutput + turnOutput, -1.0, 1.0);

        boolean aligned = Math.abs(yawErrorDeg) <= RobotConfig.Autonomous.ALIGN_TOLERANCE_DEG;
        boolean inRange = distanceM >= RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MIN_DISTANCE_M
            && distanceM <= RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MAX_DISTANCE_M;
        boolean atShootPose = aligned && inRange;

        if (phase == 0) {
            shooter.setTargetRPM(RobotConfig.Autonomous.TAG_ASSIST_SHOOT_RPM);
            vcLever.set(0.0);
            setIntakeRollers(0.0);
            drive.tankDrive(leftOut, rightOut);

            if (atShootPose) {
                if (tagAssistStableStartSec < 0.0) {
                    tagAssistStableStartSec = timer.get();
                }
                if ((timer.get() - tagAssistStableStartSec) >= RobotConfig.Autonomous.TAG_ASSIST_STABLE_TIME_SEC) {
                    phase = 1;
                    timer.restart();
                    tagAssistFeedStartSec = -1.0;
                }
            } else {
                tagAssistStableStartSec = -1.0;
            }
        } else {
            shooter.setTargetRPM(RobotConfig.Autonomous.TAG_ASSIST_SHOOT_RPM);
            drive.tankDrive(leftOut, rightOut);

            boolean shooterAtSpeed = shooter.isAtSpeed();
            boolean canShoot = atShootPose && shooterAtSpeed;
            boolean feedWindowActive = tagAssistFeedStartSec >= 0.0
                && (timer.get() - tagAssistFeedStartSec) < RobotConfig.Autonomous.TAG_ASSIST_FEED_DURATION_SEC;

            if (canShoot || feedWindowActive) {
                vcLever.set(RobotConfig.Autonomous.SHOT_LEVER_POWER);
                setIntakeRollers(RobotConfig.Autonomous.SHOT_INTAKE_POWER);
                if (tagAssistFeedStartSec < 0.0) {
                    tagAssistFeedStartSec = timer.get();
                }
            } else {
                vcLever.set(0.0);
                setIntakeRollers(0.0);
            }

            if (tagAssistFeedStartSec >= 0.0
                && (timer.get() - tagAssistFeedStartSec) >= RobotConfig.Autonomous.TAG_ASSIST_FEED_DURATION_SEC) {
                phase = 2;
                stop();
                stopScoringActuators();
                SmartDashboard.putString("Auto/Status", "TagAssistShoot Complete");
                SmartDashboard.putString("Auto/TagAssistShoot/Result", "ShotFired");
                return;
            }
        }

        boolean shooterAtSpeed = shooter.isAtSpeed();
        boolean canShoot = atShootPose && shooterAtSpeed;
        boolean feedActive = tagAssistFeedStartSec >= 0.0
            && phase == 1
            && (timer.get() - tagAssistFeedStartSec) <= RobotConfig.Autonomous.TAG_ASSIST_FEED_DURATION_SEC;

        SmartDashboard.putString("Auto/Status", phase == 0
            ? "TagAssistShoot Aligning"
            : "TagAssistShoot Shooting");
        SmartDashboard.putNumber("Auto/TagAssistShoot/TargetId", tagAssistTargetId);
        SmartDashboard.putBoolean("Auto/TagAssistShoot/Visible", true);
        SmartDashboard.putNumber("Auto/TagAssistShoot/Distance_m", distanceM);
        SmartDashboard.putNumber("Auto/TagAssistShoot/YawError_deg", yawErrorDeg);
        SmartDashboard.putNumber("Auto/TagAssistShoot/DriveOut", driveOutput);
        SmartDashboard.putNumber("Auto/TagAssistShoot/TurnOut", turnOutput);
        SmartDashboard.putBoolean("Auto/TagAssistShoot/InRange", inRange);
        SmartDashboard.putBoolean("Auto/TagAssistShoot/Aligned", aligned);
        SmartDashboard.putBoolean("Auto/TagAssistShoot/ShooterAtSpeed", shooterAtSpeed);
        SmartDashboard.putBoolean("Auto/TagAssistShoot/CanShoot", canShoot);
        SmartDashboard.putBoolean("Auto/TagAssistShoot/FeedActive", feedActive);
    }

    private void runTowePreload() {
        if (phase >= 5) {
            stop();
            stopScoringActuators();
            intakeElevator.setClosePosition();
            SmartDashboard.putString("Auto/Status", "TowePreload Complete");
            return;
        }

        if (totalTimer.get() > RobotConfig.Autonomous.TOWE_PRELOAD_TIMEOUT_SEC) {
            phase = 5;
            stop();
            stopScoringActuators();
            intakeElevator.setClosePosition();
            SmartDashboard.putString("Auto/Status", "TowePreload Timeout");
            return;
        }

        if (phase == 0) {
            stopScoringActuators();
            intakeElevator.setClosePosition();
            if (timer.get() < RobotConfig.Autonomous.TOWE_PRELOAD_BACKUP_SEC) {
                // TowerPreload'da gyro/encoder kullanılmaz: sadece açık çevrim geri sürüş.
                drive.tankDrive(
                    RobotConfig.Autonomous.TOWE_PRELOAD_BACKUP_POWER,
                    RobotConfig.Autonomous.TOWE_PRELOAD_BACKUP_POWER
                );
                SmartDashboard.putString("Auto/Status", "TowerPreload Backward");
                return;
            }
            phase = 1;
            timer.restart();
        }

        // Bu modda yalnızca FRONT camera kullanılır.
        int visibleTarget = -1;
        for (int tagId : TOWE_PRELOAD_TARGET_TAGS) {
            if (vision.isFrontTagVisible(tagId)) {
                visibleTarget = tagId;
                break;
            }
        }
        if (visibleTarget > 0) {
            towePreloadTargetId = visibleTarget;
        } else if (towePreloadTargetId > 0 && !vision.isFrontTagVisible(towePreloadTargetId)) {
            towePreloadTargetId = -1;
        }

        if (towePreloadTargetId <= 0) {
            stopScoringActuators();
            drive.tankDrive(
                -RobotConfig.Autonomous.TOWE_PRELOAD_SEARCH_SPIN_POWER,
                RobotConfig.Autonomous.TOWE_PRELOAD_SEARCH_SPIN_POWER
            );
            SmartDashboard.putString("Auto/Status", "TowePreload Searching 10/25");
            SmartDashboard.putNumber("Auto/TowePreload/TargetId", -1);
            SmartDashboard.putNumber("Auto/TowePreload/Distance_m", -1.0);
            return;
        }

        double distanceM = vision.getFrontDistanceToTag(towePreloadTargetId);
        double yawDeg = vision.getFrontYawToTag(towePreloadTargetId);
        if (distanceM < 0.0) {
            drive.tankDrive(
                -RobotConfig.Autonomous.TOWE_PRELOAD_SEARCH_SPIN_POWER,
                RobotConfig.Autonomous.TOWE_PRELOAD_SEARCH_SPIN_POWER
            );
            SmartDashboard.putString("Auto/Status", "TowePreload Lost Tag");
            SmartDashboard.putNumber("Auto/TowePreload/TargetId", towePreloadTargetId);
            SmartDashboard.putNumber("Auto/TowePreload/Distance_m", -1.0);
            return;
        }

        double yMin = RobotConfig.Autonomous.TOWE_PRELOAD_Y_MIN_M;
        double yMax = RobotConfig.Autonomous.TOWE_PRELOAD_Y_MAX_M;
        boolean inWindow = distanceM >= yMin && distanceM <= yMax;

        // Tag görüldükten sonra "hafif" değil, belirgin geri/ileri komut ver.
        double driveOut = 0.0;
        if (distanceM < yMin) {
            driveOut = RobotConfig.Autonomous.TOWE_PRELOAD_BACKUP_POWER; // geri
        } else if (distanceM > yMax) {
            driveOut = Math.min(
                RobotConfig.Autonomous.TOWE_PRELOAD_MAX_DRIVE_POWER,
                Math.abs(RobotConfig.Autonomous.TOWE_PRELOAD_BACKUP_POWER)
            ); // ileri
        }
        double turnOut = clamp(
            -yawDeg * RobotConfig.Autonomous.KP_ALIGN,
            -RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT,
            RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT
        );

        if (phase == 1) {
            drive.tankDrive(
                clamp(driveOut - turnOut, -1.0, 1.0),
                clamp(driveOut + turnOut, -1.0, 1.0)
            );
            stopScoringActuators();
            intakeElevator.setClosePosition();
            SmartDashboard.putString("Auto/Status", "TowerPreload Set Distance");
            SmartDashboard.putNumber("Auto/TowePreload/TargetId", towePreloadTargetId);
            SmartDashboard.putNumber("Auto/TowePreload/Distance_m", distanceM);
            SmartDashboard.putNumber("Auto/TowePreload/DriveOut", driveOut);
            SmartDashboard.putNumber("Auto/TowePreload/TurnOut", turnOut);
            SmartDashboard.putBoolean("Auto/TowePreload/InYWindow", inWindow);

            if (inWindow) {
                phase = 2;
                timer.restart();
            }
            return;
        }

        if (phase == 2) {
            intakeElevator.setOpenPosition();
            shooter.setTargetRPM(RobotConfig.Shooter.MAX_RPM);

            if (timer.get() < RobotConfig.Autonomous.TOWE_PRELOAD_INTAKE_PRIME_SEC) {
                setIntakeRollers(RobotConfig.Autonomous.TOWE_PRELOAD_INTAKE_POWER);
                vcLever.set(0.0);
                drive.tankDrive(0.0, 0.0);
                SmartDashboard.putString("Auto/Status", "TowePreload Intake Prime");
                return;
            }

            setIntakeRollers(0.0);
            vcLever.set(RobotConfig.Autonomous.TOWE_PRELOAD_VC_LEVER_POWER);
            drive.tankDrive(0.0, 0.0);
            SmartDashboard.putString("Auto/Status", "TowePreload Shooting");

            if (timer.get() >= RobotConfig.Autonomous.TOWE_PRELOAD_INTAKE_PRIME_SEC
                + RobotConfig.Autonomous.TOWE_PRELOAD_SHOOT_DURATION_SEC) {
                vcLever.set(0.0);
                shooter.stop();
                setIntakeRollers(0.0);
                intakeElevator.setClosePosition();
                phase = 3;
                timer.restart();
            }
            return;
        }

        if (phase == 3) {
            double stopAt = RobotConfig.Autonomous.TOWE_PRELOAD_APPROACH_STOP_M;
            double err = distanceM - stopAt;

            if (err <= 0.0) {
                phase = 5;
                stop();
                stopScoringActuators();
                intakeElevator.setClosePosition();
                SmartDashboard.putString("Auto/Status", "TowePreload Reached <1m");
                SmartDashboard.putNumber("Auto/TowePreload/Distance_m", distanceM);
                return;
            }

            double approachDrive = clamp(
                err * RobotConfig.Autonomous.KP_DRIVE,
                0.0,
                RobotConfig.Autonomous.MAX_DRIVE_POWER
            );
            double approachTurn = clamp(
                -yawDeg * RobotConfig.Autonomous.KP_ALIGN * RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_GAIN,
                -RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT,
                RobotConfig.Autonomous.DRIVE_TURN_CORRECTION_LIMIT
            );

            drive.tankDrive(
                clamp(approachDrive - approachTurn, -1.0, 1.0),
                clamp(approachDrive + approachTurn, -1.0, 1.0)
            );
            SmartDashboard.putString("Auto/Status", "TowePreload Final Approach");
            SmartDashboard.putNumber("Auto/TowePreload/TargetId", towePreloadTargetId);
            SmartDashboard.putNumber("Auto/TowePreload/Distance_m", distanceM);
            SmartDashboard.putNumber("Auto/TowePreload/ApproachError_m", err);
            return;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // YARDIMCI METODLAR
    // ═══════════════════════════════════════════════════════════════════════

    /** Motorları durdurur. */
    public void stop() {
        drive.tankDrive(0, 0);
    }

    /** Aktif modu döndürür (debug/dashboard için). */
    public String getActiveMode() { return activeMode; }

    /** Mevcut aşamayı döndürür. */
    public int getPhase() { return phase; }

    private void runThreeStageShotSeries(int[] shotTags, int[] helperTags, int[] fallbackTags) {
        double elapsed = timer.get();
        if (elapsed >= RobotConfig.Autonomous.SHOT_SEQUENCE_TOTAL_SEC) {
            phase = 5;
            timer.restart();
            forceCompleteWait("CRS-ShotSequenceDone");
            return;
        }

        int shotIndex = (int) Math.floor(elapsed / RobotConfig.Autonomous.SHOT_WINDOW_SEC);
        shotIndex = Math.max(0, Math.min(2, shotIndex));
        double targetRpm = getShotRpmForIndex(shotIndex);
        shooter.setTargetRPM(targetRpm);
        intakeElevator.setOpenPosition();

        int shotTargetTag = pickPreferredVisibleTag(shotTags);
        TagAssistResult shotAssist = solveTagAssist(shotTargetTag, helperTags, fallbackTags);

        double shotWindowStart = shotIndex * RobotConfig.Autonomous.SHOT_WINDOW_SEC;
        double localShotTime = elapsed - shotWindowStart;
        boolean feedActive = localShotTime >= (
            RobotConfig.Autonomous.SHOT_WINDOW_SEC - RobotConfig.Autonomous.SHOT_FEED_ACTIVE_SEC
        );

        double forwardBias = feedActive ? 0.0 : getShotApproachPower(shotIndex);
        double turnOutput = 0.0;
        if (shotAssist.hasSolution) {
            turnOutput = clamp(
                shotAssist.yawErrorDeg * RobotConfig.Autonomous.SHOT_ALIGN_KP,
                -RobotConfig.Autonomous.SHOT_ALIGN_MAX,
                RobotConfig.Autonomous.SHOT_ALIGN_MAX
            );
        } else if (vision.hasAnyTarget()) {
            double yaw = vision.frontHasTarget() ? vision.getFrontYaw() : vision.getBackYaw();
            turnOutput = clamp(
                yaw * RobotConfig.Autonomous.SHOT_ALIGN_KP,
                -RobotConfig.Autonomous.SHOT_ALIGN_MAX,
                RobotConfig.Autonomous.SHOT_ALIGN_MAX
            );
        } else if (!feedActive) {
            // Hiç tag yoksa, yeni hedef yakalamak için düşük hız search spin.
            turnOutput = activeAlliance == AllianceSide.BLUE
                ? RobotConfig.Autonomous.SHOT_SEARCH_SPIN_POWER
                : -RobotConfig.Autonomous.SHOT_SEARCH_SPIN_POWER;
        }
        double leftOut = clamp(forwardBias - turnOutput, -1.0, 1.0);
        double rightOut = clamp(forwardBias + turnOutput, -1.0, 1.0);
        drive.tankDrive(leftOut, rightOut);

        if (feedActive) {
            vcLever.set(RobotConfig.Autonomous.SHOT_LEVER_POWER);
            setIntakeRollers(RobotConfig.Autonomous.SHOT_INTAKE_POWER);
        } else {
            vcLever.set(0.0);
            setIntakeRollers(0.0);
        }

        SmartDashboard.putString("Auto/Status", "CRS-Phase4 Shot Sequence");
        SmartDashboard.putNumber("Auto/CRS/ShotIndex", shotIndex + 1);
        SmartDashboard.putNumber("Auto/CRS/ShotTargetRPM", targetRpm);
        SmartDashboard.putNumber("Auto/CRS/ShotTargetTag", shotTargetTag);
        SmartDashboard.putNumber("Auto/CRS/ShotYawError_deg", shotAssist.yawErrorDeg);
        SmartDashboard.putString("Auto/CRS/ShotSource", shotAssist.source);
        SmartDashboard.putBoolean("Auto/CRS/FeedActive", feedActive);
        SmartDashboard.putNumber("Auto/CRS/ShotForwardBias", forwardBias);
        SmartDashboard.putNumber("Auto/CRS/ShotTurnOutput", turnOutput);
    }

    private double getShotRpmForIndex(int shotIndex) {
        if (shotIndex == 0) {
            return RobotConfig.Shooter.MAX_RPM * RobotConfig.Autonomous.SHOT_RPM_SCALE_CLOSE;
        }
        if (shotIndex == 1) {
            return RobotConfig.Shooter.MAX_RPM * RobotConfig.Autonomous.SHOT_RPM_SCALE_MID;
        }
        return RobotConfig.Shooter.MAX_RPM * RobotConfig.Autonomous.SHOT_RPM_SCALE_FAR;
    }

    private double getShotApproachPower(int shotIndex) {
        if (shotIndex == 0) {
            return RobotConfig.Autonomous.SHOT_CLOSE_APPROACH_POWER;
        }
        if (shotIndex == 1) {
            return RobotConfig.Autonomous.SHOT_MID_APPROACH_POWER;
        }
        return RobotConfig.Autonomous.SHOT_FAR_APPROACH_POWER;
    }

    private AllianceSide resolveAllianceSide() {
        if (blueTeamSupplier.getAsBoolean()) {
            return AllianceSide.BLUE;
        }
        if (redTeamSupplier.getAsBoolean()) {
            return AllianceSide.RED;
        }

        Optional<DriverStation.Alliance> dsAlliance = DriverStation.getAlliance();
        if (dsAlliance.isPresent()) {
            return dsAlliance.get() == DriverStation.Alliance.Blue
                ? AllianceSide.BLUE
                : AllianceSide.RED;
        }

        boolean blueVisible = vision.isAnyTagVisible(BLUE_HOME_TAGS)
            || vision.isAnyTagVisible(BLUE_SHOT_TAGS);
        boolean redVisible = vision.isAnyTagVisible(RED_HOME_TAGS)
            || vision.isAnyTagVisible(RED_SHOT_TAGS);

        if (blueVisible && !redVisible) {
            return AllianceSide.BLUE;
        }
        if (redVisible && !blueVisible) {
            return AllianceSide.RED;
        }
        if (blueVisible && redVisible) {
            // Orta alanda iki taraf da görünüyorsa alliance'ı zıplatma.
            return activeAlliance;
        }
        return activeAlliance;
    }

    private int[] getShotTagsForAlliance(AllianceSide alliance) {
        return alliance == AllianceSide.BLUE ? BLUE_SHOT_TAGS : RED_SHOT_TAGS;
    }

    private int[] getHomeTagsForAlliance(AllianceSide alliance) {
        return alliance == AllianceSide.BLUE ? BLUE_HOME_TAGS : RED_HOME_TAGS;
    }

    private int[] getHelperTagsForAlliance(AllianceSide alliance) {
        return alliance == AllianceSide.BLUE ? BLUE_HELPER_TAGS : RED_HELPER_TAGS;
    }

    private int[] buildFallbackTagPriority(int[] homeTags, int[] helperTags, int[] shotTags) {
        boolean[] used = new boolean[33];
        int[] merged = new int[homeTags.length + helperTags.length + shotTags.length + ALL_FIELD_TAGS.length];
        int idx = 0;

        idx = appendUniqueTags(homeTags, used, merged, idx);
        idx = appendUniqueTags(helperTags, used, merged, idx);
        idx = appendUniqueTags(shotTags, used, merged, idx);
        idx = appendUniqueTags(ALL_FIELD_TAGS, used, merged, idx);

        int[] out = new int[idx];
        System.arraycopy(merged, 0, out, 0, idx);
        return out;
    }

    private static int appendUniqueTags(int[] source, boolean[] used, int[] out, int idx) {
        for (int tag : source) {
            if (tag > 0 && tag < used.length && !used[tag]) {
                used[tag] = true;
                out[idx++] = tag;
            }
        }
        return idx;
    }

    private int pickPreferredVisibleTag(int[] priorityTags) {
        int visibleTag = pickFirstVisibleTag(priorityTags);
        if (visibleTag > 0) {
            return visibleTag;
        }
        return priorityTags.length > 0 ? priorityTags[0] : -1;
    }

    private int pickFirstVisibleTag(int[] priorityTags) {
        for (int tagId : priorityTags) {
            if (vision.isTagVisible(tagId)) {
                return tagId;
            }
        }
        return -1;
    }

    private TagAssistResult solveTagAssist(int targetTagId, int[] helperTagIds, int[] fallbackTagIds) {
        boolean helperVisible = vision.isAnyTagVisible(helperTagIds);

        if (targetTagId > 0) {
            double targetDistance = vision.getDistanceToTag(targetTagId);
            if (targetDistance >= 0.0) {
                return new TagAssistResult(
                    true,
                    true,
                    helperVisible,
                    vision.getYawToTag(targetTagId),
                    targetDistance,
                    "DirectTagYaw"
                );
            }

            Pose2d estimatedPose = vision.getBestEstimatedPose();
            Optional<Pose2d> targetPoseOpt = vision.getFieldTagPose2d(targetTagId);
            if (estimatedPose != null && targetPoseOpt.isPresent() && helperVisible) {
                Pose2d tagPose = targetPoseOpt.get();
                double dx = tagPose.getX() - estimatedPose.getX();
                double dy = tagPose.getY() - estimatedPose.getY();
                double desiredHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
                double currentHeadingDeg = estimatedPose.getRotation().getDegrees();
                double yawErrorDeg = wrapAngleDeg(desiredHeadingDeg - currentHeadingDeg);
                double estimatedDistance = Math.hypot(dx, dy);

                return new TagAssistResult(
                    true,
                    false,
                    true,
                    yawErrorDeg,
                    estimatedDistance,
                    "HelperPoseToTag"
                );
            }
        }

        int helperDirectTag = pickFirstVisibleTag(helperTagIds);
        if (helperDirectTag > 0) {
            double helperDistance = vision.getDistanceToTag(helperDirectTag);
            return new TagAssistResult(
                true,
                false,
                true,
                vision.getYawToTag(helperDirectTag),
                helperDistance,
                "HelperTagYaw(" + helperDirectTag + ")"
            );
        }

        int fallbackDirectTag = pickFirstVisibleTag(fallbackTagIds);
        if (fallbackDirectTag > 0) {
            double fallbackDistance = vision.getDistanceToTag(fallbackDirectTag);
            return new TagAssistResult(
                true,
                false,
                helperVisible,
                vision.getYawToTag(fallbackDirectTag),
                fallbackDistance,
                "AnyVisibleTagYaw(" + fallbackDirectTag + ")"
            );
        }

        return new TagAssistResult(false, false, helperVisible, 0.0, -1.0, "DeadReckoning");
    }

    private double[] getBlindReturnDrive(double phaseElapsedSec, AllianceSide alliance) {
        double leftOut;
        double rightOut;

        if (phaseElapsedSec < RobotConfig.Autonomous.RETURN_SEARCH_SPIN_SEC) {
            leftOut = -RobotConfig.Autonomous.SEARCH_SPIN_POWER;
            rightOut = RobotConfig.Autonomous.SEARCH_SPIN_POWER;
        } else if (phaseElapsedSec < RobotConfig.Autonomous.RETURN_SEARCH_SPIN_SEC
            + RobotConfig.Autonomous.RETURN_BLIND_STRAIGHT_SEC) {
            double base = RobotConfig.Autonomous.RETURN_DRIVE_POWER * 0.86;
            double correction = clamp(
                headingHoldPid.calculate(getYawDeg(), headingHoldSetpointDeg)
                    * RobotConfig.Autonomous.GYRO_CORRECTION_SIGN,
                -RobotConfig.Autonomous.GYRO_HEADING_MAX_CORRECTION * 0.6,
                RobotConfig.Autonomous.GYRO_HEADING_MAX_CORRECTION * 0.6
            );
            leftOut = clamp(base - correction, -1.0, 1.0);
            rightOut = clamp(base + correction, -1.0, 1.0);
        } else if (phaseElapsedSec < RobotConfig.Autonomous.RETURN_SEARCH_SPIN_SEC
            + RobotConfig.Autonomous.RETURN_BLIND_STRAIGHT_SEC
            + RobotConfig.Autonomous.RETURN_BLIND_ARC_SEC) {
            double base = RobotConfig.Autonomous.RETURN_DRIVE_POWER * 0.72;
            double bias = RobotConfig.Autonomous.RETURN_BLIND_ARC_BIAS;
            // Alliance'a göre dönüş yönü değişir; iki tarafta da aynı mantık çalışır.
            if (alliance == AllianceSide.BLUE) {
                leftOut = clamp(base - bias, -1.0, 1.0);
                rightOut = clamp(base + bias, -1.0, 1.0);
            } else {
                leftOut = clamp(base + bias, -1.0, 1.0);
                rightOut = clamp(base - bias, -1.0, 1.0);
            }
        } else {
            double base = RobotConfig.Autonomous.RETURN_DRIVE_POWER * 0.78;
            double correction = clamp(
                headingHoldPid.calculate(getYawDeg(), headingHoldSetpointDeg)
                    * RobotConfig.Autonomous.GYRO_CORRECTION_SIGN,
                -RobotConfig.Autonomous.GYRO_HEADING_MAX_CORRECTION * 0.5,
                RobotConfig.Autonomous.GYRO_HEADING_MAX_CORRECTION * 0.5
            );
            leftOut = clamp(base - correction, -1.0, 1.0);
            rightOut = clamp(base + correction, -1.0, 1.0);
        }

        return new double[] {leftOut, rightOut};
    }

    private void forceCompleteWait(String reason) {
        phase = 5;
        drive.tankDrive(0.0, 0.0);
        stopScoringActuators();
        intakeElevator.setClosePosition();
        SmartDashboard.putString("Auto/Status", "CRS-Complete Waiting");
        SmartDashboard.putString("Auto/CRS/CompleteReason", reason);
    }

    private void setIntakeRollers(double power) {
        intakeL.set(power);
        intakeR.set(-power);
    }

    private void stopScoringActuators() {
        shooter.stop();
        vcLever.set(0.0);
        setIntakeRollers(0.0);
    }

    private static final class TagAssistResult {
        final boolean hasSolution;
        final boolean targetVisible;
        final boolean helperVisible;
        final double yawErrorDeg;
        final double distanceM;
        final String source;

        TagAssistResult(
            boolean hasSolution,
            boolean targetVisible,
            boolean helperVisible,
            double yawErrorDeg,
            double distanceM,
            String source
        ) {
            this.hasSolution = hasSolution;
            this.targetVisible = targetVisible;
            this.helperVisible = helperVisible;
            this.yawErrorDeg = yawErrorDeg;
            this.distanceM = distanceM;
            this.source = source;
        }
    }

    /**
     * Sayıyı [min, max] aralığına sıkıştırır.
     * java.lang.Math.clamp Java 21'de geldi; eski JDK için bu versiyon kullanılır.
     */
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private void driveStraightWithGyro(double forwardPower) {
        double yaw = getYawDeg();
        double headingErrorDeg = shortestAngleDeltaDeg(headingHoldSetpointDeg, yaw);
        double correction = clamp(
            headingHoldPid.calculate(yaw, headingHoldSetpointDeg) * RobotConfig.Autonomous.GYRO_CORRECTION_SIGN,
            -RobotConfig.Autonomous.GYRO_HEADING_MAX_CORRECTION,
            RobotConfig.Autonomous.GYRO_HEADING_MAX_CORRECTION
        );

        SmartDashboard.putNumber("Auto/GyroHeadingTarget_deg", headingHoldSetpointDeg);
        SmartDashboard.putNumber("Auto/GyroHeadingError_deg", headingErrorDeg);
        SmartDashboard.putNumber("Auto/GyroHeadingCorrection", correction);
        SmartDashboard.putBoolean("Auto/GyroHeadingAtSetpoint", headingHoldPid.atSetpoint());

        double leftOut = clamp(forwardPower - correction, -1.0, 1.0);
        double rightOut = clamp(forwardPower + correction, -1.0, 1.0);
        drive.tankDrive(leftOut, rightOut);
    }

    private double getYawDeg() {
        double yaw = yawSupplierDeg.getAsDouble();
        if (Double.isFinite(yaw)) {
            lastYawDeg = wrapAngleDeg(yaw);
        }
        return lastYawDeg;
    }

    private static double wrapAngleDeg(double deg) {
        double wrapped = deg % 360.0;
        if (wrapped >= 180.0) {
            wrapped -= 360.0;
        } else if (wrapped < -180.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    private static double shortestAngleDeltaDeg(double targetDeg, double measurementDeg) {
        return wrapAngleDeg(targetDeg - measurementDeg);
    }
}
