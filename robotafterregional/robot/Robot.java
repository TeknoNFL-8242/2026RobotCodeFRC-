package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.ShooterSubsystem;
import frc.robot.RobotConfig.Elevator;


public class Robot extends TimedRobot {
    private static final int BLUE_TARGET_TAG_ID = 25;
    private static final int RED_TARGET_TAG_ID = 9;
    private static final int[] BLUE_TARGET_HELPER_TAGS = {26, 24, 21, 19, 20, 27, 18};
    private static final int[] RED_TARGET_HELPER_TAGS = {10, 8, 5, 11, 2, 4, 3};

    private enum AutoTeam {
        DEFAULT,
        RED,
        BLUE
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SUBSYSTEMS
    // ═══════════════════════════════════════════════════════════════════════
    private ShooterSubsystem shooterSubsystem;
    private VisionSubsystem  visionSubsystem;   // ← YENİ: PhotonVision subsystem
    private IntakeElevatorSubsystem intakeElevatorSubsystem;

    // ═══════════════════════════════════════════════════════════════════════
    // OTONOM YÖNETİCİSİ
    // ═══════════════════════════════════════════════════════════════════════
    private autonom autonomManager;              // ← YENİ: Otonom state machine

    // ═══════════════════════════════════════════════════════════════════════
    // DRIVETRAIN HARDWARE
    // ═══════════════════════════════════════════════════════════════════════
    private final WPI_VictorSPX left1  = new WPI_VictorSPX(RobotConfig.Ports.DRIVE_LEFT_1);
    private final WPI_VictorSPX left2  = new WPI_VictorSPX(RobotConfig.Ports.DRIVE_LEFT_2);
    private final WPI_VictorSPX right1 = new WPI_VictorSPX(RobotConfig.Ports.DRIVE_RIGHT_1);
    private final WPI_VictorSPX right2 = new WPI_VictorSPX(RobotConfig.Ports.DRIVE_RIGHT_2);
    private final WPI_VictorSPX vcLever = new WPI_VictorSPX(RobotConfig.Ports.VC_LEVER_MOTOR);

private final SparkFlex towerNew = new SparkFlex(RobotConfig.Ports.ELEVATOR_NEW_LEFT_CAN,MotorType.kBrushless);

    
    private final SparkMax elevatorIntake = new SparkMax(RobotConfig.Ports.ELEVATOR_INTAKE_CAN, MotorType.kBrushless);
    private final SparkMax intakeR        = new SparkMax(RobotConfig.Ports.INTAKE_RIGHT_CAN, MotorType.kBrushless);
    private final SparkMax intakeL        = new SparkMax(RobotConfig.Ports.INTAKE_LEFT_CAN, MotorType.kBrushless);
   //  private final PWMSparkMax spLever     = new PWMSparkMax(RobotConfig.Ports.PWM_SP_LEVER);
    private final PWMSparkMax spTowerR     = new PWMSparkMax(RobotConfig.Ports.PWM_SP_TOWER_ELEVATOR_RIGHT);  //asilma
    private final PWMSparkMax spTowerL    = new PWMSparkMax(RobotConfig.Ports.PWM_SP_TOWER_ELEVATOR_LEFT);  //asilma Left

    private final Encoder rightDriveEncoder = new Encoder(
        RobotConfig.Ports.DRIVE_RIGHT_ENCODER_DIO_A,
        RobotConfig.Ports.DRIVE_RIGHT_ENCODER_DIO_B
    );
    private final Encoder leftDriveEncoder = new Encoder(
        RobotConfig.Ports.DRIVE_LEFT_ENCODER_DIO_A,
        RobotConfig.Ports.DRIVE_LEFT_ENCODER_DIO_B
    );
    private final MedianFilter rightRateMedianFilter =
        new MedianFilter(RobotConfig.Encoder.DRIVE_RATE_MEDIAN_WINDOW);
    private final MedianFilter leftRateMedianFilter =
        new MedianFilter(RobotConfig.Encoder.DRIVE_RATE_MEDIAN_WINDOW);
    private final LinearFilter rightRateLowPassFilter =
        LinearFilter.singlePoleIIR(
            RobotConfig.Encoder.DRIVE_RATE_LP_TIME_CONSTANT_SEC,
            RobotConfig.Shooter.LOOP_DT_SEC
        );
    private final LinearFilter leftRateLowPassFilter =
        LinearFilter.singlePoleIIR(
            RobotConfig.Encoder.DRIVE_RATE_LP_TIME_CONSTANT_SEC,
            RobotConfig.Shooter.LOOP_DT_SEC
        );
    private int rightWakeCounter = 0;
    private int leftWakeCounter = 0;

    // ─── Drivetrain parametreleri ───
    private double leftPower  = 0;
    private double rightPower = 0;
    private boolean zLock     = false;
    private final double deadzone = RobotConfig.Drive.AXIS_DEADZONE;

    private DifferentialDrive m_drive;
    private final LogitechExtreme3DProController m_stick =
        new LogitechExtreme3DProController(RobotConfig.Ports.DRIVER_JOYSTICK);
    private final Timer m_timer = new Timer();

    // ─── Otonom Seçici ───
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final SendableChooser<AutoTeam> m_teamChooser = new SendableChooser<>();

    // Temel modlar
    private final String kAutoCollectReturnShoot = RobotConfig.Autonomous.MODE_COLLECT_RETURN_SHOOT;
    private final String kAutoIleri       = RobotConfig.Autonomous.MODE_FORWARD;
    private final String kAutoGeri        = RobotConfig.Autonomous.MODE_BACKWARD;
    private final String kAutoRoll        = RobotConfig.Autonomous.MODE_ROLL;

    // Vizyon modları ← YENİ
    private final String kAutoVisionAlign = RobotConfig.Autonomous.MODE_VISION_ALIGN;
    private final String kAutoVisionDrive = RobotConfig.Autonomous.MODE_VISION_DRIVE;
    private final String kAutoVisionScore = RobotConfig.Autonomous.MODE_VISION_SCORE;
    private final String kAutoTagAssistShoot = RobotConfig.Autonomous.MODE_TAG_ASSIST_SHOOT;
    private final String kAutoTowePreload = RobotConfig.Autonomous.MODE_TOWE_PRELOAD;

    private String m_autoSelected;
    private AutoTeam autoTeam = AutoTeam.DEFAULT;

    // ─── PathPlanner ───
    private final SubsystemBase driveAutoSubsystem = new SubsystemBase() {};
    private SendableChooser<Command> ppAutoChooser;
    private Command activePpAutoCommand;
    private boolean pathPlannerConfigured = false;
    private boolean pathPlannerRunning = false;
    private boolean intakeEmergencyTogglePrev = false;
    private boolean teleopShootablePrev = false;

    private final DifferentialDriveKinematics driveKinematics =
        new DifferentialDriveKinematics(RobotConfig.PathPlanner.TRACK_WIDTH_METERS);
    private final PIDController leftVelocityPid = new PIDController(
        RobotConfig.PathPlanner.DRIVE_VEL_KP,
        RobotConfig.PathPlanner.DRIVE_VEL_KI,
        RobotConfig.PathPlanner.DRIVE_VEL_KD
    );
    private final PIDController rightVelocityPid = new PIDController(
        RobotConfig.PathPlanner.DRIVE_VEL_KP,
        RobotConfig.PathPlanner.DRIVE_VEL_KI,
        RobotConfig.PathPlanner.DRIVE_VEL_KD
    );
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
        RobotConfig.PathPlanner.DRIVE_KS_VOLTS,
        RobotConfig.PathPlanner.DRIVE_KV_VOLTS_PER_MPS,
        RobotConfig.PathPlanner.DRIVE_KA_VOLTS_PER_MPS2
    );
    private DifferentialDrivePoseEstimator drivePoseEstimator;
    private double lastVisionTimestampSec = -1.0;

    // ─── Sürüş parametreleri ───
    private final double a     = RobotConfig.Drive.EXP_A;
    private final double k     = RobotConfig.Drive.CURVE_K;
    private final double LOG_K = Math.log(1.0 + k);

    // ─── IMU ───
    private Esp32Bridge imuBridge;
    private double imuYawDeg = 0.0;
    private boolean imuConnected = false;

    // ═══════════════════════════════════════════════════════════════════════
    // ROBOT INIT
    // ═══════════════════════════════════════════════════════════════════════
    private void configureVictors() {
        WPI_VictorSPX[] motors = { left1, left2, right1, right2 };

        for (WPI_VictorSPX m : motors) {
            m.enableVoltageCompensation(true);
            m.configVoltageCompSaturation(RobotConfig.BrownoutProtection.VICTOR_VOLTAGE_COMP_SATURATION_V);
            m.configOpenloopRamp(RobotConfig.BrownoutProtection.VICTOR_OPEN_LOOP_RAMP_SEC);
        }
    }

    private void configureIntakeElevatorSparkMax(SparkMax spark) {
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(
            RobotConfig.BrownoutProtection.SPARK_INTAKE_ELEVATOR_VOLTAGE_COMP_SATURATION_V
        );
        config.smartCurrentLimit(
            RobotConfig.BrownoutProtection.SPARK_INTAKE_ELEVATOR_SMART_CURRENT_LIMIT_A
        );
        config.secondaryCurrentLimit(
            RobotConfig.BrownoutProtection.SPARK_INTAKE_ELEVATOR_SECONDARY_CURRENT_LIMIT_A
        );
        config.openLoopRampRate(
            RobotConfig.BrownoutProtection.SPARK_INTAKE_ELEVATOR_OPEN_LOOP_RAMP_SEC
        );

        spark.configure(
            config,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    @Override
    public void robotInit() {

        // ─── Subsystem başlatma ───
        shooterSubsystem = new ShooterSubsystem();
        System.out.println("✓ ShooterSubsystem başlatıldı");

        // YENİ: Vizyon subsystem'i başlat
        visionSubsystem = new VisionSubsystem();

        configureIntakeElevatorSparkMax(elevatorIntake);
        configureIntakeElevatorSparkMax(intakeR);
        configureIntakeElevatorSparkMax(intakeL);
        intakeElevatorSubsystem = new IntakeElevatorSubsystem(elevatorIntake);
        intakeElevatorSubsystem.setZeroHere();
        intakeElevatorSubsystem.setClosePosition();
        SmartDashboard.putString("IntakeElevator/StartupZero", "Current position accepted as zero");

        // ─── Drivetrain ───
        left2.follow(left1);
        configureVictors();
        right2.follow(right1);
        m_drive = new DifferentialDrive(left1, right1);
        m_drive.setExpiration(RobotConfig.Drive.MOTOR_SAFETY_EXPIRATION_SEC);
        rightDriveEncoder.reset();
        leftDriveEncoder.reset();
        rightDriveEncoder.setDistancePerPulse(1.0);
        leftDriveEncoder.setDistancePerPulse(1.0);
        rightDriveEncoder.setSamplesToAverage(RobotConfig.Encoder.DRIVE_SAMPLES_TO_AVERAGE);
        leftDriveEncoder.setSamplesToAverage(RobotConfig.Encoder.DRIVE_SAMPLES_TO_AVERAGE);
        rightDriveEncoder.setMaxPeriod(RobotConfig.Encoder.DRIVE_STOP_MAX_PERIOD_SEC);
        leftDriveEncoder.setMaxPeriod(RobotConfig.Encoder.DRIVE_STOP_MAX_PERIOD_SEC);
        drivePoseEstimator = new DifferentialDrivePoseEstimator(
            driveKinematics,
            getGyroRotation2d(),
            getLeftDistanceMeters(),
            getRightDistanceMeters(),
            new Pose2d()
        );
        leftVelocityPid.reset();
        rightVelocityPid.reset();

        // YENİ: Otonom yöneticisini oluştur (drive/vision + shooter/intake + team)
        autonomManager = new autonom(
            m_drive,
            visionSubsystem,
            this::getImuYawDeg,
            shooterSubsystem,
            intakeElevatorSubsystem,
            vcLever,
            intakeL,
            intakeR,
            this::isBlueTeam,
            this::isRedTeam
        );

        // ─── Otonom seçici ───
        m_chooser.setDefaultOption("collectReturnShoot (default)", kAutoCollectReturnShoot);
        m_chooser.addOption("doNothing",  RobotConfig.Autonomous.MODE_DO_NOTHING);
        m_chooser.addOption("ileriGit",    kAutoIleri);
        m_chooser.addOption("geriGit",     kAutoGeri);
        m_chooser.addOption("roll",        kAutoRoll);
        m_chooser.addOption("visionAlign", kAutoVisionAlign);  // ← YENİ
        m_chooser.addOption("visionDrive", kAutoVisionDrive);  // ← YENİ
        m_chooser.addOption("visionScore", kAutoVisionScore);  // ← YENİ
        m_chooser.addOption("tagAssistShoot (IDs 9/10/25/26)", kAutoTagAssistShoot);
        m_chooser.addOption("TowerPreload (IDs 10/25)", kAutoTowePreload);

        m_teamChooser.setDefaultOption("Default", AutoTeam.DEFAULT);
        m_teamChooser.addOption("Blue", AutoTeam.BLUE);
        m_teamChooser.addOption("Red", AutoTeam.RED);

        SmartDashboard.putData("Auto/Selector", m_chooser);
        SmartDashboard.putData("Auto/TeamSelector", m_teamChooser);
        SmartDashboard.putStringArray("Auto/List", RobotConfig.Autonomous.MODE_LIST);
        SmartDashboard.putBoolean("Auto/ForceDoNothing", RobotConfig.Autonomous.FORCE_DO_NOTHING);
        SmartDashboard.putString("Auto/Team", autoTeam.name());
        SmartDashboard.putNumberArray(
            "Auto/TagAssistShoot/TargetIds",
            new double[] {9, 10, 25, 26}
        );
        SmartDashboard.putNumber(
            "Auto/TagAssistShoot/MinDistance_m",
            RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MIN_DISTANCE_M
        );
        SmartDashboard.putNumber(
            "Auto/TagAssistShoot/MaxDistance_m",
            RobotConfig.Autonomous.TAG_ASSIST_SHOOT_MAX_DISTANCE_M
        );
        SmartDashboard.putNumberArray(
            "Auto/TowePreload/TargetIds",
            new double[] {10, 25}
        );
        SmartDashboard.putNumber("Auto/TowePreload/YMin_m", RobotConfig.Autonomous.TOWE_PRELOAD_Y_MIN_M);
        SmartDashboard.putNumber("Auto/TowePreload/YMax_m", RobotConfig.Autonomous.TOWE_PRELOAD_Y_MAX_M);
        SmartDashboard.putNumber(
            "Auto/TowePreload/ApproachStop_m",
            RobotConfig.Autonomous.TOWE_PRELOAD_APPROACH_STOP_M
        );
        
        // Robot status initialization
        SmartDashboard.putString("Robot/Mode", "INIT");
        SmartDashboard.putString("Robot/Status", "Ready");

        // ─── Driver camera (robot view) ───
        CameraServer.startAutomaticCapture();

        // ─── IMU ───
        imuBridge = new Esp32Bridge();

        // ─── PathPlanner ───
        configurePathPlanner();

        // --- Elevator ---
        towerNew.configure(Elevator.CONFIG, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);    

        System.out.println("✓ Robot başlatıldı");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // ROBOT PERIODIC — Her modda 50 Hz çalışır
    // ═══════════════════════════════════════════════════════════════════════
    @Override
    public void robotPeriodic() {
        AutoTeam selectedTeam = m_teamChooser.getSelected();
        autoTeam = selectedTeam != null ? selectedTeam : AutoTeam.DEFAULT;
        SmartDashboard.putString("Auto/Team", autoTeam.name());

        // YENİ: Vizyon her zaman güncellenir (teleop + auto + test)
        visionSubsystem.update();
        shooterSubsystem.periodic();
        if (intakeElevatorSubsystem != null) {
            intakeElevatorSubsystem.periodic();
        }

        // IMU okumayı tek yerde yap; auto/teleop/test aynı veriyi kullansın.
        float[] e = imuBridge.readEuler();
        if (e != null) {
            imuConnected = true;
            imuYawDeg = e[2];
            SmartDashboard.putNumber("IMU/Roll",  e[0]);
            SmartDashboard.putNumber("IMU/Pitch", e[1]);
            SmartDashboard.putNumber("IMU/Yaw",   imuYawDeg);
        } else {
            imuConnected = false;
        }
        SmartDashboard.putBoolean("IMU/Connected", imuConnected);
        SmartDashboard.putNumber("IMU/YawForAuto_deg", imuYawDeg);

        // Pose tahmini: encoder + gyro ile sürekli güncelle, istenirse vision ile düzelt.
        updateDrivePoseEstimator();

        // Command-based komutlar (PathPlanner auto dahil) bu döngüde işlenir.
        CommandScheduler.getInstance().run();

        // Drivetrain encoder telemetry (Sag A/B: 2/3, Sol A/B: 4/5)
        SmartDashboard.putNumber("Drivetrain/EncoderRightCount", rightDriveEncoder.get());
        SmartDashboard.putNumber("Drivetrain/EncoderLeftCount", leftDriveEncoder.get());
        double rightRateRaw = rightDriveEncoder.getRate();
        double leftRateRaw = leftDriveEncoder.getRate();
        double rightRateMedian = rightRateMedianFilter.calculate(rightRateRaw);
        double leftRateMedian = leftRateMedianFilter.calculate(leftRateRaw);
        double rightRateFiltered = rightRateLowPassFilter.calculate(rightRateMedian);
        double leftRateFiltered = leftRateLowPassFilter.calculate(leftRateMedian);
        boolean rightStopped = rightDriveEncoder.getStopped();
        boolean leftStopped = leftDriveEncoder.getStopped();

        // Hareketi "uyanık" saymak için ardışık örnek ister; boşta jitter'ı bastırır.
        if (Math.abs(rightRateRaw) > RobotConfig.Encoder.DRIVE_RATE_WAKE_THRESHOLD) {
            rightWakeCounter++;
        } else {
            rightWakeCounter = 0;
        }
        if (Math.abs(leftRateRaw) > RobotConfig.Encoder.DRIVE_RATE_WAKE_THRESHOLD) {
            leftWakeCounter++;
        } else {
            leftWakeCounter = 0;
        }
        boolean rightMoving = rightWakeCounter >= RobotConfig.Encoder.DRIVE_RATE_WAKE_SAMPLES;
        boolean leftMoving = leftWakeCounter >= RobotConfig.Encoder.DRIVE_RATE_WAKE_SAMPLES;

        if (rightStopped || !rightMoving
            || Math.abs(rightRateFiltered) < RobotConfig.Encoder.DRIVE_RATE_ZERO_DEADBAND) {
            rightRateFiltered = 0.0;
        }
        if (leftStopped || !leftMoving
            || Math.abs(leftRateFiltered) < RobotConfig.Encoder.DRIVE_RATE_ZERO_DEADBAND) {
            leftRateFiltered = 0.0;
        }

        SmartDashboard.putNumber("Drivetrain/EncoderRightRateRaw", rightRateRaw);
        SmartDashboard.putNumber("Drivetrain/EncoderLeftRateRaw", leftRateRaw);
        SmartDashboard.putNumber("Drivetrain/EncoderRightRateFiltered", rightRateFiltered);
        SmartDashboard.putNumber("Drivetrain/EncoderLeftRateFiltered", leftRateFiltered);
        SmartDashboard.putBoolean("Drivetrain/EncoderRightStopped", rightStopped);
        SmartDashboard.putBoolean("Drivetrain/EncoderLeftStopped", leftStopped);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // OTONOM
    // ═══════════════════════════════════════════════════════════════════════
    @Override
    public void autonomousInit() {
        m_timer.restart();
        cancelActivePathPlannerAuto();
        leftVelocityPid.reset();
        rightVelocityPid.reset();
        
        // Update robot status
        SmartDashboard.putString("Robot/Mode", "AUTO");
        SmartDashboard.putString("Robot/Status", "Autonomous Running");

        if (!RobotConfig.Autonomous.ENABLED) {
            pathPlannerRunning = false;
            m_autoSelected = "DISABLED";
            m_drive.tankDrive(0, 0);
            shooterSubsystem.stop();
            vcLever.set(0.0);
            SmartDashboard.putString("Auto/Selected", "DISABLED");
            SmartDashboard.putString("Auto/Status", "Disabled from RobotConfig");
            System.out.println("⚠ Autonomous disabled from RobotConfig.Autonomous.ENABLED");
            return;
        }

        m_autoSelected = m_chooser.getSelected();
        if (m_autoSelected == null) {
            m_autoSelected = RobotConfig.Autonomous.DEFAULT_MODE;
        }
        if (RobotConfig.Autonomous.FORCE_DO_NOTHING) {
            m_autoSelected = RobotConfig.Autonomous.MODE_DO_NOTHING;
            SmartDashboard.putString("Auto/Status", "FORCE_DO_NOTHING active");
        }

        // Öncelik: PathPlanner auto (toggle açık ve konfigürasyon başarılıysa)
        if (RobotConfig.PathPlanner.ENABLED
            && RobotConfig.PathPlanner.RUN_IN_AUTONOMOUS
            && !RobotConfig.Autonomous.FORCE_DO_NOTHING
            && pathPlannerConfigured
            && ppAutoChooser != null) {
            activePpAutoCommand = ppAutoChooser.getSelected();
            if (activePpAutoCommand != null) {
                pathPlannerRunning = true;
                CommandScheduler.getInstance().schedule(activePpAutoCommand);
                SmartDashboard.putString("Auto/Selected", "PathPlanner");
                SmartDashboard.putString("Auto/Status", "PathPlanner Auto Running");
                System.out.println("🤖 Otonom modu: PathPlanner auto schedule edildi");
                return;
            }
            SmartDashboard.putString("Auto/Status", "PathPlanner seçimi boş, fallback custom auto");
        }

        pathPlannerRunning = false;

        // Fallback: mevcut custom auto state machine
        SmartDashboard.putString("Auto/Selected", m_autoSelected);
        System.out.println("🤖 Otonom modu: " + m_autoSelected);
        autonomManager.init(m_autoSelected);
    }

    @Override
    public void autonomousPeriodic() {
        if (!RobotConfig.Autonomous.ENABLED) {
            m_drive.tankDrive(0, 0);
            return;
        }
        if (RobotConfig.Autonomous.FORCE_DO_NOTHING) {
            m_drive.tankDrive(0, 0);
            shooterSubsystem.stop();
            vcLever.set(0.0);
            intakeL.set(0.0);
            intakeR.set(0.0);
            SmartDashboard.putString("Auto/Status", "FORCE_DO_NOTHING active");
            return;
        }

        if (pathPlannerRunning) {
            if (activePpAutoCommand == null
                || !CommandScheduler.getInstance().isScheduled(activePpAutoCommand)) {
                pathPlannerRunning = false;
                SmartDashboard.putString("Auto/Status", "PathPlanner Auto Complete");
                m_drive.tankDrive(0, 0);
            }
            return;
        }

        // PathPlanner seçildiyse ve tamamlandıysa custom auto'ya düşme.
        if (activePpAutoCommand != null) {
            m_drive.tankDrive(0, 0);
            return;
        }

        if (!pathPlannerRunning) {
            // YENİ: Tüm custom otonom mantığı autonom.java'dadır
            // robotPeriodic() içinde vision.update() zaten çalışıyor
            autonomManager.step();
        }
    }

    @Override
    public void autonomousExit() {
        m_timer.stop();
        cancelActivePathPlannerAuto();
        autonomManager.stop();
        shooterSubsystem.stop();
        vcLever.set(0.0);
        intakeL.set(0.0);
        intakeR.set(0.0);
        intakeElevatorSubsystem.setClosePosition();
        SmartDashboard.putString("Robot/Status", "Autonomous Complete");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TELEOP
    // ═══════════════════════════════════════════════════════════════════════
    @Override
    public void teleopInit() {
        cancelActivePathPlannerAuto();
        intakeEmergencyTogglePrev = false;
        teleopShootablePrev = false;
    }

    @Override
    public void teleopPeriodic() {
        // Update robot mode
        SmartDashboard.putString("Robot/Mode", "TELEOP");
        SmartDashboard.putString("Robot/Status", "Driver Control");
        
        double y_ax = -m_stick.getJoystickY();  // İleri/Geri
        double x_ax = -m_stick.getJoystickX();  // Sağ/Sol
        double z_ax = -m_stick.getJoystickZ();  // Yerinde dönüş

        boolean xCentered = Math.abs(x_ax) < deadzone;
        boolean yCentered = Math.abs(y_ax) < deadzone;
        boolean zCentered = Math.abs(z_ax) < deadzone;
        boolean button1   = m_stick.getButtonOne();
        boolean button2   = m_stick.getButtonTwo();
        boolean button3   = m_stick.getButtonThree();
        boolean button6 = m_stick.getButtonSix();
        boolean button5 = m_stick.getButtonFive();
        boolean apriltagLockRequested = button2 && (isBlueTeam() || isRedTeam());
boolean button4 = m_stick.getButtonFour();
boolean button12 = m_stick.getButtonTwelve();
        boolean padUp = m_stick.getDPadUp(); //ball intake open
        boolean padDown = m_stick.getDPadDown();//ball intake close
        boolean padLeft = m_stick.getDPadLeft();
        boolean padRight = m_stick.getDPadRight();
        boolean intakeSetZeroBtn = m_stick.getButton(RobotConfig.IntakeElevator.BUTTON_SET_ZERO);
        boolean intakeEmergencyBtn = m_stick.getButton(RobotConfig.IntakeElevator.BUTTON_EMERGENCY_DISABLE);
       
        boolean hang = false;


        if (!xCentered || !yCentered)              zLock = true;
        if (xCentered && yCentered && zCentered)   zLock = false;

        if (apriltagLockRequested) {
            int targetTagId = isBlueTeam() ? BLUE_TARGET_TAG_ID : RED_TARGET_TAG_ID;
            int[] helperTagIds = isBlueTeam() ? BLUE_TARGET_HELPER_TAGS : RED_TARGET_HELPER_TAGS;
            double tagDistanceM = visionSubsystem.getDistanceToTag(targetTagId);
            boolean targetVisible = tagDistanceM >= 0.0;
            boolean helperVisible = visionSubsystem.isAnyTagVisible(helperTagIds);
            double alignErrorDeg;
            String alignSource;

            if (targetVisible) {
                alignErrorDeg = visionSubsystem.getYawToTag(targetTagId);
                alignSource = "DirectTagYaw";
            } else {
                Pose2d estimatedPose = visionSubsystem.getBestEstimatedPose();
                Optional<Pose2d> tagPoseOpt = visionSubsystem.getFieldTagPose2d(targetTagId);
                if (estimatedPose != null && tagPoseOpt.isPresent() && helperVisible) {
                    Pose2d tagPose = tagPoseOpt.get();
                    double dx = tagPose.getX() - estimatedPose.getX();
                    double dy = tagPose.getY() - estimatedPose.getY();

                    double desiredHeadingDeg = Math.toDegrees(Math.atan2(dy, dx));
                    double currentHeadingDeg = estimatedPose.getRotation().getDegrees();
                    alignErrorDeg = wrapAngleDeg(desiredHeadingDeg - currentHeadingDeg);
                    tagDistanceM = Math.hypot(dx, dy);
                    alignSource = "HelperPoseToTag";
                } else {
                    // If no target and no estimated pose, spin slowly to search for tags.
                    leftPower = -0.18;
                    rightPower = 0.18;
                    teleopShootablePrev = false;
                    SmartDashboard.putNumber("Teleop/TagAssist/TargetTagId", targetTagId);
                    SmartDashboard.putBoolean("Teleop/TagAssist/Visible", false);
                    SmartDashboard.putBoolean("Teleop/TagAssist/HelperVisible", helperVisible);
                    SmartDashboard.putString("Teleop/TagAssist/Source", "SearchSpin");
                    SmartDashboard.putBoolean("Teleop/TagAssist/Shootable", false);
                    SmartDashboard.putNumber("Teleop/TagAssist/Distance_m", -1.0);
                    SmartDashboard.putNumber("Teleop/TagAssist/YawError_deg", 0.0);
                    alignErrorDeg = 0.0;
                    alignSource = "SearchSpin";
                }
            }

            if (!"SearchSpin".equals(alignSource)) {
                double correction = clamp(
                    alignErrorDeg * RobotConfig.Autonomous.KP_ALIGN,
                    -RobotConfig.Autonomous.MAX_TURN_POWER,
                    RobotConfig.Autonomous.MAX_TURN_POWER
                );

                // Driver keeps forward/backward control on Y while robot auto-aligns to the target tag.
                leftPower = clamp(y_ax - correction, -1.0, 1.0);
                rightPower = clamp(y_ax + correction, -1.0, 1.0);
            }

            boolean shootable = tagDistanceM >= 2.55 && tagDistanceM <= 3.15;
            if (shootable && !teleopShootablePrev) {
                System.out.println("SHOOTABLE");
            }
            teleopShootablePrev = shootable;

            SmartDashboard.putNumber("Teleop/TagAssist/TargetTagId", targetTagId);
            SmartDashboard.putBoolean("Teleop/TagAssist/Visible", targetVisible);
            SmartDashboard.putBoolean("Teleop/TagAssist/HelperVisible", helperVisible);
            SmartDashboard.putString("Teleop/TagAssist/Source", alignSource);
            SmartDashboard.putNumber("Teleop/TagAssist/YawError_deg", alignErrorDeg);
            SmartDashboard.putNumber("Teleop/TagAssist/Distance_m", tagDistanceM);
            SmartDashboard.putBoolean("Teleop/TagAssist/Shootable", shootable);
        } else if ((Math.abs(z_ax) > RobotConfig.Drive.Z_TURN_ENABLE_THRESHOLD
            && Math.abs(x_ax) < RobotConfig.Drive.Z_TURN_ENABLE_THRESHOLD
            && Math.abs(y_ax) < RobotConfig.Drive.Z_TURN_ENABLE_THRESHOLD) && !zLock) {
            leftPower  =  z_ax;
            rightPower = -z_ax;
        } else if (Math.abs(x_ax) > 0 && Math.abs(y_ax) > 0) {
            double xAbs = Math.abs(x_ax);
            double yAbs = Math.abs(y_ax);
            double f    = Math.log(1.0 + k * xAbs) / LOG_K;
            double g    = Math.pow(1.0 - yAbs, a);

            if (x_ax > 0 && y_ax > 0) {
                leftPower  = yAbs + yAbs * f * g;
                rightPower = yAbs * (1.0 - f);
            } else if (x_ax < 0 && y_ax > 0) {
                leftPower  = yAbs * (1.0 - f);
                rightPower = yAbs + yAbs * f * g;
            } else if (x_ax > 0 && y_ax < 0) {
                leftPower  = -yAbs - yAbs * f * g;
                rightPower = -yAbs * (1.0 - f);
            } else if (x_ax < 0 && y_ax < 0) {
                leftPower  = -yAbs * (1.0 - f);
                rightPower = -yAbs - yAbs * f * g;
            }
        } else if (Math.abs(y_ax) > 0) {
            leftPower  = y_ax;
            rightPower = y_ax;
        } else {
            leftPower  = 0;
            rightPower = 0;
            teleopShootablePrev = false;
        }



        if (!apriltagLockRequested) {
            teleopShootablePrev = false;
        }



        m_drive.tankDrive(leftPower, rightPower);

        // ─── Intake Elevator (encoder position control) ───
        if (intakeEmergencyBtn && !intakeEmergencyTogglePrev) {
            intakeElevatorSubsystem.setEmergencyStop(!intakeElevatorSubsystem.isEmergencyStopped());
        }
        intakeEmergencyTogglePrev = intakeEmergencyBtn;

        if (intakeSetZeroBtn) {
            intakeElevatorSubsystem.setZeroHere();
            intakeElevatorSubsystem.setClosePosition();       // 0 turn
        } else if (padUp) {
            intakeElevatorSubsystem.setOpenPosition();        // +3 turn
        } else if (padDown) {
            intakeElevatorSubsystem.setCloseDownPosition();   // -3 turn
        }

        // ─── Tower PWM Spark motors ───
        if (padLeft) {
           towerNew.set(0.4);
        } else if (padRight) {
          towerNew.set(-0.4);
            elevatorIntake.set(0);

         
            hang=true;
        } else {
           towerNew.set(0);

        }




        // ─── Shooter + intake ───
          double shooterSlider = normalizeSliderToUnit(m_stick.getSlider());
               // Only when Button1 + Button6 together, invert slider effect.
      

        // Flywheel/shooter works only while Button1 is held.
        if (button1 && shooterSlider > RobotConfig.Operator.SHOOTER_SLIDER_MIN) {
      double targetRPM = RobotConfig.Operator.SHOOTER_TELEOP_BASE_RPM
                + (shooterSlider * RobotConfig.Operator.SHOOTER_TELEOP_RANGE_RPM);
            shooterSubsystem.setTargetRPM(targetRPM);
        } else {
            shooterSubsystem.stop();
        }

        // Default intake direction from slider.
        double intakeLeftCmd = shooterSlider * RobotConfig.Operator.INTAKE_SCALE;
        double intakeRightCmd = -shooterSlider * RobotConfig.Operator.INTAKE_SCALE;
    
        // Button1 + Button6 -> reverse intake and keep it active.
        if (button1 && button6) {
            //intakeLeftCmd = -shooterSlider * RobotConfig.Operator.INTAKE_SCALE;
            intakeRightCmd = shooterSlider * RobotConfig.Operator.INTAKE_SCALE;
            shooterSubsystem.setTargetRPM(0);
        }
        if (button6){
            intakeLeftCmd = -intakeLeftCmd;
            //intakeRightCmd = -intakeRightCmd;
           
        }
        intakeL.set(intakeLeftCmd);
        intakeR.set(intakeRightCmd);

        if (button1) {
            vcLever.setVoltage(-12 * shooterSlider);
            if (!button6) {
                intakeL.set(0.0);
                intakeR.set(0.0);
            }
        }else {
            vcLever.set(0.0);
        }



      


        // Drivetrain telemetry
        SmartDashboard.putNumber("Drivetrain/LeftPower", leftPower);
        SmartDashboard.putNumber("Drivetrain/RightPower", rightPower);
        SmartDashboard.putBoolean("Drivetrain/ZLock", zLock);
    }

    


    @Override
    public void teleopExit() {
        shooterSubsystem.stop();
        SmartDashboard.putString("Robot/Status", "Teleop Ended");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // TEST
    // ═══════════════════════════════════════════════════════════════════════
    @Override
    public void testInit() {
        System.out.println("==== TEST INIT ====");
        SmartDashboard.putString("Robot/Mode", "TEST");
        SmartDashboard.putString("Robot/Status", "Testing");
        intakeEmergencyTogglePrev = false;
    }

    @Override
    public void testPeriodic() {
        SmartDashboard.putNumber("Test/Timestamp", System.currentTimeMillis());

        boolean padUp = m_stick.getDPadUp();
        boolean padDown = m_stick.getDPadDown();
        boolean intakeSetZeroBtn = m_stick.getButton(RobotConfig.IntakeElevator.BUTTON_SET_ZERO);
        boolean intakeEmergencyBtn = m_stick.getButton(RobotConfig.IntakeElevator.BUTTON_EMERGENCY_DISABLE);

      //  double shooterSlider = m_stick.getSlider();
      //  if (shooterSlider > RobotConfig.Operator.SHOOTER_SLIDER_MIN) {
     //       double testRPM = shooterSlider * RobotConfig.Operator.SHOOTER_TEST_RANGE_RPM;
        //    shooterSubsystem.setTargetRPM(testRPM);
    //    } else {
     //       shooterSubsystem.stop();
       // }

        // Test mode specific telemetry
      //  SmartDashboard.putNumber("Test/ShooterSlider", shooterSlider);
        //vcLever.setVoltage(6.0);
        //vcLever.set(shooterSlider);
        //intakeL.set(-shooterSlider * RobotConfig.Operator.INTAKE_SCALE);
       // intakeR.set( shooterSlider * RobotConfig.Operator.INTAKE_SCALE);

        if (intakeEmergencyBtn && !intakeEmergencyTogglePrev) {
            intakeElevatorSubsystem.setEmergencyStop(!intakeElevatorSubsystem.isEmergencyStopped());
        }
        intakeEmergencyTogglePrev = intakeEmergencyBtn;

        if (intakeSetZeroBtn) {
            intakeElevatorSubsystem.setZeroHere();
            intakeElevatorSubsystem.setClosePosition();
        } else if (padUp) {
            intakeElevatorSubsystem.setOpenPosition();
        } else if (padDown) {
            intakeElevatorSubsystem.setCloseDownPosition();
        }

        // ─── Vision test outputs ───
        SmartDashboard.putBoolean("Test/Vision/FrontHasTarget", visionSubsystem.frontHasTarget());
        SmartDashboard.putBoolean("Test/Vision/BackHasTarget",  visionSubsystem.backHasTarget());
        SmartDashboard.putNumber ("Test/Vision/FrontDist_m",    visionSubsystem.getFrontDistance());
        SmartDashboard.putNumber ("Test/Vision/FrontYaw_deg",   visionSubsystem.getFrontYaw());
        SmartDashboard.putNumber ("Test/Vision/BackDist_m",     visionSubsystem.getBackDistance());
        SmartDashboard.putNumber ("Test/Vision/BackYaw_deg",    visionSubsystem.getBackYaw());
        
        // Test mechanism status
        SmartDashboard.putNumber(
            "Test/ElevatorIntake",
            padUp ? 1.0 : (padDown ? -1.0 : 0.0)
        );
        SmartDashboard.putNumber("Test/IntakeL", intakeL.get());
        SmartDashboard.putNumber("Test/IntakeR", intakeR.get());
        SmartDashboard.putNumber("Test/vcLever", vcLever.get());
    }

    private void configurePathPlanner() {
        if (!RobotConfig.PathPlanner.ENABLED) {
            SmartDashboard.putBoolean("PathPlanner/Enabled", false);
            SmartDashboard.putBoolean("PathPlanner/Configured", false);
            SmartDashboard.putString("PathPlanner/Status", "Disabled from RobotConfig");
            return;
        }

        try {
            com.pathplanner.lib.config.RobotConfig ppRobotConfig =
                com.pathplanner.lib.config.RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                this::getEstimatedPose,
                this::resetEstimatedPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                new PPLTVController(0.02),
                ppRobotConfig,
                this::shouldFlipPathForAlliance,
                driveAutoSubsystem
            );

            ppAutoChooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto/PathPlannerChooser", ppAutoChooser);
            pathPlannerConfigured = true;

            SmartDashboard.putBoolean("PathPlanner/Enabled", true);
            SmartDashboard.putBoolean("PathPlanner/Configured", true);
            SmartDashboard.putString("PathPlanner/Status", "Configured");
            System.out.println("✓ PathPlanner configured");
        } catch (Exception ex) {
            pathPlannerConfigured = false;
            SmartDashboard.putBoolean("PathPlanner/Enabled", true);
            SmartDashboard.putBoolean("PathPlanner/Configured", false);
            SmartDashboard.putString("PathPlanner/Status", "Config error: " + ex.getMessage());
            System.err.println("❌ PathPlanner configure failed: " + ex.getMessage());
        }
    }

    private void updateDrivePoseEstimator() {
        if (drivePoseEstimator == null) {
            return;
        }

        drivePoseEstimator.update(
            getGyroRotation2d(),
            getLeftDistanceMeters(),
            getRightDistanceMeters()
        );

        if (RobotConfig.PathPlanner.ENABLE_VISION_FUSION) {
            Optional<EstimatedRobotPose> bestVisionPose = getBestVisionEstimatedPose();
            if (bestVisionPose.isPresent()) {
                EstimatedRobotPose vision = bestVisionPose.get();
                if (vision.timestampSeconds > lastVisionTimestampSec + 1e-5) {
                    drivePoseEstimator.addVisionMeasurement(
                        vision.estimatedPose.toPose2d(),
                        vision.timestampSeconds
                    );
                    lastVisionTimestampSec = vision.timestampSeconds;
                }
            }
        }

        Pose2d pose = drivePoseEstimator.getEstimatedPosition();
        SmartDashboard.putNumber("PathPlanner/PoseX_m", pose.getX());
        SmartDashboard.putNumber("PathPlanner/PoseY_m", pose.getY());
        SmartDashboard.putNumber("PathPlanner/PoseHeading_deg", pose.getRotation().getDegrees());
        SmartDashboard.putNumber("PathPlanner/LeftDist_m", getLeftDistanceMeters());
        SmartDashboard.putNumber("PathPlanner/RightDist_m", getRightDistanceMeters());
        SmartDashboard.putNumber("PathPlanner/LeftVel_mps", getLeftVelocityMetersPerSec());
        SmartDashboard.putNumber("PathPlanner/RightVel_mps", getRightVelocityMetersPerSec());
    }

    private Optional<EstimatedRobotPose> getBestVisionEstimatedPose() {
        Optional<EstimatedRobotPose> front = visionSubsystem.getFrontEstimatedPose();
        Optional<EstimatedRobotPose> back = visionSubsystem.getBackEstimatedPose();

        if (front.isPresent() && back.isPresent()) {
            double frontAmb = visionSubsystem.getFrontAmbiguity();
            double backAmb = visionSubsystem.getBackAmbiguity();
            if (frontAmb == backAmb) {
                return front.get().timestampSeconds >= back.get().timestampSeconds ? front : back;
            }
            return frontAmb < backAmb ? front : back;
        }
        if (front.isPresent()) {
            return front;
        }
        return back;
    }

    private Rotation2d getGyroRotation2d() {
        double yaw = getImuYawDeg();
        if (RobotConfig.PathPlanner.GYRO_INVERTED) {
            yaw = -yaw;
        }
        return Rotation2d.fromDegrees(yaw);
    }

    private double getLeftDistanceMeters() {
        double sign = RobotConfig.PathPlanner.LEFT_ENCODER_INVERTED ? -1.0 : 1.0;
        return leftDriveEncoder.get() * RobotConfig.PathPlanner.DRIVE_ENCODER_DISTANCE_PER_PULSE_M * sign;
    }

    private double getRightDistanceMeters() {
        double sign = RobotConfig.PathPlanner.RIGHT_ENCODER_INVERTED ? -1.0 : 1.0;
        return rightDriveEncoder.get() * RobotConfig.PathPlanner.DRIVE_ENCODER_DISTANCE_PER_PULSE_M * sign;
    }

    private double getLeftVelocityMetersPerSec() {
        double sign = RobotConfig.PathPlanner.LEFT_ENCODER_INVERTED ? -1.0 : 1.0;
        return leftDriveEncoder.getRate() * RobotConfig.PathPlanner.DRIVE_ENCODER_DISTANCE_PER_PULSE_M * sign;
    }

    private double getRightVelocityMetersPerSec() {
        double sign = RobotConfig.PathPlanner.RIGHT_ENCODER_INVERTED ? -1.0 : 1.0;
        return rightDriveEncoder.getRate() * RobotConfig.PathPlanner.DRIVE_ENCODER_DISTANCE_PER_PULSE_M * sign;
    }

    private Pose2d getEstimatedPose() {
        return drivePoseEstimator.getEstimatedPosition();
    }

    private void resetEstimatedPose(Pose2d pose) {
        drivePoseEstimator.resetPosition(
            getGyroRotation2d(),
            getLeftDistanceMeters(),
            getRightDistanceMeters(),
            pose
        );
        lastVisionTimestampSec = -1.0;
    }

    private ChassisSpeeds getRobotRelativeSpeeds() {
        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(
            getLeftVelocityMetersPerSec(),
            getRightVelocityMetersPerSec()
        );
        return driveKinematics.toChassisSpeeds(wheelSpeeds);
    }

    private void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        DifferentialDriveWheelSpeeds target = driveKinematics.toWheelSpeeds(speeds);
        double leftTargetMps = target.leftMetersPerSecond;
        double rightTargetMps = target.rightMetersPerSecond;

        double leftMeasuredMps = getLeftVelocityMetersPerSec();
        double rightMeasuredMps = getRightVelocityMetersPerSec();

        double leftVolts = driveFeedforward.calculate(leftTargetMps)
            + leftVelocityPid.calculate(leftMeasuredMps, leftTargetMps);
        double rightVolts = driveFeedforward.calculate(rightTargetMps)
            + rightVelocityPid.calculate(rightMeasuredMps, rightTargetMps);

        double leftOut = clamp(leftVolts / 12.0, -1.0, 1.0);
        double rightOut = clamp(rightVolts / 12.0, -1.0, 1.0);

        m_drive.tankDrive(leftOut, rightOut, false);

        SmartDashboard.putNumber("PathPlanner/TargetLeftVel_mps", leftTargetMps);
        SmartDashboard.putNumber("PathPlanner/TargetRightVel_mps", rightTargetMps);
        SmartDashboard.putNumber("PathPlanner/DriveLeftOut", leftOut);
        SmartDashboard.putNumber("PathPlanner/DriveRightOut", rightOut);
    }

    private boolean shouldFlipPathForAlliance() {
        if (!RobotConfig.PathPlanner.ENABLE_ALLIANCE_FLIP) {
            return false;
        }
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    private void cancelActivePathPlannerAuto() {
        if (activePpAutoCommand != null
            && CommandScheduler.getInstance().isScheduled(activePpAutoCommand)) {
            CommandScheduler.getInstance().cancel(activePpAutoCommand);
        }
        activePpAutoCommand = null;
        pathPlannerRunning = false;
    }

    private double getImuYawDeg() { return imuYawDeg; }
    public boolean isDefaultTeam() { return autoTeam == AutoTeam.DEFAULT; }
    public boolean isRedTeam() { return autoTeam == AutoTeam.RED; }
    public boolean isBlueTeam() { return autoTeam == AutoTeam.BLUE; }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
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

    private static double normalizeSliderToUnit(double rawSlider) {
        // Mirror-map [-1, 1] -> [1, 0] so raw -1 is treated as 1.
        return clamp((1.0 - rawSlider) * 0.5, 0.0, 1.0);
    }
}
