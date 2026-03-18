package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.SPI;

/**
 * RobotConfig
 * Tüm sabitleri tek yerde toplar: portlar, sürüş eğrileri, shooter ayarları,
 * otonom parametreleri, vision dönüşümleri vb.
 */
public final class RobotConfig {
    private RobotConfig() {}

    public static final class Ports {
        private Ports() {}

        // Sürücü joystick USB portu (Driver Station -> USB order)
        public static final int DRIVER_JOYSTICK = 0;

        // PWM ile sürülen bir servo/lever varsa PWM kanalı
        public static final int PWM_SP_LEVER = 0;
        public static final int PWM_SP_TOWER_ELEVATOR_RIGHT = 1;
        public static final int PWM_SP_TOWER_ELEVATOR_LEFT = 2;

        // Drivetrain motor controller CAN ID'leri (VictorSPX vb.)
        public static final int DRIVE_LEFT_1 = 3;
        public static final int DRIVE_LEFT_2 = 4;
        public static final int DRIVE_RIGHT_1 = 1;
        public static final int DRIVE_RIGHT_2 = 2;
        public static final int VC_LEVER_MOTOR = 5; //ekle id

        // Intake/Elevator motorlarının CAN ID'leri
        public static final int ELEVATOR_INTAKE_CAN = 18;
        public static final int INTAKE_RIGHT_CAN = 17;
        public static final int INTAKE_LEFT_CAN = 14;

        //new elevator motor
        public static final int ELEVATOR_NEW_LEFT_CAN = 51;


        // Shooter motor CAN ve harici encoder DIO pinleri
        public static final int SHOOTER_SPARK_CAN = 15;
        public static final int SHOOTER_ENCODER_DIO_A = 0;
        public static final int SHOOTER_ENCODER_DIO_B = 1;

        // Drivetrain encoder DIO pinleri
        // Sag: A=2, B=3 | Sol: A=4, B=5
        public static final int DRIVE_RIGHT_ENCODER_DIO_A = 2;
        public static final int DRIVE_RIGHT_ENCODER_DIO_B = 3;
        public static final int DRIVE_LEFT_ENCODER_DIO_A = 4;
        public static final int DRIVE_LEFT_ENCODER_DIO_B = 5;
    }

    public static final class Drive {
        private Drive() {}

        // Joystick drift için ölü bölge (0.10-0.20 tipik, sahada ayarlanır)
        public static final double AXIS_DEADZONE = 0.15;

        // Gaz eğrisi: küçük inputlarda daha yumuşak kontrol için üssel/power ayarı
        public static final double EXP_A = 3.9;

        // Eğri sertliği (turn/throttle shaping için)
        public static final double CURVE_K = 1.8;

        // WPILib MotorSafety timeout (loop gecikirse motorları keser)
        public static final double MOTOR_SAFETY_EXPIRATION_SEC = 0.15;

        // Z (twist) dönüşü devreye sokma eşiği, istemsiz dönmeyi engeller
        public static final double Z_TURN_ENABLE_THRESHOLD = 0.2;

    }

    public static final class BrownoutProtection {
        private BrownoutProtection() {}

        // VictorSPX drivetrain koruma ayarları
        public static final double VICTOR_VOLTAGE_COMP_SATURATION_V = 12.3;
        public static final double VICTOR_OPEN_LOOP_RAMP_SEC = 0.21 ;

        // SparkMax koruma ayarları
        public static final double SPARK_VOLTAGE_COMP_SATURATION_V = 12.0;
        public static final int SPARK_SMART_CURRENT_LIMIT_A = 36;
        public static final double SPARK_SECONDARY_CURRENT_LIMIT_A = 50.0;
        public static final double SPARK_OPEN_LOOP_RAMP_SEC = 0.2;

        // SparkMax (intake/elevator) - ayrik profil
        public static final double SPARK_INTAKE_ELEVATOR_VOLTAGE_COMP_SATURATION_V = 11.7;

        public static final int SPARK_INTAKE_ELEVATOR_SMART_CURRENT_LIMIT_A = 39;

        public static final double SPARK_INTAKE_ELEVATOR_SECONDARY_CURRENT_LIMIT_A =60;

        public static final double SPARK_INTAKE_ELEVATOR_OPEN_LOOP_RAMP_SEC = 0.3;

        // SparkMax (shooter) - ayrik profil
        public static final double SPARK_SHOOTER_VOLTAGE_COMP_SATURATION_V =
            SPARK_VOLTAGE_COMP_SATURATION_V;
        public static final int SPARK_SHOOTER_SMART_CURRENT_LIMIT_A =
            SPARK_SMART_CURRENT_LIMIT_A;
        public static final double SPARK_SHOOTER_SECONDARY_CURRENT_LIMIT_A =
            SPARK_SECONDARY_CURRENT_LIMIT_A;
        public static final double SPARK_SHOOTER_OPEN_LOOP_RAMP_SEC =
            SPARK_OPEN_LOOP_RAMP_SEC;
        public static final double SPARK_SHOOTER_CLOSED_LOOP_RAMP_SEC = 0.2;
    }

    public static final class Shooter {
        private Shooter() {}

        // Encoder çözünürlüğü (counts per revolution) -> gerçek encoder datasına göre doğrula
        public static final int ENCODER_COUNTS_PER_REV = 800;

        // Kontrol loop zamanı (WPILib robotPeriodic genelde 20ms)
        public static final double LOOP_DT_SEC = 0.02;

        // Hedef RPM'e “yeterince yakın” sayma toleransı (ready-to-shoot)
        public static final double RPM_AT_SPEED_TOLERANCE = 50.0;

        // Yazılım limitleri: hedef RPM clamp için güvenlik
        public static final double MIN_RPM = 500.0;
        public static final double MAX_RPM = 5500.0;

        // PIDF başlangıç değerleri (sahada tune edilir)
        public static final double DEFAULT_KP = 0.0006;
        public static final double DEFAULT_KI = 0.00002;
        public static final double DEFAULT_KD = 0.0001;
        public static final double DEFAULT_KF = 0.00018;

        // PIDF açık/kapalı (test için kapatıp açık loop sürebilirsiniz)
        public static final boolean DEFAULT_USE_PIDF = false;

        // Aşağıdakiler mesafe->RPM balistik hesabı varsa kullanılır (saha kalibrasyonu gerekir)
        public static final double GRAVITY = 9.80;                 // m/s^2
        public static final double SHOOTER_HEIGHT_M = 0.51;        // shooter çıkış yüksekliği
        public static final double TARGET_HEIGHT_M = 1.81;         // hedef yüksekliği (ör: speaker)
        public static final double LAUNCH_ANGLE_DEG = 182.88;        // atış açısı (mekanik ölçülmeli)
        public static final double WHEEL_RADIUS_M = 0.0508;        // 4" wheel radius = 2" = 0.0508m
        public static final double TRANSFER_EFFICIENCY = 0.88;     // slip/enerji kaybı (deneyle)
        public static final double DRAG_CORRECTION_FACTOR = 1.02;  // drag/gerçek dünya düzeltmesi (deneyle)

        // RPM -> rad/s dönüşümü (fizik hesaplarında kullanılır)
        public static final double RPM_TO_RAD_PER_SEC = 2.0 * Math.PI / 60.0;

        // Mesafe sınırları: hesap/aim için güvenli çalışma aralığı
        public static final double MIN_DISTANCE_M = 1.5;
        public static final double MAX_DISTANCE_M = 11.0;
    }

    public static final class Autonomous {
        private Autonomous() {}
        public static final boolean ENABLED = true;
        public static final boolean FORCE_DO_NOTHING = false;

        // Dashboard chooser isimleri (otonom mod seçim stringleri)
        public static final String MODE_COLLECT_RETURN_SHOOT = "collectReturnShoot";
        public static final String MODE_DO_NOTHING = "doNothing";
        public static final String MODE_FORWARD = "ileriGit"; //sadece deneme işlevsel değil
        public static final String MODE_BACKWARD = "geriGit"; //sadece deneme işlevsel değil
        public static final String MODE_ROLL = "roll"; //sadece deneme işlevsel değil
        public static final String MODE_VISION_ALIGN = "visionAlign"; //sadece deneme işlevsel değil
        public static final String MODE_VISION_DRIVE = "visionDrive"; //sadece deneme işlevsel değil
        public static final String MODE_VISION_SCORE = "visionScore"; //sadece deneme işlevsel değil
        public static final String MODE_TAG_ASSIST_SHOOT = "tagAssistShoot";
        public static final String MODE_TOWE_PRELOAD = "TowerPreload";
        public static final String DEFAULT_MODE = MODE_COLLECT_RETURN_SHOOT;

        // Seçenek listesi (SendableChooser için)
        public static final String[] MODE_LIST = new String[] {
            MODE_COLLECT_RETURN_SHOOT,
            MODE_DO_NOTHING,
            MODE_FORWARD,
            MODE_BACKWARD,
            MODE_ROLL,
            MODE_VISION_ALIGN,
            MODE_VISION_DRIVE,
            MODE_VISION_SCORE,
            MODE_TAG_ASSIST_SHOOT,
            MODE_TOWE_PRELOAD
        };

        // collectReturnShoot (default) - güvenli ve fallback'li rutin
        public static final double COLLECT_PREP_SEC = 0.35;
        public static final double COLLECT_PREP_MAX_SEC = 1.20;
        public static final double COLLECT_DRIVE_SEC = 1.20;
        public static final double RETURN_DRIVE_TIMEOUT_SEC = 3.50;
        public static final double RETURN_WAIT_SETTLE_SEC = 0.30;
        public static final double COLLECT_DRIVE_POWER = 0.42;
        public static final double RETURN_DRIVE_POWER = -0.45;
        public static final double RETURN_ALIGN_KP = 0.020;
        public static final double RETURN_ALIGN_MAX = 0.22;
        public static final double RETURN_STOP_DISTANCE_M = 3.30;
        public static final double SEARCH_SPIN_POWER = 0.20;
        public static final double RETURN_SEARCH_SPIN_SEC = 0.45;
        public static final double RETURN_BLIND_STRAIGHT_SEC = 1.10;
        public static final double RETURN_BLIND_ARC_SEC = 0.85;
        public static final double RETURN_BLIND_ARC_BIAS = 0.16;
        public static final double SHOT_SEQUENCE_TOTAL_SEC = 3.0;
        public static final double SHOT_WINDOW_SEC = 1.0;
        public static final double SHOT_RPM_SCALE_CLOSE = 0.60;
        public static final double SHOT_RPM_SCALE_MID = 0.72;
        public static final double SHOT_RPM_SCALE_FAR = 0.84;
        public static final double SHOT_ALIGN_KP = 0.022;
        public static final double SHOT_ALIGN_MAX = 0.20;
        public static final double SHOT_FEED_ACTIVE_SEC = 0.48;
        public static final double SHOT_LEVER_POWER = -0.90;
        public static final double SHOT_INTAKE_POWER = 1.00;
        public static final double SHOT_CLOSE_APPROACH_POWER = 0.10;
        public static final double SHOT_MID_APPROACH_POWER = -0.02;
        public static final double SHOT_FAR_APPROACH_POWER = -0.14;
        public static final double SHOT_SEARCH_SPIN_POWER = 0.14;
        public static final double ROUTINE_MAX_TOTAL_SEC = 14.2;

        // Basit zaman tabanlı otonom sürüş parametreleri
        public static final double FORWARD_DURATION_SEC = 2.0;
        public static final double FORWARD_POWER = 0.5;

        public static final double BACKWARD_DURATION_SEC = 2.0;
        public static final double BACKWARD_POWER = -0.5;

        public static final double ROLL_DURATION_SEC = 0.5;
        public static final double ROLL_LEFT_POWER = 0.5;
        public static final double ROLL_RIGHT_POWER = -0.5;

        // Vision ile hizalama/sürme kontrol parametreleri (sahada tune edilir)
        public static final double ALIGN_TOLERANCE_DEG = 2.0;     // hedefe dönük sayma toleransı
        public static final double TARGET_STOP_DISTANCE_M = 0.50; // tag'e bu mesafede dur

        public static final double KP_ALIGN = 0.025;              // yaw hatası -> turn output
        public static final double KP_DRIVE = 0.4;                // distance hatası -> drive output

        public static final double MAX_TURN_POWER = 0.45;          // maksimum dönüş gücü
        public static final double MAX_DRIVE_POWER = 0.6;        // maksimum ileri güç

        public static final double DRIVE_DEADZONE_M = 0.03;       // çok küçük mesafe hatalarını yok say

        // Faz/timeout değerleri: otonomun takılı kalmasını engeller
        public static final double ALIGN_TIMEOUT_SEC = 3.0;
        public static final double DRIVE_TIMEOUT_SEC = 6.0;

        public static final double SCORE_PHASE0_DURATION_SEC = 1.0;
        public static final double SCORE_PHASE1_TIMEOUT_SEC = 3.0;
        public static final double SCORE_PHASE2_TIMEOUT_SEC = 4.0;

        public static final double SCORE_PHASE0_POWER = 0.4;

        // Sadece belirli target ID'lere (9/10/25/26) kilitlenip atış yapan auto modu.
        public static final int[] TAG_ASSIST_TARGET_IDS = {9, 10, 25, 26};
        public static final double TAG_ASSIST_SHOOT_MIN_DISTANCE_M = 2.55;
        public static final double TAG_ASSIST_SHOOT_MAX_DISTANCE_M = 3.15;
        public static final double TAG_ASSIST_DRIVE_KP = 0.40;
        public static final double TAG_ASSIST_MAX_DRIVE_POWER = 0.30;
        public static final double TAG_ASSIST_SEARCH_SPIN_POWER = 0.16;
        public static final double TAG_ASSIST_STABLE_TIME_SEC = 0.20;
        public static final double TAG_ASSIST_FEED_DURATION_SEC = 0.85;
        public static final double TAG_ASSIST_TIMEOUT_SEC = 8.0;
        public static final double TAG_ASSIST_SHOOT_RPM =
            Shooter.MAX_RPM * SHOT_RPM_SCALE_MID;

        // Tower preload auto (TowerPreload)
        public static final int[] TOWE_PRELOAD_TARGET_IDS = {10, 25};
        public static final double TOWE_PRELOAD_BACKUP_SEC = 2.0;
        public static final double TOWE_PRELOAD_BACKUP_POWER = -0.60;
        public static final double TOWE_PRELOAD_Y_MIN_M = 2.35;
        public static final double TOWE_PRELOAD_Y_MAX_M = 3.9;
        public static final double TOWE_PRELOAD_DISTANCE_KP = 0.35;
        public static final double TOWE_PRELOAD_MAX_DRIVE_POWER = 0.30;
        public static final double TOWE_PRELOAD_SEARCH_SPIN_POWER = 0.16;
        public static final double TOWE_PRELOAD_INTAKE_PRIME_SEC = 0.35;
        public static final double TOWE_PRELOAD_INTAKE_POWER = 0.50;
        public static final double TOWE_PRELOAD_VC_LEVER_POWER = -1.0;
        public static final double TOWE_PRELOAD_SHOOT_DURATION_SEC = 3.0;
        public static final double TOWE_PRELOAD_APPROACH_STOP_M = 1.0;
        public static final double TOWE_PRELOAD_TIMEOUT_SEC = 14.5;

        // Sürüş sırasında dönüş düzeltme (drive + turn karıştırma)
        public static final double DRIVE_TURN_CORRECTION_GAIN = 0.5;
        public static final double DRIVE_TURN_CORRECTION_LIMIT = 0.2;

        // Gyro tabanlı heading-hold PID (düz sürüşte yön koruma)
        public static final double GYRO_HEADING_KP = 0.022;
        public static final double GYRO_HEADING_KI = 0.000;
        public static final double GYRO_HEADING_KD = 0.0012;
        public static final double GYRO_HEADING_MAX_CORRECTION = 0.28;
        public static final double GYRO_HEADING_TOLERANCE_DEG = 1.5;
        public static final double GYRO_CORRECTION_SIGN = 1.0;

        // Gyro tabanlı açıya dönme PID (roll modu)
        public static final double GYRO_TURN_KP = 0.020;
        public static final double GYRO_TURN_KI = 0.000;
        public static final double GYRO_TURN_KD = 0.0010;
        public static final double ROLL_TARGET_DELTA_DEG = 90.0;
        public static final double ROLL_TIMEOUT_SEC = 2.5;
        public static final double ROLL_TOLERANCE_DEG = 2.0;

    }

    public static final class PathPlanner {
        private PathPlanner() {}

        // Master switch: PathPlanner entegrasyonunu tamamen aç/kapat.
        public static final boolean ENABLED = true;

        // true ise autonomous'ta PathPlanner komutu çalıştırılır; false ise mevcut custom auto çalışır.
        public static final boolean RUN_IN_AUTONOMOUS = false;

        // Vision pose ölçümleriyle pose estimator düzeltmesi.
        public static final boolean ENABLE_VISION_FUSION = true;

        // Alliance rengine göre path flip.
        public static final boolean ENABLE_ALLIANCE_FLIP = true;

        // Sensör yön düzeltmeleri (mekanik montaja göre sahada tune edilir).
        public static final boolean GYRO_INVERTED = false;
        public static final boolean LEFT_ENCODER_INVERTED = false;
        public static final boolean RIGHT_ENCODER_INVERTED = true;

        // DIO encoder ham sayımı -> metre dönüşümü.
        // Not: Bu değer robotunuza göre kalibre edilmelidir.
        public static final double DRIVE_ENCODER_DISTANCE_PER_PULSE_M = 0.00025;

        // Diferansiyel kinematik için iz genişliği (sol-sağ teker merkez arası, metre).
        public static final double TRACK_WIDTH_METERS = 0.60;

        // PathPlanner robot-relative hızları open-loop çıkışa çevirmek için.
        public static final double MAX_LINEAR_SPEED_MPS = 3.0;

        // İsteğe bağlı wheel speed PID + FF (PathPlanner sürüş çıkışı için).
        public static final double DRIVE_VEL_KP = 1.2;
        public static final double DRIVE_VEL_KI = 0.0;
        public static final double DRIVE_VEL_KD = 0.0;

        public static final double DRIVE_KS_VOLTS = 0.18;
        public static final double DRIVE_KV_VOLTS_PER_MPS = 2.3;
        public static final double DRIVE_KA_VOLTS_PER_MPS2 = 0.25;
    }

    public static final class Vision {
        private Vision() {}

        // PhotonVision / WPILib camera name (UI'da ne yazıyorsa aynı olmalı)
        public static final String FRONT_CAMERA_NAME = "Front_Camera";
        public static final String BACK_CAMERA_NAME = "Back_Camera";

        // Pose çözümünde belirsizlik limiti: yüksek ambiguity ölçümleri ignore edilir
        public static final double MAX_AMBIGUITY = 0.2;

        // Sezonun field layout'u (tag konumları için)
        public static final AprilTagFields APRILTAG_FIELD = AprilTagFields.k2026RebuiltAndymark;

        // Kamera robot transformu: (x,y,z) metre, rotation radyan
        // Robot merkezine göre kameranın konumu ve baktığı açı
        public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(-0.2875, -0.195, 0.39),
            new Rotation3d(0.0, 0.0, 0.0)
        );

        public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
            new Translation3d(-0.3575, -0.135, 0.34),
            new Rotation3d(0.0, 0.0, Math.PI)
        );
    }

    public static final class Imu {
        private Imu() {}

        // IMU SPI portu (MXP üzerinden SPI)
        public static final SPI.Port SPI_PORT = SPI.Port.kMXP;

        // SPI paket boyutu (kendi protokolünüz varsa)
        public static final int TRANSFER_SIZE = 16;

        // Cihazdan Euler açısı isteme komutu (custom)
        public static final int CMD_REQUEST_EULER = 0x01;

        // SPI işlem öncesi küçük gecikme (bazı sensörler için)
        public static final double PREP_DELAY_SECONDS = 0.001;

        // SPI clock hızı (sensör datasheet limitlerine göre)
        public static final int SPI_CLOCK_HZ = 1_000_000;
    }

    public static final class Operator {
        private Operator() {}

        // Operatör joystick deadzone (ayrı bir deadzone katmanı)
        public static final double JOYSTICK_DEADZONE = 0.0;

        // Teleop shooter slider input min clamp (0-1 arası)
        public static final double SHOOTER_SLIDER_MIN = 0.1;

        // Teleop RPM map: base + slider*range
        public static final double SHOOTER_TELEOP_BASE_RPM = 500.0;
        public static final double SHOOTER_TELEOP_RANGE_RPM = 4500.0;

        // Test modunda daha geniş RPM aralığı
        public static final double SHOOTER_TEST_RANGE_RPM = 5000.0;

        // Intake güç scaling (çok agresif olmasın diye)
        public static final double INTAKE_SCALE = 1;

        // Elevator güçleri (mekanik sürtünmeye göre sahada ayarlanır)
        public static final double ELEVATOR_UP_POWER = 0.12;
        public static final double ELEVATOR_DOWN_POWER = -0.27;
        public static final double ELEVATOR_DOWN_POWER_HARD = -0.40;
        

         public static final double TOWER_ELEVATOR_DOWN_POWER = -0.27;
        public static final double TOWER_ELEVATOR_UP_POWER = -0.27;
        public static final double TOWER_ELEVATOR_LOCK_POWER = -0.27;

    }

    public static final class IntakeElevator {
        private IntakeElevator() {}


        // Position control targets (turns relative to zero) for ball intake elevators' open close
        public static final double OPEN_TURNS_FROM_ZERO = 2.31; // softer open target
        public static final double CLOSE_DOWN_TURNS_FROM_ZERO = -2.36;//test için 2
        public static final double MIN_TURNS_FROM_ZERO = -2.43;
        public static final double MAX_TURNS_FROM_ZERO = 2.34;

        // PID gains (separate tuning for each direction)
        public static final double OPEN_KP = 0.49;
        public static final double OPEN_KI = 0.0;
        public static final double OPEN_KD = 0.022;
        public static final double CLOSE_KP = 0.59;
        public static final double CLOSE_KI = 0.0;
        public static final double CLOSE_KD = 0.059;
        public static final double POSITION_TOLERANCE_TURNS = 0.03;

        // Safety/output
        public static final double HOLD_FEEDFORWARD = 0.0;
        public static final double MAX_OUTPUT = 0.55;
        public static final double CLOSE_PROFILE_START_OUTPUT = 0.84; // far from target (strong start close)
        public static final double CLOSE_PROFILE_END_OUTPUT = 0.00;   // near target
        public static final double CLOSE_PROFILE_WINDOW_TURNS = 3.0;  // error window for ramp
        public static final double OPEN_PROFILE_START_OUTPUT = 0.21;  // far from target (softer open)
        public static final double OPEN_PROFILE_END_OUTPUT = 0.05;    // near target hold/recovery
        public static final double OPEN_PROFILE_WINDOW_TURNS = 3.0;   // error window for ramp

        // Controls
        public static final int BUTTON_SET_ZERO = 7;
        public static final int BUTTON_EMERGENCY_DISABLE = 8;
    }

    public static final class Encoder {
        private Encoder() {}

        // Eğer wheel speed hesaplanıyorsa teker çapı (m)
        public static final double WHEEL_DIAMETER_METERS = 0.116;

        // SmartDashboard/NetworkTables key ismi (telemetri)
        public static final String WHEEL_SPEED_KEY = "intakewheelspeed";

        // Drivetrain encoder telemetry filtre ayarları
        public static final int DRIVE_SAMPLES_TO_AVERAGE = 16;
        public static final int DRIVE_RATE_MEDIAN_WINDOW = 5;
        public static final double DRIVE_RATE_LP_TIME_CONSTANT_SEC = 0.10;

        // Duruyor kabul etme ve jitter bastırma ayarları
        public static final double DRIVE_STOP_MAX_PERIOD_SEC = 0.10;
        public static final double DRIVE_RATE_ZERO_DEADBAND = 5.0;
        public static final double DRIVE_RATE_WAKE_THRESHOLD = 12.0;
        public static final int DRIVE_RATE_WAKE_SAMPLES = 3;
    }

    public static final class Elevator {
        public static final SparkFlexConfig CONFIG = new SparkFlexConfig();

        static {
            CONFIG.idleMode(IdleMode.kBrake);
        }
    }
}
