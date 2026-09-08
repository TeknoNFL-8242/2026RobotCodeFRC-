package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Robot extends TimedRobot {
    
    // ═══════════════════════════════════════════════════════════════════════
    // SUBSYSTEMS
    // ═══════════════════════════════════════════════════════════════════════
    private ShooterSubsystem shooterSubsystem;
    
    // ═══════════════════════════════════════════════════════════════════════
    // DRIVETRAIN HARDWARE
    // ═══════════════════════════════════════════════════════════════════════
    private final WPI_VictorSPX left1 = new WPI_VictorSPX(3);
    private final WPI_VictorSPX left2 = new WPI_VictorSPX(4);
    private final WPI_VictorSPX right1 = new WPI_VictorSPX(1);
    private final WPI_VictorSPX right2 = new WPI_VictorSPX(2);

    private final SparkMax elevatorIntake = new SparkMax(18, MotorType.kBrushless);

    private final SparkMax intakeR = new SparkMax(17, MotorType.kBrushless);
    private final SparkMax intakeL = new SparkMax(14, MotorType.kBrushless);
    // private final PWMSparkMax spShooter = new PWMSparkMax(0);  //spShooter.set(0.8); // %80 ,example for use 
    //private final PWMSparkMax spIntake = new PWMSparkMax(1); 

     private final PWMSparkMax spLever = new PWMSparkMax(0); //the ball forwarder -to shooter-
    //private final PWMSparkMax spLinearel = new PWMSparkMax(3); 
    // private SparkMax spShooter = new SparkMax(15, MotorType.kBrushless);
   // private SparkMax spForwardLever = new SparkMax(6, MotorType.kBrushless);

    


    // Mekanizma Motorları (El/Asansör vb.) eski koddan FRC 2025 elevator
    //private final WPI_VictorSPX el1 = new WPI_VictorSPX(5);
    //private final WPI_VictorSPX el2 = new WPI_VictorSPX(6);

    private double leftPower = 0;
    private double rightPower = 0;
    private boolean zLock = false;
    private double deadzone = 0.15; //mutlak değerle kullan bunu 
    
    
    private DifferentialDrive m_drive;
    private final LogitechExtreme3DProController m_stick = new LogitechExtreme3DProController(0);
    
    private final Timer m_timer = new Timer();
    
// Otonom Seçici
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    private final String kAutoIleri = "ileriGit"; // Türkçe karakter yok
    private final String kAutoGeri = "geriGit";
    private final String kAutoRoll = "roll";
    private String m_autoSelected;

    private final double a = 3.9; // boost değeri
    private final double k = 1.8;
    ; // Logaritmik dönüş hassasiyeti
    private final double LOG_K = Math.log(1.0 + k); // Sabit değer, bir kere hesaplanır

	// --- Drivetrain encoderlar DEVRE DIŞI (sadece Flywheel encoder DIO 0-1'de kullanılıyor)
	// private edu.wpi.first.wpilibj.Encoder leftEncoder;
	// private edu.wpi.first.wpilibj.Encoder rightEncoder;
	//private DigitalInput leftIndex;   // opsiyonel Z/index girişleri
	//private DigitalInput rightIndex;

    // Encoder sabitleri - şimdilik devre dışı

    private Esp32Bridge imuBridge;

    @Override
    public void robotInit() {
        // ─────────────────────────────────────────────────────────────────────
        // Initialize Subsystems
        // ─────────────────────────────────────────────────────────────────────
        shooterSubsystem = new ShooterSubsystem();
        System.out.println("✓ ShooterSubsystem initialized");
        
        // Arka motorları ön motorlara bağlıyoruz (Follower modu)
        left2.follow(left1);
        right2.follow(right1);

        // DifferentialDrive sadece ana motorları bilmeli
        m_drive = new DifferentialDrive(left1, right1);

        // Motor safety tolerance: loop overrun durumunda hemen kesilmesin diye biraz tolerans
        m_drive.setExpiration(0.1);

        // 1. Chooser'ı hazırla
        m_chooser.setDefaultOption("ileriGit", kAutoIleri);
        m_chooser.addOption("geriGit", kAutoGeri);
        m_chooser.addOption("roll", kAutoRoll);
        
        // 2. Bu satır NetworkTables'a Chooser olarak gönderir
        SmartDashboard.putData("Select Autonomous ...", m_chooser);

        // 3. KRİTİK: Bazı eski Dashboard'lar sadece String Array anlar. 
        // Eğer kutu hala boşsa bu satır onu zorla dolduracak.
        SmartDashboard.putStringArray("Auto List", new String[] {"ileriGit", "geriGit", "roll"});

        CameraServer.startAutomaticCapture();

        // Drivetrain encoder kodları devre dışı - sadece flywheel encoder (DIO 0-1) kullanılıyor
        
        imuBridge = new Esp32Bridge(); // veya new Esp32Bridge(<DIO_pin>) eğer manuel CS kullanacaksanız
    }

    @Override
    public void autonomousInit() {
        m_timer.restart();
        // Dashboard'dan o an seçili olan stringi çekiyoruz
        // Seçimi buradan alıyoruz
        m_autoSelected = SmartDashboard.getString("Auto Selector", "ileriGit");
        System.out.println("Calisan Mod: " + m_autoSelected);
    }



    @Override
    public void autonomousPeriodic() {
        // 1.0 saniye dolana kadar çalış
        if (m_timer.get() < 1.0) {
            
            // m_autoSelected değerine göre karar ver
            switch (m_autoSelected) {
                case kAutoGeri:
                    geri();
                    break;

                case kAutoIleri:
                    duz();
                    break;

                case kAutoRoll:
                    rollforauto();
                    break;

                default:
                    // Eğer hiçbir seçenek eşleşmezse motorları durdur (Güvenlik)
                    m_drive.tankDrive(0, 0);
                    break;
            }

        } else {
            // Zaman dolduğunda dur
            m_drive.tankDrive(0, 0);
        }
    }
    
    private void duz() {
        m_drive.tankDrive(0.5, 0.5);
    }
    
    private void geri() {
        m_drive.tankDrive(-0.5, -0.5);
    }

    private void rollforauto(){
        m_drive.tankDrive(0.5, -0.5);
    }

    @Override
    public void autonomousExit() {
        m_timer.stop();
    }

    @Override
    public void testInit() {
        // Test moduna girildiğinde bir kere çağrılır
        System.out.println("==== TEST INIT ====");
        SmartDashboard.putString("Robot/Mode", "TEST");
    }

    @Override
    public void testPeriodic() {
        // This method is called periodically during test mode
        System.out.println("Test Periodic Executing");
        SmartDashboard.putNumber("Test/testPeriodicTimestamp", System.currentTimeMillis());
        
        boolean buttonElleftup = m_stick.getButtonFive();
        boolean buttonErightdown = m_stick.getButtonThree();
        
        // Test Shooter via RPM slider
        double shooterSlider = m_stick.getSlider();
        if (shooterSlider > 0.1) {
            // Map slider (0-1) to RPM range: 0 to 5000 RPM
            double testRPM = shooterSlider * 5000.0;
            shooterSubsystem.setTargetRPM(testRPM);
        } else {
            shooterSubsystem.stop();
        }
        
        // Test Other Mechanisms
        spLever.set(shooterSlider);
        intakeL.set(-shooterSlider * 0.3);
        intakeR.set(shooterSlider * 0.3);

        if (buttonElleftup == true){
            elevatorIntake.set(0.12);
        } else if (buttonErightdown == true){
            elevatorIntake.set(-0.19);
        } else {
            elevatorIntake.set(0);
        }



       

        float[] e = imuBridge.readEuler();
        if (e != null) {
            System.out.println("==== GYRO: OK ====");
            SmartDashboard.putNumber("IMU Roll",  e[0]);
            SmartDashboard.putNumber("IMU Pitch", e[1]);
            SmartDashboard.putNumber("IMU Yaw",   e[2]);
        }
    }
    
    @Override
    public void teleopPeriodic() {
        double y_ax = -m_stick.getJoystickY();  // İleri/Geri
        double x_ax = -m_stick.getJoystickX();  // Sağa/Sola
        double z_ax = -m_stick.getJoystickZ();  // Yerinde dönüş
        boolean xCentered = Math.abs(x_ax) < deadzone;
        boolean yCentered = Math.abs(y_ax) < deadzone;
        boolean zCentered = Math.abs(z_ax) < deadzone;
        boolean button1 = m_stick.getButtonOne();

        // FIX: shooterFlywhell.set(0.3) buradan kaldırıldı. 
        // Slider ile aşağıda kontrol ediliyor; burada koşulsuz set etmek çakışmaya yol açıyordu.

        if (!xCentered || !yCentered) {
            zLock = true;
        }

        // Kilidi açmak için: X, Y ve Z'nin AYNI ANDA merkeze (sıfıra) gelmiş olması lazım
        if (xCentered && yCentered && zCentered) {
            zLock = false;
        }

        if (button1) {
            leftPower = y_ax;
            rightPower = y_ax;
        }
        else if ((Math.abs(z_ax) > 0.2  && Math.abs(x_ax) < 0.2 && Math.abs(y_ax) < 0.2) && (!zLock)) { //added zLock safety feature 
            leftPower = z_ax;
            rightPower = -z_ax;    
        }
        else if (Math.abs(x_ax) > 0 && Math.abs(y_ax) > 0) {
            double xAbs = Math.abs(x_ax);
            double yAbs = Math.abs(y_ax);

            // Logaritmik dönüş oranı (0..1)
            double f = Math.log(1.0 + k * xAbs) / LOG_K;

            // düşük hızda dış paleti boost'layan g(y)
            double g = Math.pow(1.0 - yAbs, a);

            if (x_ax > 0 && y_ax > 0) {
                // İleri + sağa dönüş
                leftPower = yAbs + yAbs * f * g;
                rightPower = yAbs * (1.0 - f);

            } else if (x_ax < 0 && y_ax > 0) {
                // İleri + sola dönüş
                leftPower = yAbs * (1.0 - f);
                rightPower = yAbs + yAbs * f * g;

            } else if (x_ax > 0 && y_ax < 0) {
                // Geri + sağa dönüş
                leftPower = -yAbs - yAbs * f * g;
                rightPower = -yAbs * (1.0 - f);

            } else if (x_ax < 0 && y_ax < 0) {
                // Geri + sola dönüş
                leftPower = -yAbs * (1.0 - f);
                rightPower = -yAbs - yAbs * f * g;
            }
        }
        else if (Math.abs(y_ax) > 0) {
            leftPower = y_ax;
            rightPower = y_ax;
        }
        else {
            leftPower = 0;
            rightPower = 0;
        }

        m_drive.tankDrive(leftPower, rightPower);

        // ─────────────────────────────────────────────────────────────────────
        // SHOOTER CONTROL via ShooterSubsystem
        // ─────────────────────────────────────────────────────────────────────
        double shooterSlider = m_stick.getSlider();
        if (shooterSlider > 0.1) {
            // Map slider (0-1) to RPM range: 500 to 5000 RPM
            double targetRPM = 500.0 + (shooterSlider * 4500.0);
            shooterSubsystem.setTargetRPM(targetRPM);
        } else {
            shooterSubsystem.stop();
        }
    }
    
    @Override
    public void teleopExit() {
        shooterSubsystem.stop();
    }
}