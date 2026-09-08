package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;


public class Robot extends TimedRobot {
    private final WPI_VictorSPX left1 = new WPI_VictorSPX(3);
    private final WPI_VictorSPX left2 = new WPI_VictorSPX(4);
    private final WPI_VictorSPX right1 = new WPI_VictorSPX(1);
    private final WPI_VictorSPX right2 = new WPI_VictorSPX(2);
    
    private double leftPower = 0;
    private double rightPower = 0;

    private DifferentialDrive m_drive;
    private final LogitechExtreme3DProController m_stick = new LogitechExtreme3DProController(0);
    private final Timer m_timer = new Timer();
    
    private boolean zlock = false;
 
    
    private SendableChooser<String> autoChooser = new SendableChooser<>();
    private String selectedAuto;
    
    private static final String duz_oto = "İleri";
    private static final String geri_oto = "Geri";

    private final double a = 2.9;
    private final double k = 2.3;
    private final double LOG_K = Math.log(1.0 + k);

    @Override
    public void robotInit() {
        left2.follow(left1);
        right2.follow(right1);
        m_drive = new DifferentialDrive(left1, right1);

        autoChooser.setDefaultOption(duz_oto, duz_oto);
        autoChooser.addOption(geri_oto, geri_oto);
        SmartDashboard.putData("Otonom Seçimi", autoChooser);

        CameraServer.startAutomaticCapture();
    }

    @Override
    public void autonomousInit() {
        m_timer.restart();
        selectedAuto = autoChooser.getSelected();
        System.out.println("Seçilen otonom: " + selectedAuto);
    }

    @Override
    public void autonomousPeriodic() {
        if (m_timer.get() < 5.0) {
            switch(selectedAuto) {
                case duz_oto:
                    duz();
                    break;
                case geri_oto:
                    geri();
                    break;
                default:
                    m_drive.tankDrive(0, 0);
                    break;
            }
        } else {
            m_drive.tankDrive(0, 0);
        }
    }
    
    private void duz() {
        m_drive.tankDrive(0.8, 0.8);
    }
    
    private void geri() {
        m_drive.tankDrive(-1, -1);
    }

    @Override
    public void autonomousExit() {
        m_timer.stop();
    }package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.cameraserver.CameraServer;

public class Robot extends TimedRobot {
    private final WPI_VictorSPX left1 = new WPI_VictorSPX(3);
    private final WPI_VictorSPX left2 = new WPI_VictorSPX(4);
    private final WPI_VictorSPX right1 = new WPI_VictorSPX(1);
    private final WPI_VictorSPX right2 = new WPI_VictorSPX(2);
    
    private double leftPower = 0;
    private double rightPower = 0;

    private DifferentialDrive m_drive;
    private final LogitechExtreme3DProController m_stick = new LogitechExtreme3DProController(0);
    private final Timer m_timer = new Timer();
    
    private boolean zlock = false;
 
    private SendableChooser<String> autoChooser = new SendableChooser<>();
    private String selectedAuto;
    
    private static final String duz_oto = "İleri";
    private static final String geri_oto = "Geri";

    private final double a = 2.9;
    private final double k = 2.3;
    private final double LOG_K = Math.log(1.0 + k);

    @Override
    public void robotInit() {
        left2.follow(left1);
        right2.follow(right1);
        m_drive = new DifferentialDrive(left1, right1);

        autoChooser.setDefaultOption(duz_oto, duz_oto);
        autoChooser.addOption(geri_oto, geri_oto);
        SmartDashboard.putData("Otonom Seçimi", autoChooser);

        CameraServer.startAutomaticCapture();
    }

    @Override
    public void autonomousInit() {
        m_timer.restart();
        selectedAuto = autoChooser.getSelected();
        System.out.println("Seçilen otonom: " + selectedAuto);
    }

    @Override
    public void autonomousPeriodic() {
        if (m_timer.get() < 5.0) {
            switch(selectedAuto) {
                case duz_oto:
                    duz();
                    break;
                case geri_oto:
                    geri();
                    break;
                default:
                    m_drive.tankDrive(0, 0);
                    break;
            }
        } else {
            m_drive.tankDrive(0, 0);
        }
    }
    
    private void duz() {
        m_drive.tankDrive(0.8, 0.8);
    }
    
    private void geri() {
        m_drive.tankDrive(-1, -1);
    }

    @Override
    public void autonomousExit() {
        m_timer.stop();
    }

    @Override
    public void teleopPeriodic() {
        double y_ax = -m_stick.getJoystickY();
        double x_ax = -m_stick.getJoystickX();
        double z_ax = -m_stick.getJoystickZ();
        boolean button1 = m_stick.getButtonOne();

        // ---- Z LOCK MANTIĞI (SADECE BURASI EKLENDİ) ----
        double moveDeadband = 0.18;
        double zDeadband = 0.25;

        // X veya Y hareketliyse Z kilitlenir
        if (Math.abs(x_ax) > moveDeadband || Math.abs(y_ax) > moveDeadband) {
            zlock = true;
        }

        // X ve Y merkezdeyse ve Z de deadband içindeyse kilit açılır
        if (Math.abs(x_ax) < moveDeadband && Math.abs(y_ax) < moveDeadband
                && Math.abs(z_ax) < zDeadband) {
            zlock = false;
        }
        // ------------------------------------------------

        if (button1) {
            leftPower = y_ax;
            rightPower = y_ax;
        }
        //else if (zlock && Math.abs(z_ax) > 0) {
        //    contiune
        //}
        else if (Math.abs(x_ax) > 0 && Math.abs(y_ax) > 0) {
            double xAbs = Math.abs(x_ax);
            double yAbs = Math.abs(y_ax);
            double f = Math.log(1.0 + k * xAbs) / LOG_K;
            double g = Math.pow(1.0 - yAbs, a);

            if (x_ax > 0 && y_ax > 0) {
                leftPower = yAbs + yAbs * f * g;
                rightPower = yAbs * (1.0 - f);
            } else if (x_ax < 0 && y_ax > 0) {
                leftPower = yAbs * (1.0 - f);
                rightPower = yAbs + yAbs * f * g;
            } else if (x_ax > 0 && y_ax < 0) {
                leftPower = -yAbs - yAbs * f * g;
                rightPower = -yAbs * (1.0 - f);
            } else if (x_ax < 0 && y_ax < 0) {
                leftPower = -yAbs * (1.0 - f);
                rightPower = -yAbs - yAbs * f * g;
            }
        }
        else if (Math.abs(y_ax) > 0) {
            leftPower = y_ax;
            rightPower = y_ax;
        }
        else if (Math.abs(z_ax) > 0 && !zlock) {
            leftPower = z_ax;
            rightPower = -z_ax;
        }
        else {
            leftPower = 0;
            rightPower = 0;
        }

        m_drive.tankDrive(leftPower, rightPower);
    }
}
@Override
public void teleopPeriodic() {
    double y_ax = -m_stick.getJoystickY();
    double x_ax = -m_stick.getJoystickX();
    double z_ax = -m_stick.getJoystickZ();
    boolean button1 = m_stick.getButtonOne();

    double deadband = 0.12;

    boolean xCentered = Math.abs(x_ax) < deadband;
    boolean yCentered = Math.abs(y_ax) < deadband;
    boolean zActive   = Math.abs(z_ax) > deadband;

    // Robot hareket ediyorsa Z kilitlenir
    if (!xCentered || !yCentered) {
        zlock = true;
    }

    // X ve Y merkeze gelince Z kilidi açılır
    if (xCentered && yCentered) {
        zlock = false;
    }

    if (button1) {
        leftPower = y_ax;
        rightPower = y_ax;
    }
    else if (Math.abs(x_ax) > 0 && Math.abs(y_ax) > 0) {
        double xAbs = Math.abs(x_ax);
        double yAbs = Math.abs(y_ax);
        double f = Math.log(1.0 + k * xAbs) / LOG_K;
        double g = Math.pow(1.0 - yAbs, a);

        if (x_ax > 0 && y_ax > 0) {
            leftPower = yAbs + yAbs * f * g;
            rightPower = yAbs * (1.0 - f);
        } else if (x_ax < 0 && y_ax > 0) {
            leftPower = yAbs * (1.0 - f);
            rightPower = yAbs + yAbs * f * g;
        } else if (x_ax > 0 && y_ax < 0) {
            leftPower = -yAbs - yAbs * f * g;
            rightPower = -yAbs * (1.0 - f);
        } else if (x_ax < 0 && y_ax < 0) {
            leftPower = -yAbs * (1.0 - f);
            rightPower = -yAbs - yAbs * f * g;
        }
    }
    else if (Math.abs(y_ax) > 0) {
        leftPower = y_ax;
        rightPower = y_ax;
    }
    // ✅ Z sadece X ve Y ortadaysa ve kilit kapalıysa çalışır
    else if (zActive && !zlock && xCentered && yCentered) {
        leftPower = z_ax;
        rightPower = -z_ax;
    }
    else {
        leftPower = 0;
        rightPower = 0;
    }

    m_drive.tankDrive(leftPower, rightPower);
}

}