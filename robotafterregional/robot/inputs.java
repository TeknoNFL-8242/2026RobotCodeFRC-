package frc.robot;

/**
 * inputs.java
 *
 * Bu dosya bir "doküman/örnek" olarak bırakıldı: sensörlerin ve hesaplayıcıların
 * birbirleriyle nasıl konuştuğunu kısa bir kod örneği şeklinde gösterir.
 *
 * Örnek akış (kullanım):
 *
 *   // 1) Başlatma (robotInit içinde)
 *   ShooterSubsystem shooter = new ShooterSubsystem();
 *   DigitalTwin dt = new DigitalTwin();          // vision/estimator
 *
 *   // 2) Vision okuması geldiğinde
 *   dt.setDistanceMeters(3.2);                   // metre cinsinden
 *   shooter.shootAtDistance(dt.getDistanceMeters());
 *
 *   // 3) Teleop veya periodic içinde motora uygula (Robot.java içinde)
 *   // shooter.periodic() Robot.robotPeriodic() icinde cagirilir
 *
 * Bu dosya sadece ilişkiyi gösterir; gerçek başlatma Robot.java içinde yapılmalıdır.
 */
public class inputs {
    // Bu sınıf çalışma zamanı kodu içermez, sadece örnek/document amaçlı bırakıldı.
}
