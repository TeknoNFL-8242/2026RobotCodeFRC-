package frc.robot;

//import java.security.Key;

//import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Intakeencoder: küçük wrapper, ortak Encoder.java'daki fonksiyonları kullanır.
 * Bu sayede enkoderla ilgili tekil merkez dosya Encoder.java olur.
 */
public class Intakeencoder {
	// ortak Encoder sınıfı (Encoder.java)
	private final Encoder impl;

	public Intakeencoder() {
		this.impl = new Encoder();
	}

	public double getRPM() {
		return impl.getRPM();
	}

	public void setRPM(double rpm) {
		impl.setRPM(rpm);
	}


	public double getwheelspeed(){

		double rpm = getRPM();
		double WHEELDIAMETERMETERS = 0.116;
		double wheelCircumference = Math.PI * WHEELDIAMETERMETERS;
		double wheelSpeedMetersPerSecond = (rpm / 60.0) * wheelCircumference;
		SmartDashboard.putNumber("intakewheelspeed", wheelSpeedMetersPerSecond);
		return wheelSpeedMetersPerSecond;
		
	}




	

}
