package frc.robot;

//import com.fasterxml.jackson.databind.PropertyNamingStrategies.NamingBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Merkezi enkoder stub'u. İhtiyaç halinde bu sınıf WPILib Encoder ile sarmalanıp
 * gerçek donanıma bağlanır.
 *
 * Kullanım: new Encoder() -> getRPM(), setRPM(...)
 */
public class Encoder {

	private double rpm = 0.0;

	/** Gerçek encoder'dan RPM okunacak. Şu an stub. */
	public double getRPM() {
		//TODO: encoder.getRate() * 60.0 vb. ile bağla
		return rpm;
	}

	public void setRPM(double rpm) { this.rpm = rpm; }

		public double getwheelspeed(){

		double rpm = getRPM();
		double WHEELDIAMETERMETERS = 0.116;
		double wheelCircumference = Math.PI * WHEELDIAMETERMETERS;
		double wheelSpeedMetersPerSecond = (rpm / 60.0) * wheelCircumference;
		SmartDashboard.putNumber("intakewheelspeed", wheelSpeedMetersPerSecond);
		return wheelSpeedMetersPerSecond;
	
		
	    
		
	}

}


