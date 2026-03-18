package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class LogitechExtreme3DProController {
	
	public Joystick controller;
	public int port;
	
	public LogitechExtreme3DProController(int port) {
		this.port = port;
		this.controller = new Joystick(port);
	}

	// Configuration
	double DEAD_ZONE = RobotConfig.Operator.JOYSTICK_DEADZONE;
	
	public double correctDeadSpot(double value) {
		if (Math.abs(value) < DEAD_ZONE) {
			return 0;
		}
		// Linear scaling: deadzone sonrası 0'dan başlat
		double sign = Math.signum(value);
		return sign * ((Math.abs(value) - DEAD_ZONE) / (1.0 - DEAD_ZONE));
	}

	public boolean getButton(int buttonNumber) {
		return controller.getRawButton(buttonNumber);
	}

	public double getAxis(int axisNumber) {
		return controller.getRawAxis(axisNumber);
	}

	public int getPOV(int povNumber) {
		return controller.getPOV(povNumber);
	}

	public double getThrottle() {
		return controller.getThrottle();
	}

	
	// Joystick
	
	public double getJoystickX() {
		return correctDeadSpot(getAxis(0));
	}
	
	public double getJoystickY() {
		return correctDeadSpot(getAxis(1));
	}
	
	public double getJoystickZ() {
		return correctDeadSpot(getAxis(2));
	}


	// Numeral Buttons

	public boolean getButtonOne() {
		return getButton(1);
	}

	public boolean getButtonTwo() {
		return getButton(2);
	}

	public boolean getButtonThree() {
		return getButton(3);
	}

	public boolean getButtonFour() {
		return getButton(4);
	}

	public boolean getButtonFive() {
		return getButton(5);
	}

	public boolean getButtonSix() {
		return getButton(6);
	}

	public boolean getButtonSeven() {
		return getButton(7);
	}

	public boolean getButtonEight() {
		return getButton(8);
	}

	public boolean getButtonNine() {
		return getButton(9);
	}

	public boolean getButtonTen() {
		return getButton(10);
	}

	public boolean getButtonEleven() {
		return getButton(11);
	}

	public boolean getButtonTwelve() {
		return getButton(12);
	}

	// Slider

	public double getSlider() {
		return getThrottle();
	}

	// DPad

	public boolean getDPadUp(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree >= 337 || degree <= 22);
	}

	public boolean getDPadDown(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree <= 202 && degree >= 157);
	}

	public boolean getDPadLeft(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree <= 292 && degree >= 247);
	}

	public boolean getDPadRight(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree <= 112 && degree >= 67);
	}

	public boolean getDPadUpRight(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree >= 22 && degree <= 67);
	}

	public boolean getDPadUpLeft(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree <= 337 && degree >= 292);
	}

	public boolean getDPadDownRight(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree <= 157 && degree >= 112);
	}

	public boolean getDPadDownLeft(){
		int degree = getPOV(0);
		if (degree < 0) return false;
		return (degree <= 247 && degree >= 207);
	}
	
}
