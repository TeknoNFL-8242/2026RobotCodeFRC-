package frc.robot;

/**
 * DigitalTwin: Vision / estimator wrapper
 *
 * Inputs:
 *  - Camera/vision pipeline (ör. AprilTag) veya manuel set edilebilir değerler.
 * Outputs:
 *  - getDistanceMeters() -> Shooter.setDistanceAndCalculate(distance)
 *
 * Kullanım örn:
 *   DigitalTwin dt = new DigitalTwin();
 *   dt.setDistanceMeters(3.2);                      // vision'dan veya estimator'dan alınan değer
 *   shooter.setDistanceAndCalculate(dt.getDistanceMeters());
 */
public class DigitalTwin {
	private double distanceMeters = 0.0;

	/** Vision/estimator tarafından sağlanan yatay mesafeyi ayarlar (metre). */
	public void setDistanceMeters(double meters) {
		this.distanceMeters = meters;
	}

	/** Mevcut mesafeyi döndürür (metre). */
	public double getDistanceMeters() {
		return this.distanceMeters;
	}

	/** (Opsiyonel) Vision verisinden mesafe hesaplama stub'u. */
	public void estimateFromVision(/* vision data */) {
		// ...implemente edilecek: AprilTag veya stereo verisinden hesapla
		// setDistanceMeters( ... );
	}
}

