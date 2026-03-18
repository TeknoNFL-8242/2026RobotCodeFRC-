package frc.robot;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.HashSet;
import java.util.Map;
import java.util.HashMap;

/**
 * VisionSubsystem
 *
 * PhotonVision ile iki kamerayı (Front_Camera, Back_Camera) yönetir.
 * AprilTag'lerden:
 *   - Hedef Yaw (derece)  → robotun hedefe göre sağ/sol açısı
 *   - Hedef Pitch (derece) → robotun hedefe göre yukarı/aşağı açısı
 *   - Hedef Mesafesi (metre)
 *   - AprilTag ID
 *   - Field-relative robot pozu (Pose2d) — her iki kameradan birleşik
 *
 * Kamera fiziksel konumları aşağıdaki sabitlerde ayarlanır.
 * Robota göre X=ileri, Y=sol, Z=yukarı (WPILib NED değil, ENU standardı).
 *
 *  ┌─────────────────────────────────────────────────────────────┐
 *  │  ÖNEMLİ: Kamera offset'lerini robotunuza göre güncelleyin!  │
 *  │  FRONT_CAMERA_TRANSFORM ve BACK_CAMERA_TRANSFORM sabitleri  │
 *  └─────────────────────────────────────────────────────────────┘
 */
public class VisionSubsystem {

    // ═══════════════════════════════════════════════════════════════════════
    // KAMERA FİZİKSEL KONUMLARI — Robota göre ölçün ve güncelleyin!
    // X = ileri (metre), Y = sol (metre), Z = yukarı (metre)
    // Rotation3d(roll, pitch, yaw) — radyan cinsinden
    // ═══════════════════════════════════════════════════════════════════════

    // ═══════════════════════════════════════════════════════════════════════
    // KAMERA ve POSE ESTIMATOR NESNELERİ
    // ═══════════════════════════════════════════════════════════════════════
    private final PhotonCamera frontCamera;
    private final PhotonCamera backCamera;
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator backPoseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    // ═══════════════════════════════════════════════════════════════════════
    // ANLIK VERİLER — update() çağrıldığında güncellenir
    // ═══════════════════════════════════════════════════════════════════════

    // Ön kamera
    private boolean frontHasTarget   = false;
    private double  frontYaw         = 0.0;   // derece, + = hedef sağda
    private double  frontPitch       = 0.0;   // derece, + = hedef yukarıda
    private double  frontDistance    = 0.0;   // metre, kameradan hedefe 3D mesafe
    private int     frontTagId       = -1;
    private double  frontAmbiguity   = 1.0;   // 0..1, düşük = daha güvenilir
    private Optional<EstimatedRobotPose> frontEstimatedPose = Optional.empty();
    private final Set<Integer> frontVisibleTagIds = new HashSet<>();
    private final Map<Integer, Double> frontYawByTagId = new HashMap<>();
    private final Map<Integer, Double> frontDistanceByTagId = new HashMap<>();

    // Arka kamera
    private boolean backHasTarget    = false;
    private double  backYaw          = 0.0;
    private double  backPitch        = 0.0;
    private double  backDistance     = 0.0;
    private int     backTagId        = -1;
    private double  backAmbiguity    = 1.0;
    private Optional<EstimatedRobotPose> backEstimatedPose = Optional.empty();
    private final Set<Integer> backVisibleTagIds = new HashSet<>();
    private final Map<Integer, Double> backYawByTagId = new HashMap<>();
    private final Map<Integer, Double> backDistanceByTagId = new HashMap<>();

    // System status
    private long updateCount = 0;

    // ═══════════════════════════════════════════════════════════════════════
    // CONSTRUCTOR
    // ═══════════════════════════════════════════════════════════════════════
    public VisionSubsystem() {
        frontCamera = new PhotonCamera(RobotConfig.Vision.FRONT_CAMERA_NAME);
        backCamera  = new PhotonCamera(RobotConfig.Vision.BACK_CAMERA_NAME);

        // 2025 Reefscape field layout — farklı sezon için burayı değiştirin
        fieldLayout = AprilTagFieldLayout.loadField(RobotConfig.Vision.APRILTAG_FIELD);

        frontPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  // Birden fazla tag varsa en doğrusu
            RobotConfig.Vision.FRONT_CAMERA_TRANSFORM
        );
        backPoseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            RobotConfig.Vision.BACK_CAMERA_TRANSFORM
        );

        // Tek tag görünce fallback: en az belirsiz olanı seç
        frontPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        backPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        System.out.println("✓ VisionSubsystem başlatıldı (Front_Camera + Back_Camera)");
    }

    // ═══════════════════════════════════════════════════════════════════════
    // ANA GÜNCELLEME — Her periyodik döngüde çağrılmalı
    // ═══════════════════════════════════════════════════════════════════════

    /**
     * Her iki kamerayı okur, verileri günceller, SmartDashboard'a yazar.
     * Robot.java'daki robotPeriodic() içinde çağırın.
     */
    public void update() {
        updateCount++;
        frontYawByTagId.clear();
        frontDistanceByTagId.clear();
        backYawByTagId.clear();
        backDistanceByTagId.clear();
        processCameraResults(frontCamera, frontPoseEstimator, true);
        processCameraResults(backCamera,  backPoseEstimator,  false);
        publishToDashboard();
    }

    // ═══════════════════════════════════════════════════════════════════════
    // İÇ İŞLEME MANTIĞI
    // ═══════════════════════════════════════════════════════════════════════

    private void processCameraResults(PhotonCamera camera,
                                      PhotonPoseEstimator estimator,
                                      boolean isFront) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (results.isEmpty()) {
            clearCameraState(isFront);
            return;
        }

        // En son sonucu al (birden fazla kare birikmiş olabilir)
        PhotonPipelineResult latestResult = results.get(results.size() - 1);

        if (!latestResult.hasTargets()) {
            clearCameraState(isFront);
            return;
        }

        // ─── En iyi hedef (en yüksek alan, en az ambiguity) ───
        PhotonTrackedTarget best = latestResult.getBestTarget();
        Set<Integer> visibleIds = new HashSet<>();
        for (PhotonTrackedTarget target : latestResult.getTargets()) {
            int id = target.getFiducialId();
            if (id >= 0) {
                visibleIds.add(id);
                Transform3d cameraToTarget = target.getBestCameraToTarget();
                double targetDistance = cameraToTarget.getTranslation().getNorm();
                double targetYaw = target.getYaw();
                if (isFront) {
                    frontYawByTagId.put(id, targetYaw);
                    frontDistanceByTagId.put(id, targetDistance);
                } else {
                    backYawByTagId.put(id, targetYaw);
                    backDistanceByTagId.put(id, targetDistance);
                }
            }
        }

        double yaw      = best.getYaw();
        double pitch    = best.getPitch();
        double ambig    = best.getPoseAmbiguity();
        int    tagId    = best.getFiducialId();

        // 3D mesafe: kameradan hedefe olan vektörün uzunluğu
        Transform3d camToTarget = best.getBestCameraToTarget();
        double distance = camToTarget.getTranslation().getNorm();

        // ─── Pose tahmini (yalnızca ambiguity düşükse) ───
        Optional<EstimatedRobotPose> estimatedPose = Optional.empty();
        if (ambig < RobotConfig.Vision.MAX_AMBIGUITY || latestResult.getTargets().size() > 1) {
            estimatedPose = estimator.update(latestResult);
        }

        // ─── Verileri kaydet ───
        if (isFront) {
            frontHasTarget      = true;
            frontYaw            = yaw;
            frontPitch          = pitch;
            frontDistance       = distance;
            frontTagId          = tagId;
            frontAmbiguity      = ambig;
            frontEstimatedPose  = estimatedPose;
            frontVisibleTagIds.clear();
            frontVisibleTagIds.addAll(visibleIds);
        } else {
            backHasTarget       = true;
            backYaw             = yaw;
            backPitch           = pitch;
            backDistance        = distance;
            backTagId           = tagId;
            backAmbiguity       = ambig;
            backEstimatedPose   = estimatedPose;
            backVisibleTagIds.clear();
            backVisibleTagIds.addAll(visibleIds);
        }
    }

    private void clearCameraState(boolean isFront) {
        if (isFront) {
            frontHasTarget = false;
            frontYaw = 0.0;
            frontPitch = 0.0;
            frontDistance = 0.0;
            frontTagId = -1;
            frontAmbiguity = 1.0;
            frontEstimatedPose = Optional.empty();
            frontVisibleTagIds.clear();
            frontYawByTagId.clear();
            frontDistanceByTagId.clear();
        } else {
            backHasTarget = false;
            backYaw = 0.0;
            backPitch = 0.0;
            backDistance = 0.0;
            backTagId = -1;
            backAmbiguity = 1.0;
            backEstimatedPose = Optional.empty();
            backVisibleTagIds.clear();
            backYawByTagId.clear();
            backDistanceByTagId.clear();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // SMARTDASHBOARD ÇIKTILARI
    // ═══════════════════════════════════════════════════════════════════════

    private void publishToDashboard() {
        // Vision system status
        SmartDashboard.putString("Vision/Status", "Active");
        SmartDashboard.putNumber("Vision/UpdateCount", updateCount);
        
        // ─── Ön Kamera ───
        SmartDashboard.putBoolean("Vision/Front/HasTarget",   frontHasTarget);
        SmartDashboard.putNumber ("Vision/Front/TagID",       frontTagId);
        SmartDashboard.putNumber ("Vision/Front/Yaw_deg",     frontYaw);
        SmartDashboard.putNumber ("Vision/Front/Pitch_deg",   frontPitch);
        SmartDashboard.putNumber ("Vision/Front/Distance_m",  frontDistance);
        SmartDashboard.putNumber ("Vision/Front/Ambiguity",   frontAmbiguity);

        // ─── Arka Kamera ───
        SmartDashboard.putBoolean("Vision/Back/HasTarget",    backHasTarget);
        SmartDashboard.putNumber ("Vision/Back/TagID",        backTagId);
        SmartDashboard.putNumber ("Vision/Back/Yaw_deg",      backYaw);
        SmartDashboard.putNumber ("Vision/Back/Pitch_deg",    backPitch);
        SmartDashboard.putNumber ("Vision/Back/Distance_m",   backDistance);
        SmartDashboard.putNumber ("Vision/Back/Ambiguity",    backAmbiguity);

        // ─── Birleşik Poz Tahmini ───
        Pose2d bestPose = getBestEstimatedPose();
        if (bestPose != null) {
            SmartDashboard.putNumber("Vision/Robot/X_m",        bestPose.getX());
            SmartDashboard.putNumber("Vision/Robot/Y_m",        bestPose.getY());
            SmartDashboard.putNumber("Vision/Robot/Heading_deg",bestPose.getRotation().getDegrees());
        }
        SmartDashboard.putBoolean("Vision/Robot/PoseValid", bestPose != null);
    }

    // ═══════════════════════════════════════════════════════════════════════
    // PUBLIC GETTER'LAR — autonom.java ve Robot.java'dan kullanılır
    // ═══════════════════════════════════════════════════════════════════════

    // ─── Ön Kamera Getter'ları ───

    /** Ön kamerada hedef var mı? */
    public boolean frontHasTarget()     { return frontHasTarget; }

    /** Ön kamera hedef açısı (derece). Pozitif = hedef sağda. */
    public double  getFrontYaw()        { return frontYaw; }

    /** Ön kamera dikey açısı (derece). Pozitif = hedef yukarıda. */
    public double  getFrontPitch()      { return frontPitch; }

    /** Ön kameradan hedefe metre cinsinden 3D mesafe. */
    public double  getFrontDistance()   { return frontDistance; }

    /** Ön kameranın gördüğü en iyi AprilTag ID (-1 = yok). */
    public int     getFrontTagId()      { return frontTagId; }

    /** Ön kamera pose ambiguity (0=kesin, 1=belirsiz). */
    public double  getFrontAmbiguity()  { return frontAmbiguity; }

    // ─── Arka Kamera Getter'ları ───

    /** Arka kamerada hedef var mı? */
    public boolean backHasTarget()      { return backHasTarget; }

    /** Arka kamera hedef açısı (derece). */
    public double  getBackYaw()         { return backYaw; }

    /** Arka kamera dikey açısı (derece). */
    public double  getBackPitch()       { return backPitch; }

    /** Arka kameradan hedefe metre cinsinden mesafe. */
    public double  getBackDistance()    { return backDistance; }

    /** Arka kameranın gördüğü en iyi AprilTag ID (-1 = yok). */
    public int     getBackTagId()       { return backTagId; }

    /** Arka kamera pose ambiguity. */
    public double  getBackAmbiguity()   { return backAmbiguity; }

    // ─── Poz Tahminleri ───

    /** Ön kameranın robot poz tahmini (Optional). */
    public Optional<EstimatedRobotPose> getFrontEstimatedPose() {
        return frontEstimatedPose;
    }

    /** Arka kameranın robot poz tahmini (Optional). */
    public Optional<EstimatedRobotPose> getBackEstimatedPose() {
        return backEstimatedPose;
    }

    /**
     * İki kameradan en güvenilir poz tahminini döner.
     * Ambiguity değeri düşük olan tercih edilir.
     * Hiç tahmin yoksa null döner.
     */
    public Pose2d getBestEstimatedPose() {
        Optional<EstimatedRobotPose> chosen = Optional.empty();
        double bestAmbig = RobotConfig.Vision.MAX_AMBIGUITY;

        if (frontEstimatedPose.isPresent() && frontAmbiguity < bestAmbig) {
            bestAmbig = frontAmbiguity;
            chosen = frontEstimatedPose;
        }
        if (backEstimatedPose.isPresent() && backAmbiguity < bestAmbig) {
            chosen = backEstimatedPose;
        }

        return chosen.map(p -> p.estimatedPose.toPose2d()).orElse(null);
    }

    /**
     * Belirli bir tag ID'ye olan mesafeyi döner.
     * Önce ön kameraya bakar, sonra arka kameraya.
     * @param tagId İstenen AprilTag ID
     * @return Metre cinsinden mesafe, bulunamazsa -1.0
     */
    public double getDistanceToTag(int tagId) {
        if (frontDistanceByTagId.containsKey(tagId)) return frontDistanceByTagId.get(tagId);
        if (backDistanceByTagId.containsKey(tagId)) return backDistanceByTagId.get(tagId);
        return -1.0;
    }

    /**
     * Ön kameradan belirli tag'e olan mesafeyi döner.
     * @param tagId İstenen AprilTag ID
     * @return Metre cinsinden mesafe, bulunamazsa -1.0
     */
    public double getFrontDistanceToTag(int tagId) {
        if (frontDistanceByTagId.containsKey(tagId)) return frontDistanceByTagId.get(tagId);
        return -1.0;
    }

    /**
     * Belirli bir tag ID'ye olan yaw açısını döner.
     * @param tagId İstenen AprilTag ID
     * @return Derece cinsinden yaw, bulunamazsa 0.0
     */
    public double getYawToTag(int tagId) {
        if (frontYawByTagId.containsKey(tagId)) return frontYawByTagId.get(tagId);
        if (backYawByTagId.containsKey(tagId)) return backYawByTagId.get(tagId);
        return 0.0;
    }

    /**
     * Ön kameradan belirli tag'e olan yaw açısını döner.
     * @param tagId İstenen AprilTag ID
     * @return Derece cinsinden yaw, bulunamazsa 0.0
     */
    public double getFrontYawToTag(int tagId) {
        if (frontYawByTagId.containsKey(tagId)) return frontYawByTagId.get(tagId);
        return 0.0;
    }

    /**
     * Field coordinate systeminde belirli bir AprilTag'in 2D pozunu döner.
     * @param tagId İstenen AprilTag ID
     * @return Tag pozu, bulunamazsa Optional.empty()
     */
    public Optional<Pose2d> getFieldTagPose2d(int tagId) {
        return fieldLayout.getTagPose(tagId).map(pose3d -> pose3d.toPose2d());
    }

    /**
     * Herhangi bir kamerada hedef görünüyor mu?
     */
    public boolean hasAnyTarget() {
        return frontHasTarget || backHasTarget;
    }

    /**
     * İstenen tag ID herhangi bir kamerada görünüyorsa true döner.
     */
    public boolean isTagVisible(int tagId) {
        return frontVisibleTagIds.contains(tagId)
            || backVisibleTagIds.contains(tagId)
            || frontYawByTagId.containsKey(tagId)
            || backYawByTagId.containsKey(tagId);
    }

    /**
     * İstenen tag yalnızca ön kamerada görünüyorsa true döner.
     */
    public boolean isFrontTagVisible(int tagId) {
        return frontVisibleTagIds.contains(tagId)
            || frontYawByTagId.containsKey(tagId)
            || frontDistanceByTagId.containsKey(tagId);
    }

    /**
     * Verilen listeden herhangi bir tag görünüyorsa true döner.
     */
    public boolean isAnyTagVisible(int[] tagIds) {
        for (int tagId : tagIds) {
            if (isTagVisible(tagId)) {
                return true;
            }
        }
        return false;
    }
}
