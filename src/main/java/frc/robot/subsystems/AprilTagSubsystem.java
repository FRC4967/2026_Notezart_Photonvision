package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagSubsystem extends SubsystemBase implements BotPoseSource {

    public record VisionObservation(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    // Mechanical Advantage-style std dev coefficients:
    // stdDev = coefficient * distance^1.2 / numTags^2
    // Smaller = more trust in vision. Tune these if pose jumps too much.
    private static final double XY_STD_DEV_COEFF    = 0.01;
    private static final double THETA_STD_DEV_COEFF  = 0.03;

    // Reject single-tag estimates with ambiguity above this (0=perfect, 1=worst)
    private static final double AMBIGUITY_THRESHOLD  = 0.3;

    private static final double FIELD_BORDER_MARGIN  = 0.5; // meters
    private static final double FIELD_LENGTH         = 16.54;
    private static final double FIELD_WIDTH          = 8.21;

    // -----------------------------------------------------------------------
    // Known from design spec:
    //   Lateral offset:  ±11.25 cm from robot centerline
    //   Yaw:             28.5° outward (right cam = -28.5°, left cam = +28.5°)
    //
    //   CAMERA_X  = how far forward the lens is from the robot center (you know, x) (meters)
    //   CAMERA_Y  = how far over the lens is from the robot center (you know, y) (also meters)
    //   CAMERA_Z  = how high the lens is off the floor (meters)
    //   CAMERA_PITCH = upward tilt angle (positive = tilted up, degrees)
    // -----------------------------------------------------------------------
    private static final double CAMERA_X     = 0.344; // TODO: measure
    private static final double CAMERA_Y     = 0; // TODO: measure
    private static final double CAMERA_Z     = 0.2714625;
    
    private static final double CAMERA_ROLL = 0;
    private static final double CAMERA_YAW = 28.5;
    private static final double CAMERA_PITCH = 0;

    private static final Transform3d ROBOT_TO_RIGHT_CAM = new Transform3d(
        new Translation3d(CAMERA_X, -CAMERA_Y, CAMERA_Z),
        new Rotation3d(CAMERA_ROLL, Units.degreesToRadians(-CAMERA_PITCH), Units.degreesToRadians(-CAMERA_YAW))
    );
    private static final Transform3d ROBOT_TO_LEFT_CAM = new Transform3d(
        new Translation3d(CAMERA_X,  CAMERA_Y, CAMERA_Z),
        new Rotation3d(CAMERA_ROLL, CAMERA_PITCH, Units.degreesToRadians(CAMERA_YAW))
    );

    private final AprilTagFieldLayout fieldLayout;
    private final PhotonCamera rightCam;
    private final PhotonCamera leftCam;
    private final PhotonPoseEstimator rightEstimator;
    private final PhotonPoseEstimator leftEstimator;
    private final Field2d field = new Field2d();

    private List<VisionObservation> latestObservations = new ArrayList<>();

    public AprilTagSubsystem() {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2026RebuiltWelded.m_resourceFile);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        rightCam = new PhotonCamera("Right Cam");
        leftCam  = new PhotonCamera("Left Cam");

        rightEstimator = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_RIGHT_CAM);
        leftEstimator  = new PhotonPoseEstimator(fieldLayout, ROBOT_TO_LEFT_CAM);

        SmartDashboard.putData("Vision Field", field);
    }

    @Override
    public void periodic() {
        latestObservations.clear();

        processCamera(rightCam, rightEstimator);
        processCamera(leftCam,  leftEstimator);

        if (!latestObservations.isEmpty()) {
            Pose2d latestPose = latestObservations.get(latestObservations.size() - 1).pose();
            field.setRobotPose(latestPose);
            SmartDashboard.putNumber("Vision/Pose X", latestPose.getX());
            SmartDashboard.putNumber("Vision/Pose Y", latestPose.getY());
        }

        SmartDashboard.putData("Vision Field", field);
        SmartDashboard.putBoolean("Vision/Right Cam Connected", rightCam.isConnected());
        SmartDashboard.putBoolean("Vision/Left Cam Connected",  leftCam.isConnected());
        SmartDashboard.putNumber("Vision/Observations This Frame", latestObservations.size());
    }

    private void processCamera(PhotonCamera camera, PhotonPoseEstimator estimator) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            // Reject single-tag results that are too ambiguous
            if (result.getTargets().size() == 1
                    && result.getBestTarget().getPoseAmbiguity() > AMBIGUITY_THRESHOLD) {
                continue;
            }

            // Try multi-tag first (most accurate), fall back to lowest-ambiguity single-tag
            Optional<EstimatedRobotPose> estimated = estimator.estimateCoprocMultiTagPose(result);
            if (estimated.isEmpty()) {
                estimated = estimator.estimateLowestAmbiguityPose(result);
            }
            if (estimated.isEmpty()) continue;

            Pose2d robotPose = estimated.get().estimatedPose.toPose2d();

            // Discard poses that land outside the field boundary
            if (robotPose.getX() < -FIELD_BORDER_MARGIN
                    || robotPose.getX() > FIELD_LENGTH + FIELD_BORDER_MARGIN
                    || robotPose.getY() < -FIELD_BORDER_MARGIN
                    || robotPose.getY() > FIELD_WIDTH + FIELD_BORDER_MARGIN) {
                continue;
            }

            // Average distance to all visible tags
            int numTags = 0;
            double totalDist = 0.0;
            for (var target : result.getTargets()) {
                Optional<Pose3d> tagPose = fieldLayout.getTagPose(target.getFiducialId());
                if (tagPose.isPresent()) {
                    totalDist += tagPose.get().getTranslation()
                        .getDistance(estimated.get().estimatedPose.getTranslation());
                    numTags++;
                }
            }
            if (numTags == 0) continue;

            double avgDist = totalDist / numTags;

            // Scale std devs: more tags + closer = more trust
            double xyStdDev = XY_STD_DEV_COEFF * Math.pow(avgDist, 1.2) / Math.pow(numTags, 2.0);
            // Don't trust rotation from a single tag — set to infinity
            double thetaStdDev = numTags > 1
                ? THETA_STD_DEV_COEFF * Math.pow(avgDist, 1.2) / Math.pow(numTags, 2.0)
                : Double.POSITIVE_INFINITY;

            latestObservations.add(new VisionObservation(
                robotPose,
                estimated.get().timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)
            ));
        }
    }

    /** Returns all valid pose observations from both cameras this frame. */
    public List<VisionObservation> getObservations() {
        return latestObservations;
    }

    @Override
    public Pose2d getPose() {
        if (latestObservations.isEmpty()) return null;
        return latestObservations.get(latestObservations.size() - 1).pose();
    }

    @Override
    public void setPose(Pose2d newPose) {}
}
