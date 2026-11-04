package frc.robot.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class CombinedField extends SubsystemBase implements BotPoseSourceType{
    private boolean isCorrectingOdometry;
    private SwerveSubsystem drivetrain;
    private VisionField aprilTags;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    // Initial odometry trust (x, y, theta) — std devs in meters/radians
    /** Standard deviation of odometry in meters */
    private static final double ODOMETRY_STD = 0.2;
    /** Standard deviation of odometry in radianss */
    private static final double ODOMETRY_ROT_STD = 0;
    /** Standard deviation of vision in meterss */
    private static final double VISION_STD = 0.5;
    /** Standard deviation of vision in radians */
    private static final double VISION_ROT_STD = Math.PI/9;

    public CombinedField(VisionField aprilTags, SwerveSubsystem drivetrain) {
        this(aprilTags, drivetrain, false);
    }

    public CombinedField(VisionField aprilTags, SwerveSubsystem drivetrain, boolean isCorrectingOdometry) {
        super();
        this.aprilTags = aprilTags;
        this.drivetrain = drivetrain;
        this.isCorrectingOdometry = isCorrectingOdometry;
        poseEstimator = new SwerveDrivePoseEstimator(
            drivetrain.getKinematics(),
            drivetrain.getGyroRotation(),
            drivetrain.getSwerveModulePositions(),
            getPose(),
            VecBuilder.fill(ODOMETRY_STD, ODOMETRY_STD, ODOMETRY_ROT_STD),
            VecBuilder.fill(VISION_STD, VISION_STD, VISION_ROT_STD));

        SmartDashboard.putData("Combined field", field);
    }

    @Override
    public void periodic() {
        poseEstimator.update(drivetrain.getGyroRotation(), drivetrain.getSwerveModulePositions());

        poseEstimator.addVisionMeasurement(aprilTags.getPose(), 99999999, VecBuilder.fill(VISION_STD, VISION_STD, VISION_ROT_STD));

        drivetrain.setPose(poseEstimator.getEstimatedPosition());

        field.setRobotPose(poseEstimator.getEstimatedPosition());
        SmartDashboard.putNumber("Aprilometry/X", poseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Aprilometry/Y", poseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Aprilometry/Heading", poseEstimator.getEstimatedPosition().getRotation().getDegrees());
    }

    @Override
    public Pose2d getPose() {
        return field.getRobotPose();
    }

    @Override
    public void setPose(Pose2d newPose) {
        drivetrain.setPose(newPose);
        poseEstimator.resetPosition(drivetrain.getGyroRotationRaw(), drivetrain.getSwerveModulePositions(), new Pose2d(newPose.getX(), newPose.getY(), new Rotation2d(17))/* TODO EVERYTHING HINGES ON THIS NUMBER FOR SOME UNGODLY REASON AND ALSO TE GYRO IS READING 0 NOW */);
    }

    public void setOdometryCorrection(boolean isCorrectingOdometry) {
        poseEstimator.resetPosition(drivetrain.getGyroRotationRaw(), drivetrain.getSwerveModulePositions(), drivetrain.getPose());
        this.isCorrectingOdometry = isCorrectingOdometry;
    }
}