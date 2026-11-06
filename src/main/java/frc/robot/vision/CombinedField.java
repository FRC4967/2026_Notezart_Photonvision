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
    private VisionField visionField;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    // Initial odometry trust (x, y, theta) — std devs in meters/radians
    /** Standard deviation of odometry in meters */
    private static final double ODOMETRY_MODIFICATION_RATE = 0.5;

    private static final double ODOMETRY_STD = 0.2;
    /** Standard deviation of odometry in radianss */
    private static final double ODOMETRY_ROT_STD = 0;
    /** Standard deviation of vision in meterss */
    private static final double VISION_STD = 0;
    /** Standard deviation of vision in radians */
    private static final double VISION_ROT_STD = Math.PI/9;

    public CombinedField(VisionField aprilTags, SwerveSubsystem drivetrain) {
        this(aprilTags, drivetrain, false);
    }

    public CombinedField(VisionField aprilTags, SwerveSubsystem drivetrain, boolean isCorrectingOdometry) {
        super();
        this.visionField = aprilTags;
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

        poseEstimator.addVisionMeasurement(visionField.getPose(), 99999999, VecBuilder.fill(VISION_STD, VISION_STD, VISION_ROT_STD));

        double driveVisionDiscontinuity = Math.hypot(
            Math.pow(poseEstimator.getEstimatedPosition().getX()-drivetrain.getPose().getX(), 2),
            Math.pow(poseEstimator.getEstimatedPosition().getY()-drivetrain.getPose().getY(), 2));

        double driveVisionAdjustment = 0;

        if (driveVisionDiscontinuity<2.5) {
            driveVisionAdjustment = driveVisionDiscontinuity*(1/(1.63*Math.pow(driveVisionDiscontinuity, 1/2))*
                (Math.pow(driveVisionDiscontinuity-0.4,1/3)+0.75)/2);
        } else {
            driveVisionAdjustment = 1;
        }

        drivetrain.setPose(new Pose2d(
            poseEstimator.getEstimatedPosition().getX()*driveVisionAdjustment+drivetrain.getPose().getX()*(1-driveVisionAdjustment),
            poseEstimator.getEstimatedPosition().getY()*driveVisionAdjustment+drivetrain.getPose().getY()*(1-driveVisionAdjustment), 
            drivetrain.getGyroRotation()));

        field.setRobotPose(drivetrain.getPose());
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
        poseEstimator.resetPosition(new Rotation2d(Math.toRadians(drivetrain.getGyro())), drivetrain.getSwerveModulePositions(), new Pose2d(newPose.getX(), newPose.getY(), new Rotation2d(Math.toRadians(drivetrain.getGyro())))/* TODO EVERYTHING HINGES ON THIS NUMBER FOR SOME UNGODLY REASON AND ALSO TE GYRO IS READING 0 NOW */);
    }

    public void setOdometryCorrection(boolean isCorrectingOdometry) {
        poseEstimator.resetPosition(new Rotation2d(Math.toRadians(drivetrain.getGyro())), drivetrain.getSwerveModulePositions(), new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), new Rotation2d(Math.toRadians(drivetrain.getGyro()))));
        this.isCorrectingOdometry = isCorrectingOdometry;
    }
}