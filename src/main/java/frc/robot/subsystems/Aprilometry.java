package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Aprilometry extends SubsystemBase implements BotPoseSource{
    private boolean isCorrectingOdometry;
    private SwerveSubsystem drivetrain;
    private AprilTagSubsystem aprilTags;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    // Initial odometry trust (x, y, theta) — std devs in meters/radians
    /** Standard deviation of odometry in meters */
    private static final double ODOMETRY_STD = 0.2;
    /** Standard deviation of odometry in radianss */
    private static final double ODOMETRY_ROT_STD = 0;
    /** Standard deviation of vision in meterss */
    private static final double VISION_STD = 0.1;
    /** Standard deviation of vision in radians */
    private static final double VISION_ROT_STD = Math.PI/9;

    public Aprilometry(AprilTagSubsystem aprilTags, SwerveSubsystem drivetrain) {
        this(aprilTags, drivetrain, false);
    }

    public Aprilometry(AprilTagSubsystem aprilTags, SwerveSubsystem drivetrain, boolean isCorrectingOdometry) {
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

        SmartDashboard.putData("Aprilometry field", field);
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
        poseEstimator.resetPosition(drivetrain.getGyroRotation(), drivetrain.getSwerveModulePositions(), newPose);
    }

    public void setOdometryCorrection(boolean isCorrectingOdometry) {
        poseEstimator.resetPosition(drivetrain.getGyroRotation(), drivetrain.getSwerveModulePositions(), drivetrain.getPose());
        this.isCorrectingOdometry = isCorrectingOdometry;
    }
}