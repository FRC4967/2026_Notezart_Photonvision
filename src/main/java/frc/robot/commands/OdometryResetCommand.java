package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.vision.CombinedField;

public class OdometryResetCommand extends Command{
    private final CombinedField aprilometry;
    private SwerveSubsystem swerveSubsystem;
    private Pose2d updatePose;

    /** resets the odometry to against the speaker in case limelight breaks and odometry is off */
    public OdometryResetCommand(CombinedField aprilometry, SwerveSubsystem swerveSubsystem){
        this(aprilometry, swerveSubsystem, new Pose2d());
    }

    public OdometryResetCommand(CombinedField aprilometry, SwerveSubsystem swerveSubsystem, Pose2d updatePose){
        this.aprilometry = aprilometry;
        this.updatePose = updatePose;
        this.swerveSubsystem = swerveSubsystem;
      

        addRequirements(aprilometry);
        addRequirements(swerveSubsystem);
    }

    // @Override
    // public void initialize() {

    // }

    @Override
    public void execute() {
        aprilometry.setPose(updatePose);

        swerveSubsystem.resetEncoders();
    }

    @Override
    public boolean isFinished() {
        return true;
    }   
}
