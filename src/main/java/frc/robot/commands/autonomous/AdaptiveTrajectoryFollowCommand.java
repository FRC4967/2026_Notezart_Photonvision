package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.AngleUtil;
import frc.robot.subsystems.AutonSubsystem;
// import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class AdaptiveTrajectoryFollowCommand extends SwerveControllerCommand {
    public AdaptiveTrajectoryFollowCommand(AutonSubsystem autonSubsystem, SwerveSubsystem swerveSubsystem,/* PhotonVision photonVision,*/
    Supplier<Boolean> ifHasNote, Trajectory trajectory, ProfiledPIDController thetaController, Pose2d goToPose) {
        super(
        trajectory, 
        swerveSubsystem::getPose, 
        swerveSubsystem.getKinematics(), 
        new PIDController(1, 0, 0), 
        new PIDController(1, 0, 0), 
        thetaController, 
        () -> {
            // Pose2d endPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;


            // if (photonVision.ifhasTargets() && (!ifHasNote.get())) {
            // // if (false) {
            //     System.out.println("AMBATTTAAKUUUUUUUUUUUUUUUUUUUHM");
            //     return new Rotation2d(Math.toRadians(swerveSubsystem.getGyro() + (photonVision.getYaw() / 50)));
            // } else 
            if (!ifHasNote.get()){
                System.out.println("I DONT HAVE A NOTE!!!!!!!!!!");
                // if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                //     System.out.println("REDDDDDDDDD 40");
                    return new Rotation2d(Math.toRadians(AngleUtil.angleToPoint(swerveSubsystem.getPose(), goToPose)));
                // } else {
                    // return endPose.getRotation();
                // }
            }else {
                System.out.println("aiming to shoot");
                return new Rotation2d(-1 * swerveSubsystem.getAngleFromSpeaker());
            }
        },
        swerveSubsystem::setAutonModuleStates,
        swerveSubsystem
        );


        addRequirements(swerveSubsystem);
    }
}