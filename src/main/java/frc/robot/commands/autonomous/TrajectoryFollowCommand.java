package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.AngleUtil;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class TrajectoryFollowCommand extends SwerveControllerCommand {
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;

    /* NOTE: the swerveControllerCommand does not take the headings of the trajectory,
     * but instead just creates a spline between the start and end heading.
    */
    public TrajectoryFollowCommand(AutonSubsystem autonSubsystem, SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
    CompliantWheelsSubsystem compliantWheelsSubsystem, Trajectory trajectory, ProfiledPIDController thetaController, boolean goingToNote, Pose2d goToPos) {
        super(
        trajectory, 
        swerveSubsystem::getPose, 
        swerveSubsystem.getKinematics(), 
        new PIDController(1.25, 0, 0.3), 
        new PIDController(1.25, 0, 0.3), 
        thetaController, 
        () -> {
            // Pose2d endPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
            
            if(goingToNote){
                return new Rotation2d(Math.toRadians(AngleUtil.angleToPoint(swerveSubsystem.getPose(), goToPos)));
            } else {
                return new Rotation2d(-1 * swerveSubsystem.getAngleFromSpeaker());
                // return new Rotation2d(110);
            }
        },
        swerveSubsystem::setAutonModuleStates,
        swerveSubsystem
        );

        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;

        addRequirements(autonSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.intakeSpinInward();
        compliantWheelsSubsystem.startCompliantWheels();
        super.initialize();
    }

    @Override
    public void execute() {
        shooterSubsystem.autonAngleToSpeaker(swerveSubsystem.getDistanceFromBotToSpeaker());   
        shooterSubsystem.angleToTargetAngle();
        shooterSubsystem.shootToWhiteLine();
        if(compliantWheelsSubsystem.isShooterLineBreakBroken()){
            compliantWheelsSubsystem.stopCompliantWheels();
        }
        super.execute();
    }
}