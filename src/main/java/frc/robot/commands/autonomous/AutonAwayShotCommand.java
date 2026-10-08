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

public class AutonAwayShotCommand extends SwerveControllerCommand {
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;

    public AutonAwayShotCommand(AutonSubsystem autonSubsystem, SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem,
    CompliantWheelsSubsystem compliantWheelsSubsystem, Trajectory trajectory, ProfiledPIDController thetaController) {
        super(
        trajectory, 
        swerveSubsystem::getPose, 
        swerveSubsystem.getKinematics(), 
        new PIDController(2, 0, 0.1), 
        new PIDController(2, 0, 0.1), 
        thetaController, 
        () -> {
            Pose2d endPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
            
            if(!compliantWheelsSubsystem.isShooterLineBreakBroken()){
                return new Rotation2d(Math.toRadians(AngleUtil.angleToPoint(swerveSubsystem.getPose(), endPose)));
            } else {
                return new Rotation2d(-1 * swerveSubsystem.getAngleFromSpeaker());
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
        shooterSubsystem.startShooting();
        super.initialize();
    }

    @Override
    public void execute() {
        shooterSubsystem.angleToTargetAngle();     
        shooterSubsystem.autonAngleToSpeaker(swerveSubsystem.getDistanceFromBotToSpeaker()); 
        if (shooterSubsystem.isReadyToAutonShoot(compliantWheelsSubsystem.isShooterLineBreakBroken() && (Math.abs(swerveSubsystem.getPercentageToAngleToSpeaker()) <= .1))) {
            compliantWheelsSubsystem.startCompliantWheels();
        } else {
            compliantWheelsSubsystem.stopCompliantWheels();
        }
        super.execute();
    }
}
