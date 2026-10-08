package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonIntakeCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;

    public AutonIntakeCommand(AutonSubsystem autonSubsystem, SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;

        addRequirements(autonSubsystem);
        addRequirements(swerveSubsystem);
        addRequirements(shooterSubsystem);

        addRequirements(intakeSubsystem);
        addRequirements(compliantWheelsSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.stopSystem();
        intakeSubsystem.intakeSpinInward();
        compliantWheelsSubsystem.startCompliantWheels();
    }

    @Override
    public void execute() {
        shooterSubsystem.autonAngleToSpeaker(swerveSubsystem.getDistanceFromBotToSpeaker());
        shooterSubsystem.angleToTargetAngle();
    }
  
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.intakeStop();
        compliantWheelsSubsystem.stopCompliantWheels();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return compliantWheelsSubsystem.isShooterLineBreakBroken();
    }
    
}
