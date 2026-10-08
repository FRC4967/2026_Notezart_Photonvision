package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonToppyCommand extends Command {
    AutonSubsystem autonSubsystem;
    SwerveSubsystem swerveSubsystem;
    ShooterSubsystem shooterSubsystem;
    IntakeSubsystem intakeSubsystem;
    CompliantWheelsSubsystem compliantWheelsSubsystem;

    private boolean iHateIndex = true;

    public AutonToppyCommand(AutonSubsystem autonSubsystem, SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem) {
        this.autonSubsystem = autonSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;

        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
        addRequirements(compliantWheelsSubsystem);
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterPowerToDistance(swerveSubsystem.getDistanceFromBotToSpeaker());
        if (!compliantWheelsSubsystem.isShooterLineBreakBroken()) {
            iHateIndex = true;
            compliantWheelsSubsystem.startCompliantWheels();
            intakeSubsystem.intakeSpinInward();
        } else if (shooterSubsystem.isReadyToAutonShoot(extraAutonCondition())) {
            if (iHateIndex) {
                autonSubsystem.updateNotesShotCount();
                iHateIndex = false;
            }
            compliantWheelsSubsystem.startCompliantWheels();
            intakeSubsystem.intakeStop();
        } else {
            iHateIndex = true;
            compliantWheelsSubsystem.stopCompliantWheels();
            intakeSubsystem.intakeStop();
        }

        shooterSubsystem.autonAngleToSpeaker(swerveSubsystem.getDistanceFromBotToSpeaker());
        shooterSubsystem.angleToTargetAngle();
    }

    private boolean extraAutonCondition() {
        boolean isAtAngle = (Math.abs((-swerveSubsystem.getPercentageToAngleToSpeaker()) * swerveSubsystem.getMaxAngularSpeed()) <= .1);
        // boolean isAtAngle = (Math.abs(swerve/Subsystem.getAngleDiferrenceToSpeaker()) <= 1.2);
        boolean isInDistance = swerveSubsystem.isInAutonShootingRange();
        boolean isIntersected = swerveSubsystem.isLineToStageIntersected();

        return isAtAngle && isInDistance && (!isIntersected);
    }
}
