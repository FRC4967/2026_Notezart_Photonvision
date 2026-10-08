package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// need to finish command in diffrent branch and after testing
public class AutoSpeakerShotCommand extends Command{
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;

    private Supplier<Boolean> shootSupplier;

    public AutoSpeakerShotCommand(ShooterSubsystem shooterSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem, SwerveSubsystem swerveSubsystem, Supplier<Boolean> shootButton) {
        this.shooterSubsystem = shooterSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.shootSupplier = shootButton;

        addRequirements(shooterSubsystem);
        addRequirements(compliantWheelsSubsystem);
    }

    @Override
    public void execute() {
        double distToSpeaker = swerveSubsystem.getDistanceFromBotToSpeaker();
        shooterSubsystem.setTargetAngleToSpeaker(distToSpeaker);
        shooterSubsystem.angleToTargetAngle();
        // shooterSubsystem.setShooterPower(.5 + (swerveSubsystem.getDistanceFromBotToSpeaker()-3 / 10));
        shooterSubsystem.setShooterPowerToDistance(distToSpeaker);
        if(shooterSubsystem.isReadyToSpeakerShoot()){
            compliantWheelsSubsystem.startCompliantWheels();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooting();
        compliantWheelsSubsystem.stopCompliantWheels();
    }

    @Override
    public boolean isFinished() {
        return !compliantWheelsSubsystem.isShooterLineBreakBroken();
    }

}
