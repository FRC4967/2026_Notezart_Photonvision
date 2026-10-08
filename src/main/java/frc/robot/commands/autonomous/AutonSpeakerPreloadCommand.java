package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutonSpeakerPreloadCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;

    /**
     * A different preload shot command designed to shoot the note without any automatic shooter/chassis aiming.
     * @param shooterSubsystem the basic subsytem to access
     * @param compliantWheelsSubsystem the basic subsystem to access
     */
    public AutonSpeakerPreloadCommand(ShooterSubsystem shooterSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;

        addRequirements(shooterSubsystem);
        addRequirements(compliantWheelsSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.startShooting();
    }

    @Override
    public void execute() {
        // shooterSubsystem.angleToSpeaker(ShooterConstants.MANUAL_SPEAKER_ANGLE);
        shooterSubsystem.setTargetAngularAngle(ShooterConstants.MANUAL_SPEAKER_ANGLE + 2);
        shooterSubsystem.angleToTargetAngle();

        /* sends the note out once the shooter is angled and ramped up */
        if(shooterSubsystem.isReadyToAutonShoot()){
            compliantWheelsSubsystem.startCompliantWheels();
        } 
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooting();
        compliantWheelsSubsystem.stopCompliantWheels();

        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        /* unfortunately the shooter shoots too fast for the line break to detect! */
        return !compliantWheelsSubsystem.isShooterLineBreakBroken();
    }
}
