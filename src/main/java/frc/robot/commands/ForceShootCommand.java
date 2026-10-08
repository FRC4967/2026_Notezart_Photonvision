package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ForceShootCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;

    private Timer shotTimer;

    public ForceShootCommand(ShooterSubsystem shooterSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;
    }

    @Override
    public void initialize() {
        shotTimer = new Timer();

        shotTimer.reset();
    }

    @Override
    public void execute() {
        shotTimer.start();

        compliantWheelsSubsystem.startCompliantWheels();
    }

    @Override
    public boolean isFinished() {
        return shotTimer.hasElapsed(ShooterConstants.SHOOT_TIME);
    }

}
