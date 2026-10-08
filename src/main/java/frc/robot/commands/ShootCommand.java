package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CompliantWheelsSubsystem;

public class ShootCommand extends Command {
    CompliantWheelsSubsystem compliantWheelsSubsystem;

    Supplier<Boolean> isReadyToShoot;

    public ShootCommand(Supplier<Boolean> isReadyToShootSupplier, CompliantWheelsSubsystem compliantWheelsSubsystem) {
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;
        this.isReadyToShoot = isReadyToShootSupplier;
        
        addRequirements(compliantWheelsSubsystem);
    }

    @Override
    public void execute() {
        if (isReadyToShoot.get()) {
            compliantWheelsSubsystem.startCompliantWheels();
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        compliantWheelsSubsystem.stopCompliantWheels();
    }

    @Override
    public boolean isFinished() {
        return !compliantWheelsSubsystem.isShooterLineBreakBroken();
    }
}
