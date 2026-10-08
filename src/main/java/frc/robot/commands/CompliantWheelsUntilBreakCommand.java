package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CompliantWheelsSubsystem;

public class CompliantWheelsUntilBreakCommand extends Command{
    private CompliantWheelsSubsystem compliantWheelsSubsystem;
    private Supplier<Boolean> forceIn;
    private Supplier<Boolean> forceOut;

    public CompliantWheelsUntilBreakCommand(CompliantWheelsSubsystem compliantWheelsSubsystem, Supplier<Boolean> forceIn, Supplier<Boolean> forceOut){
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;
        this.forceIn = forceIn;
        this.forceOut = forceOut;

        addRequirements(compliantWheelsSubsystem);
    }

    @Override
    public void execute() {
        compliantWheelsSubsystem.startCompliantWheels();
    }

    @Override
    public void end(boolean interrupted) {
        compliantWheelsSubsystem.stopCompliantWheels();
    }

    @Override
    public boolean isFinished() {
        return compliantWheelsSubsystem.isShooterLineBreakBroken() || forceIn.get() || forceOut.get();
    }
}
