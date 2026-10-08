package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class DefaultIntakeCommand extends Command{
    private IntakeSubsystem intakeSubsystem;

    private Supplier<Boolean> forceOutwardSupplier;
    private Supplier<Boolean> forceInwardSupplier;
    
    public DefaultIntakeCommand(IntakeSubsystem intakeSubsystem, Supplier<Boolean> forceInward, Supplier<Boolean> forceOutward) {
        this.intakeSubsystem = intakeSubsystem;
        this.forceOutwardSupplier = forceOutward;
        this.forceInwardSupplier = forceInward;
        addRequirements(intakeSubsystem);
    }

    public void execute() {
        /* going OUT if you press both is INTENTIONAL */
        if (forceOutwardSupplier.get()) {
            intakeSubsystem.intakeSpinOutward();
        } else if (forceInwardSupplier.get()) {
            intakeSubsystem.intakeSpinInward();
        } else {
            intakeSubsystem.intakeStop();
        }
    }

    public boolean isFinished() {
        return false;
    }
}

