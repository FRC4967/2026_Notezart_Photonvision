package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class AutoClimbCommand extends Command {
    ClimberSubsystem climberSubsystem;
    Supplier<Boolean> climbDownSupplier;

    public AutoClimbCommand(ClimberSubsystem climberSubsystem, Supplier<Boolean> climbDownButton) {
        this.climberSubsystem = climberSubsystem;
        this.climbDownSupplier = climbDownButton;

        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
        /* runs the climber up */
        climberSubsystem.setClimberSpeeds(1);
    }

    @Override
    public void execute() {
        if (climbDownSupplier.get()) {
            climberSubsystem.setClimberSpeeds(-1);
        }
    }
    
    @Override
    public boolean isFinished() {
        return (climberSubsystem.isLeftBottomSwitchPressed() && climberSubsystem.isRightBottomSwitchPressed());
    }
}
