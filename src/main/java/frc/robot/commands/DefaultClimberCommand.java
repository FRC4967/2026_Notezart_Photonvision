package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class DefaultClimberCommand extends Command {
    private ClimberSubsystem climberSubsystem;

    private Supplier<Boolean> climberUp;
    private Supplier<Boolean> climberDown;

    public DefaultClimberCommand(ClimberSubsystem climberSubsystem, Supplier<Boolean> climberUp, Supplier<Boolean> climberDown) {
        this.climberSubsystem = climberSubsystem;    
        this.climberUp = climberUp;
        this.climberDown = climberDown;

        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        if (climberUp.get()){
            climberSubsystem.climberUp();
        } else if (climberDown.get()){
            climberSubsystem.climberDown();
        } else {
            climberSubsystem.stopClimber();
        }
    }
}
