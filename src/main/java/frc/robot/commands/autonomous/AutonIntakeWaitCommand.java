package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonIntakeWaitCommand extends Command {
    SwerveSubsystem swerveSubsystem;

    Supplier<Boolean> ifHasNote;

    public AutonIntakeWaitCommand(SwerveSubsystem swerveSubsystem, Supplier<Boolean> ifHasNote) {
        this.swerveSubsystem = swerveSubsystem;
        this.ifHasNote = ifHasNote;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        swerveSubsystem.stopSystem();
    }

    @Override
    public boolean isFinished() {
        return ifHasNote.get();
    }
}
