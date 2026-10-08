package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LightSubsystem;

public class DefaultLightCommand extends Command {

    private final LightSubsystem lightSubsystem;
    private final CompliantWheelsSubsystem compliantWheelsSubsystem;
    private final IntakeSubsystem intakeSubsystem;

    private Supplier<Boolean> shootSupplier;
    private Supplier<Boolean> noSourceSupplier;

    public DefaultLightCommand(LightSubsystem lightSubsystem, IntakeSubsystem intakeSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem, Supplier<Boolean> shootSupplier, Supplier<Boolean> noSourceSupplier) {
        this.lightSubsystem = lightSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shootSupplier = shootSupplier;
        this.noSourceSupplier = noSourceSupplier;

        // Only control lightSubsystem; other susbsystems are read-only
        addRequirements(lightSubsystem);
    }

    @Override
    public void execute() {
        if (shootSupplier.get()) {
            lightSubsystem.setLEDColor(Color.kGreen);
        } else if (compliantWheelsSubsystem.isShooterLineBreakBroken()) {
            lightSubsystem.setLEDColor(Color.kPurple);
        } else if (intakeSubsystem.isIntakeLineBreakBroken()){
            lightSubsystem.setLEDColor(Color.kOrange);
        } else {
            lightSubsystem.setAllianceColor();
        }

        if(noSourceSupplier.get()){
            lightSubsystem.blinkLimeLight();
        } else {
            lightSubsystem.stopLimeLight();
        }
    }

    
}
