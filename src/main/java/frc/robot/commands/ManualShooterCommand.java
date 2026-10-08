package frc.robot.commands;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShooterCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    private double defaultTargetAngle;
    private Supplier<Boolean> shooterIn;
    private Supplier<Boolean> shooterOut;
    private Supplier<Boolean> angularUpSupplier;
    private Supplier<Boolean> angularDownSupplier;


    public ManualShooterCommand(ShooterSubsystem shooterSubsystem, Supplier<Boolean> shooterOut, 
    Supplier<Boolean> shooterIn, Supplier<Boolean> angularUpButton, Supplier<Boolean> angularDownButton) {
        this.shooterSubsystem = shooterSubsystem;
        this.shooterOut = shooterOut;
        this.shooterIn = shooterIn;
        this.angularUpSupplier = angularUpButton;
        this.angularDownSupplier = angularDownButton;

        addRequirements(shooterSubsystem);
    }

    public void execute() {
        if (shooterOut.get()) {
            shooterSubsystem.forceShooterOut();
        } else if (shooterIn.get()){
            shooterSubsystem.forceShooterIn();
        } else {
            shooterSubsystem.stopShooting();
        }

        if(angularDownSupplier.get()) {
            defaultTargetAngle = defaultTargetAngle - 2;
        } else if (angularUpSupplier.get()) {
            defaultTargetAngle = defaultTargetAngle + 2;
        } else {
            defaultTargetAngle = shooterSubsystem.getAngularDegrees();
        }

        shooterSubsystem.setTargetAngularAngle(defaultTargetAngle);
        shooterSubsystem.angleToTargetAngle();
    }

    public boolean isFinished() {
        return false;
    }

}