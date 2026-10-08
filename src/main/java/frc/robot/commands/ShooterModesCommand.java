package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.ShootMode;

public class ShooterModesCommand extends Command {
    private ShooterSubsystem shooterSubsystem;

    private Supplier<Double> distanceToSpeaker;
    private Supplier<Boolean> ifHasNote;

    private Supplier<Boolean> setSpeakerState;
    private Supplier<Boolean> setSubwooferState;
    private Supplier<Boolean> setAmpState;
    private Supplier<Boolean> setIdleState;
    private Supplier<Boolean> setPassingState;
 
    /**
     * The default shooter subsystem command.
     * uses state "modes" since Caleb's code is goofy lol.
     * this new refactor unironically makes me wanna kms
     * @param shooterSubsystem the default subsystem to supply
     * @param distanceToSpeakerSupplier the distance to shooter in meters, needed for auto shooting mode
     * @param ifHasNoteSupplier if the line break on the shooter is registering a note
     * @param setSpeakerSupplier sets the mode to auto aim towards the speaker based on distance
     * @param setSubwooferSupplier sets the mode to manually aim against the subwoofer
     * @param setAmpSupplier sets the mode to shoot into amp
     * @param setPassingSupplier sets the mode to pass notes from source to speaker
     * @param setIdleSupplier sets the mode to not run anything. Also useful for emergencies
     */
    public ShooterModesCommand(ShooterSubsystem shooterSubsystem, Supplier<Double> distanceToSpeakerSupplier, Supplier<Boolean> ifHasNoteSupplier,
    Supplier<Boolean> setSpeakerSupplier, Supplier<Boolean> setSubwooferSupplier, Supplier<Boolean> setAmpSupplier,
    Supplier<Boolean> setPassingSupplier, Supplier<Boolean> setIdleSupplier
    ) {
        this.shooterSubsystem = shooterSubsystem;
        this.distanceToSpeaker = distanceToSpeakerSupplier;
        this.ifHasNote = ifHasNoteSupplier;
        this.setSpeakerState = setSpeakerSupplier;
        this.setSubwooferState = setSubwooferSupplier;
        this.setAmpState = setAmpSupplier;
        this.setPassingState = setPassingSupplier;
        this.setIdleState = setIdleSupplier;

        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        /* mode setting buttons */
        if (setIdleState.get()) {
            System.out.println("set mode to idle");
            shooterSubsystem.setShootMode(ShootMode.Idle);
        } else if (setSpeakerState.get()) {
            System.out.println("set mode to speaker");
            shooterSubsystem.setShootMode(ShootMode.Speaker);
        } else if (setSubwooferState.get()) {
            System.out.println("set mode to subwoofer");
            shooterSubsystem.setShootMode(ShootMode.SubWoofer);
        } else if (setAmpState.get()) {
            System.out.println("set mode to amp");
            shooterSubsystem.setShootMode(ShootMode.Amp);
        } else if (setPassingState.get()) {
            System.out.println("set mode to passing");
            shooterSubsystem.setShootMode(ShootMode.Passing);
        }

        shooterSubsystem.runStatesSystem(ifHasNote.get(), distanceToSpeaker.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
