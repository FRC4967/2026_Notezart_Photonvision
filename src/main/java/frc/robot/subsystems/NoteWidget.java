package frc.robot.subsystems;

import java.util.Arrays;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class NoteWidget implements Sendable {
    private double[] noteSequence;

    /**
     * instantiates the custom smartDashboard widget for selecting note routine
     * @param noteSequence the actual list of selected note order
     */
    public NoteWidget(double[] noteSequence) {
        this.noteSequence = Arrays.copyOf(noteSequence, noteSequence.length);
    }

    /** instantiates the custom smartDashboard widget for selecting note routine */
    public NoteWidget() {
        this(new double[8]);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("NoteRoute");
        builder.addDoubleArrayProperty("noteSequence", this::getNoteSequence, this::setNoteSequence);
    }

    /**
     * @return the list containing the selected note order's indexes.
     */
    public double[] getNoteSequence() {
        return noteSequence;
    }

    /**
     * sets the note order's index list.
     * @param newNoteSequence the list to set to
     */
    public void setNoteSequence(double[] newNoteSequence) {
        this.noteSequence = newNoteSequence;
    }

}