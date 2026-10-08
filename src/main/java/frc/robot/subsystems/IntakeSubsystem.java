package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TemperatureControl;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkMax intakeMotor;
    private final DigitalInput intakeLineBreak;

    private final Integer INTAKE_MOTOR_ID = 9;
    private final Integer INTAKE_LINE_BREAK_ID = 1; /* might need to change this ID if Brayden changes it again smh */
    private final double IN_INTAKE_SPEED = 0.8;
    private final double OUT_INTAKE_SPEED = -0.8;

    private boolean intakePassed;

    /** instantiates the intake subsystem. */
    public IntakeSubsystem() {
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
        intakeLineBreak = new DigitalInput(INTAKE_LINE_BREAK_ID);
    }

    /** spins the intake inwards at the default speed. */
    public void intakeSpinInward() {
        TemperatureControl.tempSet(intakeMotor, IN_INTAKE_SPEED);
    }

    /** spins the intake outwards at the default speed. */
    public void intakeSpinOutward() {
        TemperatureControl.tempSet(intakeMotor, OUT_INTAKE_SPEED);
    }

    /** sets the intake motor speeds to zero. */
    public void intakeStop() {
        intakeMotor.stopMotor();
    }

    /**
     * True: has note False: has no note
     * @return if the line break is broken
     */
    public boolean isIntakeLineBreakBroken() {
        return !intakeLineBreak.get();
    }

    /**
     * @return if a note has recently passed through the line break
     */
    public boolean hasNotePassedLineBreak() {
        if(!intakeLineBreak.get()) {
            intakePassed = true;
            return false;
        } else if ((intakeLineBreak.get()) && intakePassed) {
            intakePassed = false;
            return true;
        } else {
            return false;
        }
    }
}
