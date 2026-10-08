package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TemperatureControl;

public class CompliantWheelsSubsystem extends SubsystemBase {
    private final SparkMax topWheels;
    private final SparkMax bottomWheels;
    private final DigitalInput shooterLineBreak;

    private final Integer TOP_WHEELS_ID = 11;
    private final Integer BOTTOM_WHEELS_ID = 10;
    /** attached at the digital input on the roboRIO. */
    private final Integer SHOOTER_LINE_BREAK_ID = 0;
    private final double compliantWheelSpeed = 0.6;
    private final double reverseCompliantWheelSpeed = -0.6;

    /** instantiates the compliant wheels subsystem. */
    public CompliantWheelsSubsystem() {
        topWheels = new SparkMax(TOP_WHEELS_ID, MotorType.kBrushless);
        bottomWheels = new SparkMax(BOTTOM_WHEELS_ID, MotorType.kBrushless);

        SparkMaxConfig topConfig = new SparkMaxConfig();
        topConfig.idleMode(IdleMode.kBrake);
        topWheels.configure(topConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig bottomConfig = new SparkMaxConfig();
        bottomConfig.idleMode(IdleMode.kBrake);
        bottomConfig.follow(topWheels);
        bottomWheels.configure(bottomConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        shooterLineBreak = new DigitalInput(SHOOTER_LINE_BREAK_ID);
        SmartDashboard.putBoolean("has note", isShooterLineBreakBroken());
    }

    /** spins the compliant wheels inwards to the default speed. */
    public void startCompliantWheels() {
        TemperatureControl.tempSet(topWheels, compliantWheelSpeed);
    }

    /** spins the compliant wheels outwards at the default speed. */
    public void reverseCompliantWheels() {
        TemperatureControl.tempSet(topWheels, reverseCompliantWheelSpeed);
    }

    /** sets the motor power of the compliant wheels to zero. */
    public void stopCompliantWheels() {
        topWheels.stopMotor();
    }

    /**
     * True: sees note False: does not see note
     * @return if the side line break is broken
     */
    public boolean isShooterLineBreakBroken() {
        return !shooterLineBreak.get();
    }
}
