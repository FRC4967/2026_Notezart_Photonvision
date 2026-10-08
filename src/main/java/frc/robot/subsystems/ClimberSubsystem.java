package frc.robot.subsystems;

import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TemperatureControl;

public class ClimberSubsystem extends SubsystemBase {
    private final int LEFT_CLIMBER_ID = 15;
    private final int RIGHT_CLIMBER_ID = 16;

    private final SparkMax leftClimber;
    private final SparkMax rightClimber;
    private final SparkLimitSwitch leftTopLimitSwitch;
    private final SparkLimitSwitch leftBottomLimitSwitch;
    private final SparkLimitSwitch rightTopLimitSwitch;
    private final SparkLimitSwitch rightBottomLimitSwitch;

    private final double defaultClimbPower = 0.75;
    /** instantiates the climber subsystem */

    public ClimberSubsystem() {
        leftClimber = new SparkMax(LEFT_CLIMBER_ID, MotorType.kBrushless);
        rightClimber = new SparkMax(RIGHT_CLIMBER_ID, MotorType.kBrushless);

        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig.idleMode(IdleMode.kBrake);
        leftConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchEnabled(true);
        leftClimber.configure(leftConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig.idleMode(IdleMode.kBrake);
        rightConfig.limitSwitch
            .forwardLimitSwitchType(Type.kNormallyClosed)
            .forwardLimitSwitchEnabled(true)
            .reverseLimitSwitchType(Type.kNormallyClosed)
            .reverseLimitSwitchEnabled(true);
        rightClimber.configure(rightConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        leftTopLimitSwitch = leftClimber.getForwardLimitSwitch();
        leftBottomLimitSwitch = leftClimber.getReverseLimitSwitch();
        rightTopLimitSwitch = rightClimber.getForwardLimitSwitch();
        rightBottomLimitSwitch = rightClimber.getReverseLimitSwitch();
    }

    /**
     * sets the left climber to the inputted speed
     *
     * @param percentPower the inputted percent speed from -1 to 1
     */
    public void setLeftClimberSpeed(double percentPower) {
        TemperatureControl.tempSet(leftClimber, percentPower);
    }

    /**
     * sets the right climber to the inputted speed
     *
     * @param percentPower the inputted percent speed from -1 to 1
     */
    public void setRightClimberSpeed(double percentPower) {
        TemperatureControl.tempSet(rightClimber, percentPower);
    }

    /**
     * sets the climbers to the inputted speed
     *
     * @param percentPower the inputted percent speed from -1 to 1
     */
    public void setClimberSpeeds(double percentPower) {
        TemperatureControl.tempSet(leftClimber, percentPower);
        TemperatureControl.tempSet(rightClimber, percentPower);
    }

    public void climberUp(){
        setClimberSpeeds(defaultClimbPower);
    }

    public void climberDown(){
        setClimberSpeeds(-defaultClimbPower);
    }

    public void stopClimber(){
        leftClimber.stopMotor();
        rightClimber.stopMotor();
    }

    /**
     * @return if the left bottom limit switch is pressed
     */
    public boolean isLeftBottomSwitchPressed() {
        return leftBottomLimitSwitch.isPressed();
    }

    /**
     * @return if the left top limit switch is pressed
     */
    public boolean isLeftTopSwitchPressed() {
        return leftTopLimitSwitch.isPressed();
    }

    /**
     * @return if the right bottom limit switch is pressed
     */
    public boolean isRightBottomSwitchPressed() {
        return rightBottomLimitSwitch.isPressed();
    }

    /**
     * @return if the right top limit switch is pressed
     */
    public boolean isRightTopSwitchPressed() {
        return rightTopLimitSwitch.isPressed();
    }
}
