package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    //#region -*-*- Constants -*-*-

    // 3.884
    // 3.881
    // 3.877
    // 3.865
    // 3.825
    // 3.79

    // 3.76
    // 3.71
    private final double WHEEL_CIRCUMPHERENCE = (Units.inchesToMeters(3.7) * Math.PI);

    private final double DRIVE_GEAR_RATIO = 6.75;

    private double drivePosConversionFactor = WHEEL_CIRCUMPHERENCE / DRIVE_GEAR_RATIO;
    private double driveVelocityConversionFactor = drivePosConversionFactor  / 60;

    private final double maxMetersPerSec = 4;

    private final double directionPID_P = 0.5;
    private final double directionPID_I = 0;
    private final double directionPID_D = 0;
    //#endregion

    private final SparkMax driveMotor;
    private final RelativeEncoder driveEncoder;
    private final SparkMax directionMotor;
    private final CANcoder directionEncoder;
    private final PIDController directionPID = new PIDController(directionPID_P, directionPID_I, directionPID_D);
    private final PIDController drivePIDController = new PIDController(.5, 0, 0);
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.5, 1.2);

    /**
     * A custom-made class that instantiates each individual module of the swerve,
     * allowing control of each individual module as called seperately
     * @param driveMotorID the set ID of the SparkMax on the motor controlling the driving power of the module
     * @param directionMotorID the set ID of the SparkMax on the motor controlling direction of the module
     * @param CANcoderID the set ID of the CANCoder (Phoenix Tuner) dedicated to encoder control of the direction module
     * @param isDriveMotorReversed If the power of the drive motor's SparkMax is set to kInverted
     * @param isDirectionEncoderReversed if the power of the direction motor's SparkMax is set to kInverted
     */
    public SwerveModule(Integer driveMotorID, Integer directionMotorID, Integer CANcoderID,
    boolean isDriveMotorReversed, boolean isDirectionEncoderReversed, double driveEncoderErrorFactor){

        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        directionMotor = new SparkMax(directionMotorID, MotorType.kBrushless);

        drivePosConversionFactor *= (1 - driveEncoderErrorFactor);
        driveVelocityConversionFactor = drivePosConversionFactor / 60;

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.inverted(isDriveMotorReversed);
        driveConfig.smartCurrentLimit(40);
        driveConfig.encoder
            .positionConversionFactor(drivePosConversionFactor)
            .velocityConversionFactor(driveVelocityConversionFactor);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig directionConfig = new SparkMaxConfig();
        directionConfig.inverted(isDirectionEncoderReversed);
        directionConfig.smartCurrentLimit(40);
        directionMotor.configure(directionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        driveEncoder = driveMotor.getEncoder();
        directionEncoder = new CANcoder(CANcoderID);

        directionPID.enableContinuousInput(-Math.PI, Math.PI);
    }

    // #region -*-*-*-*-*-  gets -*-*-*-*-*-

    /**
     * @return the absolute value off of the swerve module's CANCoder.
     */
    public double getAbsoluteDirectionPos(){
        return directionEncoder.getAbsolutePosition().getValueAsDouble();
    }

    /**
     * @return the position of the swerve module in degrees at a 0-360 range.
     */
    public double getDirectionPosDeg(){
        // + 0.5 to get the position to 0-1 and * 360 to make number degrees
        return (directionEncoder.getAbsolutePosition().getValueAsDouble() + 0.5) * 360;
    }

    /**
     * @return the position of the swerve module in radians in a -PI to PI range.
     */
    public double getDirectionPosRad(){
        return Math.IEEEremainder(Math.toRadians(getDirectionPosDeg()), 2 * Math.PI);
    }

    /**
     * @return the aggregate gain of the module's drive encoder.
     */
    public double getDrivePos(){
        return -driveEncoder.getPosition();
    }

    /**
     * @return the current drive velocity of the module.
     */
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }

    /**
     * @return the relative field position of the module as a SwerveModulePosition.
     */
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(getDrivePos(), new Rotation2d(getDirectionPosRad()));
    }

    /**
     * @return the current "state" of the Swerve module in a SwerveModuleState.
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getDirectionPosRad()));
    }

    /**
     * @return The bus voltage.
     */
    public double getBusVolts() {
        return driveMotor.getBusVoltage();
    }

    // #endregion

    // #region -*-*-*-*-*-  sets  -*-*-*-*-*-

    /**
     * sets the drive power and turn angle according to the inputted desired state.
     * @param state the desired state to input
     */
    public void setAutonDesiredState(SwerveModuleState state){
        double busVoltage = driveMotor.getBusVoltage();

        Rotation2d currentHeading = Rotation2d.fromRadians(getDirectionPosRad());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, currentHeading);

        // Calculate the drive output from the drive PID controller.
        final double driveFeedbackV = drivePIDController.calculate(driveEncoder.getVelocity(), optimizedState.speedMetersPerSecond);
        final double driveFeedforwardV = driveFeedforward.calculate(optimizedState.speedMetersPerSecond);
        double driveOutputV = MathUtil.clamp(driveFeedbackV + driveFeedforwardV, -busVoltage, busVoltage);

        // Calculate the turning motor output from the turning PID controller.
        double turnOutput = directionPID.calculate(currentHeading.getRadians(), optimizedState.angle.getRadians());

        if (Math.abs(optimizedState.speedMetersPerSecond) <= (maxMetersPerSec * 0.001)) {
            driveOutputV = 0;
            turnOutput = 0;
        }

        driveMotor.setVoltage(-driveOutputV);
        directionMotor.set(MathUtil.clamp(turnOutput, -1, 1));

    }

    public void setTeleopDesiredState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getState().angle);
        double driveMotorPower = optimizedState.speedMetersPerSecond;
        if(Math.abs(driveMotorPower) < 0.01){
            stop();
            return;
        }
        double currentAngle = getDirectionPosRad();
        double targetAngle  = optimizedState.angle.getRadians();
        double directionMotorPower = directionPID.calculate(currentAngle, targetAngle);
        SmartDashboard.putNumber("Swerve/ID" + driveMotor.getDeviceId() + " Target (deg)", Math.toDegrees(targetAngle));
        TemperatureControl.tempSet(driveMotor, -driveMotorPower);
        TemperatureControl.tempSet(directionMotor, directionMotorPower);
    }

    // #endregion

    // #region -*-*-*-*-*- other *-*-*-*-*-*-

    /** sets all the drive encoders to zero. Does not affect the odometry. */
    public void resetDriveEncoder(){
        driveEncoder.setPosition(0);
    }

    /** sets the drive and direction motor speeds to zero. */
    public void stop(){
        driveMotor.stopMotor();
        directionMotor.stopMotor();
    }

    /** sets the idle mode of the drive and direction motors to brake. */
    public void setBrakeMode(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        directionMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** sets the idle mode of the drive and direction motors to coast. */
    public void setCoastMode(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        directionMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setAutonCurrentLimits() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(40);
        driveMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        directionMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setTeleopCurrentLimites() {
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.smartCurrentLimit(80);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig directionConfig = new SparkMaxConfig();
        directionConfig.smartCurrentLimit(20);
        directionMotor.configure(directionConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    //#endregion
}
