package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShootMode;
import frc.robot.TemperatureControl;


public class ShooterSubsystem extends SubsystemBase {
    // #region -*-*-*-*- Constants -*-*-*-*- 
    /** percent -1 to 1 value */
    private final double defaultShooterSpeed = .6;
    private final double defaultForceShooterOutSpeed = 0.4;
    // keep speed positive the method that uses it changes it to negitive
    private final double defaultForceShooterInSpeed = 0.4;

    /** percent -1 to 1 value */
    private final double defaultTopAmpShooterSpeed = 0.32;
    private final double defaultBottomAmpShooterSpeed = 0.068;

    private final int SHOOTER_TOP_ID = 13;
    private final int SHOOTER_BOTTOM_ID = 12;
    private final int ANGULAR_MOTOR_ID = 14;

    private final double angularDegreesRange = 102.71;
    private final double angularTicksRange = 1175; /* 15627 */
    /** ticks needed to make 0 the bottom of the shooter */
    private final double angleTicksAdjustment = 342;
    /** top angle is 117.6. In degrees */
    private final double angularLowestAngle = 14.89; /* 14.89 */
    /** the height of the middle of the speaker's opening. In meters*/
    private final double SPEAKER_TARGET_Z = 2.05;
    /** the height of the shooter at the lowest angle. In meters*/
    private final double SHOOTER_BOTTOM_Z = 0.274; /* top is .485 */
    /** the range in change of height from the lowest angle to the relatively highest angle. */
    private final double GravityAdjustmentCoefficient = 0.211;
    /** In degrees */
    public static final double AGAINST_SUBWOOFER_ANGLE = 56;
    /** In degrees */
    private static final double AMP_INTAKE_ANGLE = 90;
    private final double SHOOTER_Z_ANGLE_RANGE = 72.2;
    /** In RPM */
    private final double MIN_SHOOTER_SPEED = 3500;
    private final double MAX_SHOOTER_SPEED = 6000;
    private final double SHOOTER_SPEED_AVERAGE = (MIN_SHOOTER_SPEED + MAX_SHOOTER_SPEED) / 2;
    private final double SHOOTER_SPEED_RANGE = MAX_SHOOTER_SPEED - MIN_SHOOTER_SPEED;

    private final double ARM_P = 4.5; /* 2.5 */
    private final double ARM_I = 0.0; /* .1 */
    private final double ARM_D = 0.1; /* .21 */
    private final double AUTO_ARM_P = 5.5; /* .95 */
    private final double AUTO_ARM_I = 0.0; /* .1 */
    private final double AUTO_ARM_D = 0.0; /* .21 */

    // 1700 with -230 to slow
    // 1900 with -230 to fast
    // 1850 with -245 to fast
    // 1750 with -245 just to slow
    // 1775 with -245 too slow
    private final double AVG_AMP_TOP_SHOOTER_SPEED = 1850;
    // 200
    private final double MIN_AMP_TOP_SHOOTER_SPEED = AVG_AMP_TOP_SHOOTER_SPEED - 175;
    private final double MAX_AMP_TOP_SHOOTER_SPEED = AVG_AMP_TOP_SHOOTER_SPEED + 175;

    private final double AVG_AMP_BOTTOM_SHOOTER_SPEED = -245;
    private final double MIN_AMP_BOTTOM_SHOOTER_SPEED = AVG_AMP_BOTTOM_SHOOTER_SPEED - 175;
    private final double MAX_AMP_BOTTOM_SHOOTER_SPEED = AVG_AMP_BOTTOM_SHOOTER_SPEED + 175;

    // 1940 too fast
    // 1640 too slow
    // 1890 too fast
    private final double AVG_PASSING_SHOOTER_SPEED = 1760;
    private final double MIN_PASSING_SHOOTER_SPEED = AVG_PASSING_SHOOTER_SPEED - 200;
    private final double MAX_PASSING_SHOOTER_SPEED = AVG_PASSING_SHOOTER_SPEED + 200;

    private final PIDController angularPIDController;
    
    private final double kS = 0.44481;
    private final double kG = 0.082815;
    private final double kV = 0.24106;
    private final double kA = 0.0082896;
    private final ArmFeedforward angularFeedForward = new ArmFeedforward(kS, kG, kV, kA);


    /** in degrees per second */
    private final double MAX_ANGULAR_VELOCITY = 40;
    /** in degrees per second */
    private final double MAX_ANGULAR_ACCELERATION = 15;
    /** the coefficient to divide the distance to the speaker by */
    private final double distToSpeakerCoefficient = 3;

    //#endregion

    private double outputVoltage;
    private double feedForward;
    private double feedBack;
    private SimpleMotorFeedforward shooterFeedForward = new SimpleMotorFeedforward(1, 0.25);
    private PIDController shooterAmpPID = new PIDController(0.000045, 0, 0);
    private PIDController shooterSpeakerPID = new PIDController(0.000075, 0, 0);
    private double topFeedBack = 0;
    private double bottomFeedBack = 0;
    private final double distTillGravity = 2.35;
    // smaller number goes higher
    // 2.35 to high
    // 3.0 to high on last note
    // 3.25 to low

    // 3.1 to high
    private final double distTillGravityAuto = 3.55;

    private final SparkMax shooterTop;
    private final SparkMax shooterBottom;
    private RelativeEncoder topShooterEncoder;
    private RelativeEncoder bottomShooterEncoder;
    private TalonSRX angularMotor;
    private double currentTargetAngle;
    private boolean defaultShootCondition;

    private ShootMode shootMode = ShootMode.Idle;
    
    /** Instantiates the Shooter subsystem */
    public ShooterSubsystem() {        
        shooterTop = new SparkMax(SHOOTER_TOP_ID, MotorType.kBrushless);
        shooterBottom = new SparkMax(SHOOTER_BOTTOM_ID, MotorType.kBrushless);
        topShooterEncoder = shooterTop.getEncoder();
        bottomShooterEncoder = shooterBottom.getEncoder();
        setShooterCoast();
        angularMotor = new TalonSRX(ANGULAR_MOTOR_ID);
        angularMotor.setNeutralMode(NeutralMode.Brake);
        angularMotor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        angularPIDController = new PIDController(ARM_P, ARM_I, ARM_D);

        /** do note remove */
        SmartDashboard.putNumber("Shooter Out Speed", defaultForceShooterOutSpeed);
        SmartDashboard.putNumber("Shooter In Speed", defaultForceShooterInSpeed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Angler Angle", getAngularDegrees());
        SmartDashboard.putNumber("Angler Ticks", getRawAngularTicks());
        // SmartDashboard.putNumber("Top feedback", topFeedBack);
        // SmartDashboard.putNumber("Bottom feedback", bottomFeedBack);
        SmartDashboard.putBoolean("is at angular target", isAtTarget());
        SmartDashboard.putBoolean("is at shooter speed", isAtAutonShootingSpeed());
        SmartDashboard.putNumber("Top Shooter velocity", topShooterEncoder.getVelocity());
        SmartDashboard.putNumber("Bottom Shooter Velocity", bottomShooterEncoder.getVelocity());
        // SmartDashboard.putNumber("current target angle", currentTargetAngle);
        // SmartDashboard.putNumber("Output Voltage", outputVoltage);
        // SmartDashboard.putNumber("Bus Voltage", angularMotor.getBusVoltage());
        // SmartDashboard.putNumber("Feed Forward", feedForward);
        // SmartDashboard.putNumber("Feed Back", feedBack);
        // SmartDashboard.putBoolean("forward limit switch", angularMotor.isFwdLimitSwitchClosed() == 1);
        // SmartDashboard.putBoolean("reverse limit switch", angularMotor.isRevLimitSwitchClosed() == 1);

        /* temp because if a branch with encoder set to relative deploys the "absolute" position is ruined */
        if (angularMotor.isFwdLimitSwitchClosed() == 1) {
            angularMotor.setSelectedSensorPosition(-angleTicksAdjustment);   
        }
     }

    //#region -*-*-*-*-*-*- Shooter Functions -*-*-*-*-*-*-

    /** sets the idle mode of the shooter wheels to brake. */
    public void setShooterBrake(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        shooterTop.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterBottom.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** sets the idle mode of the shooter wheels to coast. */
    public void setShooterCoast(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kCoast);
        shooterTop.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        shooterBottom.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** sets the shooter wheels to the speed to white line */
    public void shootToWhiteLine() {
        topFeedBack = topFeedBack + shooterSpeakerPID.calculate(topShooterEncoder.getVelocity(), MAX_SHOOTER_SPEED);
        bottomFeedBack = bottomFeedBack + shooterSpeakerPID.calculate(bottomShooterEncoder.getVelocity(), MAX_SHOOTER_SPEED);

        double topShooterVolts = MathUtil.clamp(topFeedBack, -shooterTop.getBusVoltage(), shooterTop.getBusVoltage());
        double bottomShooterVolts = MathUtil.clamp(bottomFeedBack, -shooterBottom.getBusVoltage(), shooterBottom.getBusVoltage());

        TemperatureControl.tempSetVoltage(shooterTop, topShooterVolts);
        TemperatureControl.tempSetVoltage(shooterBottom, bottomShooterVolts);
    }

    /**
     * sets the shooter motor speed to a power based on the given distance with a closed loop.
     * @param distToSpeaker the given shooter distance
     */
    public void setShooterPowerToDistance(double distToSpeaker) {
        double shooterSpeed = MIN_SHOOTER_SPEED + (SHOOTER_SPEED_RANGE * MathUtil.clamp(distToSpeaker / 6.32, 0, 1));

        topFeedBack = topFeedBack + shooterSpeakerPID.calculate(topShooterEncoder.getVelocity(), shooterSpeed);
        bottomFeedBack = bottomFeedBack + shooterSpeakerPID.calculate(bottomShooterEncoder.getVelocity(), shooterSpeed);

        double topShooterVolts = MathUtil.clamp(topFeedBack, -shooterTop.getBusVoltage(), shooterTop.getBusVoltage());
        double bottomShooterVolts = MathUtil.clamp(bottomFeedBack, -shooterBottom.getBusVoltage(), shooterBottom.getBusVoltage());

        TemperatureControl.tempSetVoltage(shooterTop, topShooterVolts);
        TemperatureControl.tempSetVoltage(shooterBottom, bottomShooterVolts);
    }

    /** sets the shooter motors to idling in order to save power. */
    public void idleShooterMotors(){
        topFeedBack = topFeedBack + shooterSpeakerPID.calculate(topShooterEncoder.getVelocity(), SHOOTER_SPEED_AVERAGE);
        bottomFeedBack = bottomFeedBack + shooterSpeakerPID.calculate(bottomShooterEncoder.getVelocity(), SHOOTER_SPEED_AVERAGE);

        double topShooterVolts = MathUtil.clamp(topFeedBack, -shooterTop.getBusVoltage(), shooterTop.getBusVoltage());
        double bottomShooterVolts = MathUtil.clamp(bottomFeedBack, -shooterBottom.getBusVoltage(), shooterBottom.getBusVoltage());

        TemperatureControl.tempSetVoltage(shooterTop, topShooterVolts);
        TemperatureControl.tempSetVoltage(shooterBottom, bottomShooterVolts);
    }

    /** starts the shooters to the default shooter speed */
    public void startShooting(){
        shooterTop.set(defaultShooterSpeed);
        shooterBottom.set(defaultShooterSpeed);
        topFeedBack = 17;
        bottomFeedBack = 17;
    }

    /** sets the speed of the shooter motors to zero. */
    public void stopShooting() {
        shooterTop.stopMotor();
        shooterBottom.stopMotor();
        topFeedBack = 0;
        bottomFeedBack = 0;
    }

    /**
     * sets the motor power of the shooter.
     * @param power the percent -1 to 1 power.
     */
    public void setShooterPower(double power) {
        TemperatureControl.tempSet(shooterTop, power);
        TemperatureControl.tempSet(shooterBottom, power);
    }

    public void forceShooterOut(){
        setShooterPower(SmartDashboard.getNumber("Shooter Out Speed", defaultForceShooterOutSpeed));
    }

    public void forceShooterIn(){
        // changes to negitive so I dont have to input a negitive sign on shuffle board
        setShooterPower(-1 * SmartDashboard.getNumber("Shooter In Speed", defaultForceShooterInSpeed));
    }

    public void setPassingPower(){
        topFeedBack = topFeedBack + shooterAmpPID.calculate(topShooterEncoder.getVelocity(), AVG_PASSING_SHOOTER_SPEED);
        bottomFeedBack = bottomFeedBack + shooterAmpPID.calculate(bottomShooterEncoder.getVelocity(), AVG_PASSING_SHOOTER_SPEED);

        double topShooterVolts = MathUtil.clamp(topFeedBack, -12, 12);
        double bottomShooterVolts = MathUtil.clamp(bottomFeedBack, -12, 12);

        TemperatureControl.tempSetVoltage(shooterTop, topShooterVolts);
        TemperatureControl.tempSetVoltage(shooterBottom, bottomShooterVolts);
    }

     /**
     * sets the motor power of the top and bottom shooter for the amp.
     */
    public void setAmpPower() {
        topFeedBack = topFeedBack + shooterAmpPID.calculate(topShooterEncoder.getVelocity(), AVG_AMP_TOP_SHOOTER_SPEED);
        bottomFeedBack = bottomFeedBack + shooterAmpPID.calculate(bottomShooterEncoder.getVelocity(), AVG_AMP_BOTTOM_SHOOTER_SPEED);

        double topShooterVolts = MathUtil.clamp(topFeedBack, -12, 12);
        double bottomShooterVolts = MathUtil.clamp(bottomFeedBack, -12, 12);

        TemperatureControl.tempSetVoltage(shooterTop, topShooterVolts);
        TemperatureControl.tempSetVoltage(shooterBottom, bottomShooterVolts);
    }

    public void setTopAmpPower() {
        topFeedBack = topFeedBack + shooterAmpPID.calculate(topShooterEncoder.getVelocity(), AVG_AMP_TOP_SHOOTER_SPEED);
        double topShooterVolts = MathUtil.clamp(topFeedBack, -12, 12);
        TemperatureControl.tempSetVoltage(shooterTop, topShooterVolts);

    }

    public void setBottomAmpPower() {
        bottomFeedBack = bottomFeedBack + shooterAmpPID.calculate(bottomShooterEncoder.getVelocity(), AVG_AMP_BOTTOM_SHOOTER_SPEED);
        double bottomShooterVolts = MathUtil.clamp(bottomFeedBack, -12, 12);
        TemperatureControl.tempSetVoltage(shooterBottom, bottomShooterVolts);

    }

    /**
     * sets the motor power of the top shooter.
     * @param power the percent -1 to 1 power.
     */
    public void setTopShooterPower(double power) {
        TemperatureControl.tempSet(shooterTop, power);
    }

    /**
     * sets the motor power of the bottom shooter.
     * @param power the percent -1 to 1 power.
     */
    public void setBottomShooterPower(double power) {
        TemperatureControl.tempSet(shooterBottom, power);
    }

    public double getAvgTopShooterRPM() {
        return AVG_AMP_TOP_SHOOTER_SPEED;
    }

     public double getAvgBottomShooterRPM() {
        return AVG_AMP_BOTTOM_SHOOTER_SPEED;
    }

    /**
     * 
     * @param RPM the given revs per minute to reach
     * @param encoderToCheck which shooter encoder to compare with.
     * @return whether the shooter has reached the given RPM to shoot properly.
     */
    private boolean isAtSpeed(double RPM, RelativeEncoder encoderToCheck){
        return encoderToCheck.getVelocity() > RPM;
    }

    private boolean isAtSpeed(double RPM) {
        return (topShooterEncoder.getVelocity() > RPM) && (bottomShooterEncoder.getVelocity() > RPM);
    }

    /**
     * @return whether the top shooter motor has reached the default minimum RPM
     */
    public boolean isAtSpeed(){
        return topShooterEncoder.getVelocity() > MIN_SHOOTER_SPEED;
    }

    /**
     * Highly reccomend letting the shooter speed up past this power
     * if shooting anywhere farther than at the base of the speaker
     * @return whether the shooter RPM has reached the minimum speed to hit speaker.
     */
    public boolean isAtSpeakerShootingSpeed() {
        return isAtSpeed(MIN_SHOOTER_SPEED, topShooterEncoder);
    }

    /**
     * @return whether the shooter RPM has reached the minimun speed to preload shoot.
     */
    public boolean isAtAutonShootingSpeed() {
        return isAtSpeed(2000, topShooterEncoder);
    }

    /**
     * 
     * @param minRPM the minimum given revs per minute to reach
     * @param maxRPM the maximum given revs per minute to reach
     * @param encoderToCheck which shooter encoder to compare with.
     * @return whether the shooter has reached the given RPM range to shoot properly.
     */
    private boolean isAtRangedSpeed(double minRPM, double maxRPM, RelativeEncoder encoderToCheck) {
        return encoderToCheck.getVelocity() > minRPM && encoderToCheck.getVelocity() < maxRPM;
    }

    /**
     * @return whether the bottom shooter motor has reached the default minimum RPM
     */
    public boolean isBottomAtAmpSpeed() {
       return isAtRangedSpeed(MIN_AMP_BOTTOM_SHOOTER_SPEED, MAX_AMP_BOTTOM_SHOOTER_SPEED, bottomShooterEncoder);
    }

    public boolean isTopAtAmpSpeed() {
        return isAtRangedSpeed(MIN_AMP_TOP_SHOOTER_SPEED, MAX_AMP_TOP_SHOOTER_SPEED, topShooterEncoder);     
    }

    /**
     * @return whether the top and bottom motors are in range for RPM
     */
    public Boolean isAtAmpSpeed(){
        return isTopAtAmpSpeed() && isBottomAtAmpSpeed();
    }

    public boolean isAtPassingSpeed(){
        return isAtRangedSpeed(MIN_PASSING_SHOOTER_SPEED, MAX_PASSING_SHOOTER_SPEED, topShooterEncoder) && isAtRangedSpeed(MIN_PASSING_SHOOTER_SPEED, MAX_PASSING_SHOOTER_SPEED, topShooterEncoder);
    }

    /**
     * calculates if the shooter is ready to shoot in the speaker. based on the target angle and speed (RPM).
     * @param extraCondition an extra condition for preparedness, typically a button.
     * @return if the robot is ready to shoot.
     */
    public boolean isReadyToSpeakerShoot() {
        return isAtTarget() && isAtSpeakerShootingSpeed();
    }

    public boolean isReadyToSpedShoot(double RPM, Boolean extraCondition) {
        return isAtTarget() && isAtSpeed(RPM) && extraCondition;
    }

    /**
     * calculates if the shooter is ready to shoot in auton. based on the target angle and speed (RPM).
     * @param extraCondition an extra condition to check for
     * @return if the robot is ready to shoot
     */
    public boolean isReadyToAutonShoot(boolean extraCondition) {
        System.out.println("target: " + isAtTarget());
        System.out.println(" speed: " + isAtAutonShootingSpeed());
        System.out.println(" angle: " + extraCondition);
        return isAtTarget() && isAtAutonShootingSpeed() && extraCondition;
    }

    /**
     * calculates if the shooter is readt to shoot in auton. based on target angle and speed (RPM)
     * @return if the robot is ready to shoot
     */
    public boolean isReadyToAutonShoot() {
        return isReadyToAutonShoot(true);
    }

    /**
     * calculates if the shooter is readt to shoot in the amp. based on target angle and speed (RPM)
     * @return if the robot is ready to shoot
     */
    public boolean isReadyToAmpShoot() {
        return isTopLimitSwitchPressed() && isAtAmpSpeed();
    }

    /**
     * changes the condition for ready to shoot for the shoot command
     * @param newCondition new shoot condition
     */
    public void setDefaultCondition(boolean newCondition) {
        defaultShootCondition = newCondition;
    }

    /**
     * @return the currently set shoot condition
     */
    public boolean isReadyToShoot() {
        return defaultShootCondition;
    }

    //#endregion

    //#region -*-*-*-*-*-*- Angular Functions -*-*-*-*-*-*-

    /** sets the PID of the angular motor to the Autonomous PID */
    public void setAutoAngularPID(){
        angularPIDController.setP(AUTO_ARM_P);
        angularPIDController.setI(AUTO_ARM_I);
        angularPIDController.setD(AUTO_ARM_D);
    }

    /** sets the PID of the angular motor to the teleOp PID */
    public void setDefaultAngularPID(){
        angularPIDController.setP(ARM_P);
        angularPIDController.setI(ARM_I);
        angularPIDController.setD(ARM_D);
    }

    /**
     * @return if the limit switch at the top of the angular is closed.
     */
    public boolean isTopLimitSwitchPressed(){
        return angularMotor.isRevLimitSwitchClosed() == 1;
    }

    /**
     * @return if the limit switch at the bottom of the angular is closed.
     */
    public boolean isBottomLimitSwitchPressed() {
        return angularMotor.isFwdLimitSwitchClosed() == 1;
    }

    /** resets the encoder on the angular motor. Affects positioning code. */
    public void resetAngularEncoder() {
        angularMotor.setSelectedSensorPosition(0);
    }

    /**
     * sets the encoder on the angular motor to the given value. Affects positioning code.
     * @param sensorPos the position to set IN TICKS
     */
    public void setAngularEncoder(double sensorPos) {
        angularMotor.setSelectedSensorPosition(sensorPos);
    }

    /** sets the angular encoder to the bottom angle */
    public void setAngluarToLowest(){
        angularMotor.setSelectedSensorPosition(-(angularLowestAngle * (angularTicksRange / angularDegreesRange)));
    }

    /**
     * @return the ticks reading off the angular encoder
     */
    public double getRawAngularTicks() {
        return angularMotor.getSelectedSensorPosition();
    }

    public double getRawAngularTicksAdjusted(){
        return getRawAngularTicks() + angleTicksAdjustment;
    }

    /**
     * note: is set to the absolute value
     * @return the ticks reading off the angular encoder 
     */
    public double getABSAngularTicks() {
        double ticks = Math.abs(getRawAngularTicksAdjusted());
        return ticks;
    }

    /**
     * @return the angular position of the angular in degrees.
     */
    public double getAngularDegrees() {
        double encoderDegrees = (getABSAngularTicks() * (angularDegreesRange / angularTicksRange)) + angularLowestAngle;
        return encoderDegrees;
    }

    /**
     * @return the angular position of the angular in radians.
     */
    public double getAngularRadians() {
        double encoderRadians = Math.toRadians(getAngularDegrees());
        return encoderRadians;
    }

    /**
     * Calculates the height of the inside surface shooter's bottom panel 
     * with a gravity factor added to arc the shot better.
     * Based on currently read angle of shooter.
     * DOES NOT WORK PAST PERPINDICULAR RANGE, but we should never be auto shooting from there.
     * @param distToSpeaker the distance from the speaker
     * @return the height of the shooter
     */
    public double getGravityAdjustmet(double distToSpeaker) {
        return (GravityAdjustmentCoefficient * (distToSpeaker / distToSpeakerCoefficient));
    }

    /**
     * calculates the height of the inside surface shooter's bottom panel.
     * DOES NOT WORK PAST PERPINDICULAR RANGE, but we should never be auto shooting from there.
     * @param distToSpeaker the distance from the speaker
     * @return the height of the shooter
     */
    public double getRealAutonShooterHeight(double distToSpeaker) {
        return SHOOTER_BOTTOM_Z * (1 - (distToSpeaker / distTillGravityAuto));
    }

    /**
     * calculates the height of the inside surface shooter's bottom panel.
     * DOES NOT WORK PAST PERPINDICULAR RANGE, but we should never be auto shooting from there.
     * @param shooterAngle the angle the angular is at
     * @return the height of the shooter
     */
    public double getRealTeleOpShooterHeight(double distToSpeaker) {
        return SHOOTER_BOTTOM_Z * (1 - (distToSpeaker / distTillGravity));
    }

    /**
     * sets the angular motor to a set speed.
     * @param controlMode the control type of the input
     * @param inputValue the dedicated input
     */
    public void setAngularMotor(ControlMode controlMode, double inputValue) {
        TemperatureControl.tempSet(angularMotor, controlMode, inputValue);
    }

    /**
     * sets the angular motor to a set voltage.
     * @param voltage the voltage output
     */
    public void setAngularVoltage(double voltage) {
        TemperatureControl.tempSet(angularMotor, ControlMode.PercentOutput, voltage/angularMotor.getMotorOutputVoltage());
    }
    
    /**
     * sets the angular angle to the last given target angle
     */
    public void angleToTargetAngle() {

        feedForward = angularFeedForward.calculate(
            Math.toRadians(currentTargetAngle), 
            Math.toRadians(MAX_ANGULAR_VELOCITY),
            Math.toRadians(MAX_ANGULAR_ACCELERATION
        ));

        feedBack = angularPIDController.calculate(currentTargetAngle, getAngularDegrees());

        // outputVoltage = MathUtil.clamp(feedForward + feedBack, -Constants.MAX_BUS_VOLTAGE, Constants.MAX_BUS_VOLTAGE);
        outputVoltage = MathUtil.clamp(feedBack, -Constants.MAX_BUS_VOLTAGE, Constants.MAX_BUS_VOLTAGE);

        setAngularMotor(ControlMode.PercentOutput, outputVoltage / angularMotor.getBusVoltage());
    }

    /**
     * sets the angular angle to shoot at the speaker based on the distance given.
     * @param distToSpeaker the inputted distance across the X-axis to the speaker from the robot.
     */
    public void setTargetAngleToSpeaker(double distToSpeaker){
        double speakerAngle = Math.toDegrees(Math.atan((SPEAKER_TARGET_Z - getRealTeleOpShooterHeight(distToSpeaker)) / distToSpeaker));
        setTargetAngularAngle(speakerAngle);
    }

    /**
     * sets the angular angle to shoot at the speaker based on the distance given.
     * @param distToSpeaker the inputted distance across the X-axis to the speaker from the robot.
     */
    public void autonAngleToSpeaker(double distToSpeaker){
        double speakerAngle = Math.toDegrees(Math.atan((SPEAKER_TARGET_Z - getRealAutonShooterHeight(distToSpeaker)) / distToSpeaker));
        setTargetAngularAngle(speakerAngle);
    }

    public double calculateAngleToSpeaker(double distToSpeaker) {
        return Math.toDegrees(Math.atan((SPEAKER_TARGET_Z) / distToSpeaker));
    }

    /**
     * @return the currently set target angle of the angular.
     */
    public double getTargetAngularAngle(){
        return currentTargetAngle;
    }

    /**
     * sets the currently set target angle of the angular.
     * @param angle the angler to set (in degrees)
     */
    public void setTargetAngularAngle(double angle) {
        currentTargetAngle = angle;
    }

    /**
     * @return if the angular angle has reached the last given target angle.
     */
    public boolean isAtTarget(){
        double differenceToAngle = Math.abs(getTargetAngularAngle() - getAngularDegrees());
        return (differenceToAngle <= 5);
    }

    // Sets angle for climbing
    public void toTopAngle() {
        setAngularMotor(ControlMode.PercentOutput, -1);
    }

    public void angleToIntake(){
        setTargetAngularAngle(AMP_INTAKE_ANGLE);
    }

    //Sets shooter speed for amp
    public void setShooterAmpSpeed() {
        shooterTop.set(defaultTopAmpShooterSpeed);
        shooterBottom.set(defaultBottomAmpShooterSpeed);
    }

    //Sets shooter power for when shooting in speaker
    public void setShooterSpeed() {
        setShooterPower(defaultShooterSpeed);
    }

    //sets angle to subwoofer angle
    public void setTargetAngleAgainstSubwoofer() {
        setTargetAngularAngle(AGAINST_SUBWOOFER_ANGLE);
    }

    //#endregion

    //#region -*-*-*-*-*-*- Modes Functions -*-*-*-*-*-*-*-

    public void setShootMode(ShootMode newShootMode) {
        shootMode = newShootMode;
    }

    public ShootMode getShootMode() {
        return shootMode;
    }

    public void runStatesSystem(boolean ifHasNote, double distanceToSpeaker) {
        switch (shootMode) {
            case Speaker:
                setShooterCoast();
                setDefaultCondition(isReadyToSpeakerShoot());
                if (ifHasNote) {
                    setShooterPowerToDistance(distanceToSpeaker);
                    setTargetAngleToSpeaker(distanceToSpeaker);
                    angleToTargetAngle();
                } else {
                    setTargetAngularAngle(getAngularDegrees());
                    
                    angleToTargetAngle();
                    idleShooterMotors();
                }
            break;
            case Amp:
                setDefaultCondition(isReadyToAmpShoot());
                setShooterBrake();
                if (getAvgTopShooterRPM() >= MAX_AMP_TOP_SHOOTER_SPEED) {
                    System.out.println("TOP TOO FASTTTT");
                    setTopShooterPower(0);
                } else {
                    setTopAmpPower();
                } 

                if (getAvgBottomShooterRPM() >= MAX_AMP_BOTTOM_SHOOTER_SPEED) {
                    System.out.println("BOTTOM TOO FAST");
                    setBottomShooterPower(0);
                } else {
                    setBottomAmpPower();
                }

                if (ifHasNote) {
                    setAngularMotor(ControlMode.PercentOutput, -1);
                } else {
                    setTargetAngularAngle(90);
                    angleToTargetAngle();
                }          
            break;
            case SubWoofer:
                setDefaultCondition(true);
                setShooterCoast();
                if (ifHasNote) {
                    startShooting();
                    setTargetAngleAgainstSubwoofer();
                    angleToTargetAngle();
                } else {
                    idleShooterMotors();
                    setTargetAngleAgainstSubwoofer();
                    angleToTargetAngle();
                }
            break;
            case Passing:
                setDefaultCondition(isAtPassingSpeed());
                setShooterCoast();
                /* TODO: get the best numbers for these */
                setPassingPower();
                setTargetAngularAngle(50); // 35 is old
                angleToTargetAngle();

            break;
            case Idle:
                setDefaultCondition(true);
                setShooterCoast();
                stopShooting();
                setTargetAngularAngle(getAngularDegrees());
                angleToTargetAngle();
            break;
        }
    }

    //#endregion

}