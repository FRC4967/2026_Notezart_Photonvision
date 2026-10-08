package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.apache.commons.geometry.euclidean.twod.Line;
import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Segment;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.shape.Circle;
import org.apache.commons.numbers.core.Precision;

import com.studica.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AngleUtil;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class SwerveSubsystem extends SubsystemBase implements BotPoseSource{
   //#region -*-*- Constants -*-*-

    /** in meters */
    private final double xVal = (0.625 / 2);
    /** in meters */
    private final double yVal = (0.625 / 2);
    /** in meters per second */
    private final double maxSpeed = 4.2;
    /** in radians per second <p><tt> &omega; = v / r </tt></p>*/
    private double maxAngularSpeed = Math.PI / 4;
    // private final double maxAngularSpeed = maxSpeed;

    /** The units for the Translation2d are in meters, and positive x values represent 
    * distance from the center to the front of the bot.
    * Positive y values represent the distance from the center to the left of the bot.
    */
    private final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
        new Translation2d(xVal, yVal),
        new Translation2d(xVal, -yVal),
        new Translation2d(-xVal, yVal),
        new Translation2d(-xVal, -yVal)
    );

    private final double autonAimingPID_P = 7;
    private final double teleopAimingPID_P = 20;
    private final double aimingPID_I = 0.1;
    private final double aimingPID_D = 0.5;
    private final PIDController autonAimAssistPIDController = new PIDController(autonAimingPID_P, aimingPID_I, aimingPID_D);
    private final PIDController teleopAimAssistPIDController = new PIDController(teleopAimingPID_P, aimingPID_I, aimingPID_D);
    
    /** a cutoff value for decimal precision while calculating speaker obstacles. */
    private Vector2D botVector;
    private Segment stageLineSegment;
    private Vector2D speakerVector;
    private final Precision.DoubleEquivalence lineToSpeakerObstaclePrecision = Precision.doubleEquivalenceOfEpsilon(1e-2);

    //      *                                           FL     FR     BL     BR
    private final int[] driveMotorIDs =                {1,     3,     5,     7};
    private final int[] directionMotorIDs =            {2,     4,     6,     8};
    private final int[] absoluteEncoderIDs =           {0,     1,     2,     3};
    private final boolean[] isDriveMotorReversed =     {false,  false, false,  false};
    private final boolean[] isDirectionEncoderReversed = {false,  false,  false,  false};
    private final double[] driveEncoderErrorFactor = /*{ 0.03551178071, 0.05450016451, 0.04942700075, 0.02985604242 };*/ { 0.0, 0.0, 0.0, 0.0 };
    
    //#endregion

    public enum SwerveModules {
        FRONT_LEFT,
        FRON_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }

    SwerveModule[] swerveModules = new SwerveModule[4];

    private AHRS gyro;
    private double gyroAllianceChange;
    private SwerveDriveOdometry odometry;
    private Field2d field;
    private double lineX;

    Optional<Alliance> alliance = DriverStation.getAlliance();
    private Boolean redAlliance;

    /* testing values */
    double lastGyroUpdate;
    double lastGyroChange = 0;
    double lastSmartGyroChange = 0;

    Timer speedPerSecondTimer;

    public SwerveSubsystem() {

        speedPerSecondTimer = new Timer();

        alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
           speakerVector = Vector2D.of(Constants.obstaclePositions.get(1).getX(),
                    Constants.obstaclePositions.get(1).getY()); 
            redAlliance = true;
            gyroAllianceChange = 180;
            lineX = 10.7;
        } else {
            speakerVector = Vector2D.of(Constants.obstaclePositions.get(0).getX(),
                    Constants.obstaclePositions.get(0).getY());
            redAlliance = false;
            gyroAllianceChange = 0;
            lineX = 5.55;
        }

        for (int i = 0; i < 4; i++) {
            swerveModules[i] = new SwerveModule(
                driveMotorIDs[i],
                directionMotorIDs[i],
                absoluteEncoderIDs[i],
                isDriveMotorReversed[i],
                isDirectionEncoderReversed[i],
                driveEncoderErrorFactor[i]
            );

            swerveModules[i].setBrakeMode();
        }

        gyro = new AHRS(AHRS.NavXComType.kUSB1);
        resetGyro();
        lastGyroUpdate = gyro.getAngle();

        odometry = new SwerveDriveOdometry(
            DRIVE_KINEMATICS,
            getGyroRotation(),
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            }
        );

        field = new Field2d();
        field.setRobotPose(odometry.getPoseMeters());

    }
 
    @Override
    public void periodic() {

        odometry.update(
            getGyroRotation(),
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            }
        );

        field.setRobotPose(odometry.getPoseMeters());

        alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
           speakerVector = Vector2D.of(Constants.obstaclePositions.get(1).getX(),
                    Constants.obstaclePositions.get(1).getY()); 
            redAlliance = true;
            gyroAllianceChange = 180;
            lineX = 10.7;
        } else {
            speakerVector = Vector2D.of(Constants.obstaclePositions.get(0).getX(),
                    Constants.obstaclePositions.get(0).getY());
            redAlliance = false;
            gyroAllianceChange = 0;
            lineX = 5.55;
        }

        botVector = Vector2D.of(odometry.getPoseMeters().getX(), odometry.getPoseMeters().getY());

        //#region -*-*-*- Doucumentation on smart dashboard for testing -*-*-*-
        String[] moduleNames = {"FL", "FR", "BL", "BR"};
        for (int i = 0; i < 4; i++) {
            SmartDashboard.putNumber("Swerve/" + moduleNames[i] + " Angle (deg)", swerveModules[i].getDirectionPosDeg());
        }
        // SmartDashboard.putString("robot location", odometry.getPoseMeters().getTranslation().toString());
        SmartDashboard.putNumber("Power to speaker", getPercentageToAngleToSpeaker() * getMaxAngularSpeed());
        SmartDashboard.putBoolean("is in auton shooting range", isInAutonShootingRange());
        SmartDashboard.putBoolean("is shot not intersected", !isLineToStageIntersected());
        // SmartDashboard.putNumber("test enum", getAngleDiferrenceToSpeaker());
        // SmartDashboard.putData("Field", field);
        // SmartDashboard.putNumber("angle", getGyro());
        // SmartDashboard.putNumber("2  Odometry X", odometry.getPoseMeters().getX());
        // SmartDashboard.putNumber("2  Odometry Y", odometry.getPoseMeters().getY());
        // SmartDashboard.putNumber("speaker x", speakerVector.getX());
        // SmartDashboard.putNumber("speaker y", speakerVector.getY());
        SmartDashboard.putNumber("gyro angle", getGyro());
        SmartDashboard.putData("Odometry Field", field);
        // SmartDashboard.putNumber("angle from speaker", Math.toDegrees(getAngleFromSpeaker()));
        // SmartDashboard.putNumber("distance to speaker", getDistanceFromBotToSpeaker());
        // SmartDashboard.putNumber("FrontLeftPos", swerveModules[0].getDrivePos());
        // SmartDashboard.putNumber("FrontRightPos", swerveModules[1].getDrivePos());
        // SmartDashboard.putNumber("BackLeftPos", swerveModules[2].getDrivePos());
        // SmartDashboard.putNumber("BackRightPos", swerveModules[3].getDrivePos());

        // updateMaxSpeed();
       // SmartDashboard.putNumber("raw doggin", gyro.getAngle());
        //testingUpdates();

        //#endregion
    }

    /** only use when testing the config for precision. */
    private void testingUpdates() {
        speedPerSecondTimer.start();
        double newGyroUpdate = gyro.getAngle();
        double newGyroChange = newGyroUpdate - lastGyroUpdate;

        if (newGyroChange > lastGyroChange) {
            lastGyroChange = newGyroChange;
        }

        SmartDashboard.putNumber("max degrees per second: ", lastGyroChange);
        if (speedPerSecondTimer.hasElapsed(1)) {
            lastGyroUpdate = gyro.getAngle();
            speedPerSecondTimer.reset();
        }

        if (SmartDashboard.getNumber("selected max deg per second", 0) != lastSmartGyroChange) {
            lastSmartGyroChange = SmartDashboard.getNumber("selected max deg per second", 0);
            lastGyroChange = 0;
            gyro.reset();
            lastGyroUpdate = gyro.getAngle();
        }
    }

    // #region -*-*-*-*-*- gets -*-*-*-*-*-

    /**
     * @return the gyro angle according to alliance in degrees in a -180 to 180 range.
     */
    public double getGyro() {
        return Math.IEEEremainder(gyro.getAngle() + gyroAllianceChange, 360);
    }

    /**
     * @return the gyro angle raw in degrees in a -180 to 180 range.
     */
    public double getGyroRaw() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /**
     * @return the gyro anfle in radians from 0 to 2 PI.
     */
    public double getGyroRad() {
        return Math.toRadians(getGyro());
    }

    /** 
     * @return returns the gyro angle as a Rotation2d in counter clockwise direction.
     */
    public Rotation2d getGyroRotation() {
        // gets a rotation2D translation of gyro angle
        return Rotation2d.fromDegrees(-getGyro());
    }

    /** 
     * @return returns the gyro angle raw as a Rotation2d.
     */
    public Rotation2d getGyroRotationRaw() {
        // gets a rotation2D translation of gyro angle
        return Rotation2d.fromDegrees(-getGyroRaw());
    }

    /**
     * @return The instantiated drive kinematics.
     */
    public SwerveDriveKinematics getKinematics() {
        return DRIVE_KINEMATICS;
    }

    /**
     * @return The maximum velocity in meters per second.
     */
    public double getMaxSpeed() {
        return maxSpeed;
    }

     /**
     * @return The maximum angular velocity in radians per second.
     */
    public double getMaxAngularSpeed() {
        return maxAngularSpeed;
    }
        
    /**
     * @return The last updated swerve module relative position on the field
     */
    public SwerveModulePosition[] getSwerveModulePositions(){
        return new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            };
    }

    @Override
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * calculates the distance to the speaker across the field's X-axis in meters based on the position given.
     * @param botPose the position to calculate from
     * @return the distance from the speaker in meters.
     */
    private double getSpeakerDistance(Pose2d botPose) {
        return botPose.getTranslation().getDistance(new Translation2d(speakerVector.getX(), speakerVector.getY()));
    }

    /**
     * calculates the distance to the speaker across the field's X-axis in meters based on the current odometry position.
     * @return the distance from the speaker in meters.
     */
    public double getDistanceFromBotToSpeaker() {
        return botVector.distance(speakerVector);
    }

    /**
     * calculates the distance across the Y axis to the speaker from the robot's current odometry pose.
     * @return the distance from the speaker in meters
     */
    public double getYDifferenceFromSpeaker() {
        return speakerVector.getY() - botVector.getY();
    }

    /**
     * calculates the distance across the X axis to the speaker from the robot's current odometry pose.
     * @return the distance from the speaker in meters
     */
    public double getXDifferenceFromSpeaker() {
        return speakerVector.getX() - botVector.getX();
    }

    /**
     * calculates the nearest position in range to shoot in autonomous. WILL UPDATE LATER
     * @param botPose the position of the robot to calculate from
     * @return the nearest in range position as a Pose2d
     */
    public Pose2d getNearestInRangePositionInAuton(Pose2d botPose) {
        return new Pose2d(lineX, botPose.getY(), botPose.getRotation());
    }

    /**
     * calculates the yaw angle from the robot's XY pose to the speaker.
     * @return the angle in radians as a double
     */
    public double getAngleFromSpeaker(){
        return -1 * Math.toRadians(AngleUtil.angleToPoint(AngleUtil.toTranslation(speakerVector), AngleUtil.toTranslation(botVector)));
    }

    /**
     * calculates the angle difference from the gyro rotation to the speaker
     * @return the difference angle
     */
    public double getAngleDiferrenceToSpeaker() {
        return autonAimAssistPIDController.calculate(getGyroRad(), getAngleFromSpeaker());
    }

    /**
     * calculates the PID output to angle to robot chassis to the speaker.
     * @return the percent power as a -1 to 1
     */
    public double getPercentageToAngleToSpeaker() {
        double angleToSpeaker = getAngleFromSpeaker();
        // double angleToSpeaker = MathUtil.applyDeadband(getAngleFromSpeaker(), 0.1, Math.PI / 2);
        double gyroValue = getGyroRad();
        if(redAlliance){
            angleToSpeaker = AngleUtil.toFullAngleRadians(angleToSpeaker);
            gyroValue = AngleUtil.toFullAngleRadians(getGyroRad());
        }

        double angleSpeed;
        angleSpeed = teleopAimAssistPIDController.calculate(gyroValue, angleToSpeaker);;
        double busVolts = getBusVolts();

        double outputPercent = MathUtil.clamp(angleSpeed, -busVolts, busVolts) / busVolts;

        return outputPercent;
    }

    /**
     * @return The bus voltage.
     */
    public double getBusVolts() {
        return swerveModules[0].getBusVolts();
    }

    // #endregion

    // #region -*-*-*-*-*- sets -*-*-*-*-*-

    /**
     *  Sets the entire robot's desired states using the setDesiredStates function in the swerveModule class.
     * @param desiredStates the desired state set.
     */
    public void setAutonModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setAutonDesiredState(desiredStates[i]);
        }
    }

    public void setTeleopModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeed);
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setTeleopDesiredState(desiredStates[i]);
        }
    }



    @Override
    public void setPose(Pose2d pose) {
        odometry.resetPosition(
            getGyroRotation(),
            new SwerveModulePosition[] {
                swerveModules[0].getPosition(),
                swerveModules[1].getPosition(),
                swerveModules[2].getPosition(),
                swerveModules[3].getPosition()
            },
            pose
        );
    }

    public void setAutonCurrentLimits() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setAutonCurrentLimits();
        }
    }

    public void setTeleopCurrentLimits() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].setTeleopCurrentLimites();
        }    
    }

    // #endregion

    // #region -*-*-*-*-*- booleans -*-*-*-
    
    /**
     * calculates whether the robot is in range to shoot to it's alliance speaker.
     * @param botPose the position to calculate from
     * @param distance the acceptable distance from the speaker in meters
     * @return if the position is in range to the speaker from the given distance
     */
    public boolean isInShootingRange(Pose2d botPose, double distance) {
        boolean inRange;

        if (getSpeakerDistance(botPose) >= distance) {
            inRange = false;
            // System.out.println("**INCORRECT BUZZER** OUT OF RANGE, MAJOR L");
        } else {
            inRange = true;
            // System.out.println("**CORRECT BUZZER** IN RANGE, MAJOR W");
        }
        return inRange;
    }

    /**
     * calculates if the robot is within the autonomous shooting allowed zone.
     * @return if shooting distance is legal
     */
    public boolean isInAutonShootingRange() {
        double botPoseX = odometry.getPoseMeters().getX();

        if (redAlliance) {
            return (botPoseX >= 10.7);
        } else {
            return (botPoseX <= 5.87);
        }
    }

    /**
     * calculates whether there are any obstacles in the way of shooting.
     * @return if the current odometry pose is blocked by an obstacle 
     */
    public boolean isLineToStageIntersected() {
        boolean stageIntersected;

        //Transforms the Pose2d X and Y into a vector 2D (They better make a better way to do this)
        Vector2D stageVector;
        if (redAlliance) {
            stageVector = Vector2D.of(
                Constants.obstaclePositions.get(3).getX(),
                Constants.obstaclePositions.get(3).getY()
            );
        } else {
            stageVector = Vector2D.of(
                Constants.obstaclePositions.get(2).getX(),
                Constants.obstaclePositions.get(2).getY()
            );
        }

        Line speakerLine = Lines.fromPoints(botVector, speakerVector, lineToSpeakerObstaclePrecision);
        stageLineSegment = speakerLine.segment(botVector, speakerVector);
        Circle stageCircle = Circle.from(stageVector, .8, lineToSpeakerObstaclePrecision);

        Vector2D stageLineMidpoint = stageLineSegment.getCentroid();
        Vector2D stageIntersection = null;
        List<Vector2D> stageIntersections = stageCircle.intersections(speakerLine);

        double speakerAcceptableDistance = stageLineMidpoint.distance(speakerVector);

        // For each intersection in the list, it finds if the distance from the intersection to the speaker is greater than the distance from the robot pose to the speaker. 
        //If it is greater, it is behind and we should not use it.
        for (int i = 0; i < stageIntersections.size(); ++i) {
            Vector2D testedStageIntersection = stageIntersections.get(i);
            if (testedStageIntersection.distance(speakerVector) < botVector.distance(speakerVector)) {
                stageIntersection = testedStageIntersection;
            }
        }
        
        if (stageIntersection == null) {
            stageIntersected = false;
        } 
        else if (stageLineMidpoint.distance(stageIntersection) < speakerAcceptableDistance) {
            stageIntersected = true;
        } 
        else {
            stageIntersected = false;
        }
        return stageIntersected;

    }

    /** resets the gyro's currently set default angle. */
    public void resetGyro() {
        gyro.reset();
    }

    /** resets all drive encoders. DOES NOT RESET ODOMETRY. */
    public void resetEncoders() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].resetDriveEncoder();
        }
    }

    /** sets all swerve modules to zero power. */
    public void stopSystem() {
        for (int i = 0; i < 4; i++) {
            swerveModules[i].stop();
        }
    }

    //#endregion

}
