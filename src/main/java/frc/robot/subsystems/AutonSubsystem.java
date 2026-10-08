package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.geometry.euclidean.twod.Line;
import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Segment;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.shape.Circle;
import org.apache.commons.numbers.core.Precision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AngleUtil;
import frc.robot.TrajectoryWaypoints;
import frc.robot.TrajectoryWaypoints.AllianceConfig;
import frc.robot.TrajectoryWaypoints.BlueAlliance;
import frc.robot.TrajectoryWaypoints.RedAlliance;
import frc.robot.commands.autonomous.AutonSpeakerPreloadCommand;
import frc.robot.commands.autonomous.AutonToppyCommand;
import frc.robot.commands.autonomous.AdaptiveTrajectoryFollowCommand;
import frc.robot.commands.GyroResetCommand;
import frc.robot.commands.OdometryResetCommand;
import frc.robot.commands.autonomous.AutonAngleToSpeakerCommand;
import frc.robot.commands.autonomous.AutonAwayShotCommand;
import frc.robot.commands.autonomous.AutonChassisCommand;
import frc.robot.commands.autonomous.AutonIntakeCommand;
import frc.robot.commands.autonomous.AutonIntakeWaitCommand;
import frc.robot.commands.autonomous.AutonShooterCommand;
import frc.robot.commands.autonomous.TrajectoryFollowCommand;

public class AutonSubsystem extends SubsystemBase{
    private final SwerveSubsystem swerveSubsystem;
    private final Aprilometry aprilometry;
    private ShooterSubsystem shooterSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;
    private PhotonVision photonVision;
    private TrajectoryWaypoints trajectoryWaypoints;
    private AllianceConfig allianceConfig;
    private Boolean isRedSide;
    private SequentialCommandGroup autonRoutine;

    private Trajectory trajectory;

    private List<Pose2d> schedule;
    private Pose2d startPosition;
    private Translation2d endPosition;
    private List<Translation2d> notePositions;
    private List<Pose2d> shootPositions;

    private Integer startChoice;
    private Integer endingChoice;
    private Integer initialShotChoice;

    private ProfiledPIDController thetaController;
    private Constraints constraints;

    private SendableChooser<Integer> startChooser = new SendableChooser<>();
    private SendableChooser<Integer> endingChooser = new SendableChooser<>();
    private SendableChooser<Integer> initialShotChooser = new SendableChooser<>();
    private SendableChooser<Integer> typeChooser = new SendableChooser<>();
    private NoteWidget noteWidget = new NoteWidget();

    // Constants for interior waypoints and needs changed
    private double robotX = 0.46;
    private double robotY = 0.46;
    private double robotRadius = Math.sqrt(Math.pow(robotX, 2) + Math.pow(robotY, 2));
    // private double poleRadius = 0.59397;
    private double poleRadius = 0.55;
    private double notesShotCount = 0;

    private double circleRaduis = poleRadius + robotRadius;

    private final double timeoutIntake = 2;
    private final double timeoutShooter = 9;
    
    // Represents the points where the steel trusses are.
    private List<Vector2D> poles = new ArrayList<>(List.of(
        Vector2D.of(5.6, 5.38),
        Vector2D.of(3.4, 4.10),
        Vector2D.of(5.6, 2.85),
        Vector2D.of(10.97, 5.39),
        Vector2D.of(13.20, 4.11),
        Vector2D.of(10.97, 2.86)
    ));

    // used to fix the curveness of the trajectory generation
    private double curveFix = 0.25;

    private TrajectoryConfig config = new TrajectoryConfig(4, 2.5);


    public AutonSubsystem(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem, PhotonVision photonVision, Aprilometry aprilometry, TrajectoryWaypoints trajectoryWaypoints) {
        this.swerveSubsystem = swerveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;
        this.photonVision = photonVision;
        this.aprilometry = aprilometry;
        this.trajectoryWaypoints = trajectoryWaypoints;

        schedule = new ArrayList<>();
        constraints = new TrapezoidProfile.Constraints(Math.PI, 0.5 * Math.PI);
        thetaController = new ProfiledPIDController(10, 0, 1.25, constraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        /*  non will assume not moving at all during match. */
        startChooser.setDefaultOption("non", 0);
        startChooser.addOption("Left of Speaker", 1);
        startChooser.addOption("Right of Speaker", 2);
        startChooser.addOption("Center of Speaker", 3);      
        startChooser.addOption("Squared by Opponent Pickup", 4);
        startChooser.addOption("Against left of speaker ", 5);
        startChooser.addOption("Against right of speaker", 6);
        SmartDashboard.putData("Where is starting position?", startChooser);

        initialShotChooser.setDefaultOption("yes", 0);
        initialShotChooser.addOption("yes, no chassis angling (only for against speaker)", 1);
        initialShotChooser.addOption("yes, but while moving!", 2);
        SmartDashboard.putData("take preload shot?", initialShotChooser);

        /* I wrote a bunch of ending choices i could think of here 
        in case we want to implement some, but we may not use them considering how much
        we're trying to do in autonomous. Low in priority. */
        endingChooser.setDefaultOption("Nothing", -1);
        endingChooser.addOption("just go out", 0);
        endingChooser.addOption("towards speaker", 1);
        endingChooser.addOption("towards source", 2);
        SmartDashboard.putData("Extra ending?", endingChooser);

        typeChooser.setDefaultOption("static auton", 0);
        typeChooser.addOption("adaptive auton", 1);
        typeChooser.addOption("semi adaptive auton", 2);
        SmartDashboard.putData("type of auton?", typeChooser);

        /* sends a number you can change on the dashboard to set the order of the rings. */
        SmartDashboard.putData("note route", noteWidget);

        SmartDashboard.putNumber("X meters to test", 1);
        SmartDashboard.putNumber("Y meters to test", 0);
        SmartDashboard.putNumber("degree rotations to test", 0);
    }

    public Command TrajectoryIntoCommand(Trajectory trajectory, Boolean isGoingToNote, Pose2d goToPose) {
        return new TrajectoryFollowCommand(
            this,
            swerveSubsystem,
            shooterSubsystem,
            intakeSubsystem,
            compliantWheelsSubsystem,
            trajectory,
            thetaController,
            isGoingToNote,
            goToPose
        );
    }

    public Command AdaptiveTrajectoryCommand(Pose2d startPos, Pose2d endPos) {
        double distanceFacor = MathUtil.clamp(startPos.getTranslation().getDistance(endPos.getTranslation()) / 4.5, .7, 1);
        config = new TrajectoryConfig(4 * distanceFacor, 2 * distanceFacor);
        Trajectory newTrajectory = getTrajectory(startPos, endPos);
        return new AdaptiveTrajectoryFollowCommand(
            this, 
            swerveSubsystem, 
            photonVision,
            () -> compliantWheelsSubsystem.isShooterLineBreakBroken(), 
            newTrajectory, 
            thetaController,
            endPos
        );
    }

    public Command intakeCommand() {
        return new AutonIntakeCommand(
            this, 
            swerveSubsystem,
            shooterSubsystem,
            intakeSubsystem,
            compliantWheelsSubsystem
        ).withTimeout(timeoutIntake);
    }

    public Command waitToIntakeCommand() {
        return new AutonIntakeWaitCommand(
            swerveSubsystem,
            () -> compliantWheelsSubsystem.isShooterLineBreakBroken()
        ).withTimeout(timeoutIntake - 1);
    }

    public Command getInitialShotCommand() {
        if (initialShotChooser.getSelected() == 1) {
            return new AutonSpeakerPreloadCommand(
                shooterSubsystem, 
                compliantWheelsSubsystem
            ).withTimeout(timeoutShooter);
        } else {
            return newShootCommand(2500);
        }
    }

    public Command AngleToSpeakerCommand(double type) {
        return new AutonAngleToSpeakerCommand(
            swerveSubsystem,
            () -> compliantWheelsSubsystem.isShooterLineBreakBroken(),
            type
        );
    }

    public Command newShootCommand(double RPM) {
        return shootCommand(RPM);
    }

    public Command shootCommand(double RPM) {
        return new AutonShooterCommand(
            this,
            shooterSubsystem,
            swerveSubsystem,
            compliantWheelsSubsystem,
            RPM
        ).withTimeout(timeoutShooter);
    }

    public Command movingShotCommand(Trajectory newTrajectory) {
        return new AutonAwayShotCommand(
            this, 
            swerveSubsystem, 
            shooterSubsystem, 
            intakeSubsystem, 
            compliantWheelsSubsystem, 
            newTrajectory, 
            thetaController
        );
    }

    public void updateNotesShotCount() {
        notesShotCount ++;
    }

    public double getNotesShotCount() {
        return notesShotCount;
    }
    
    public void updateShuffleboardValues() {
        startChoice = startChooser.getSelected();
        endingChoice = endingChooser.getSelected();
        initialShotChoice = initialShotChooser.getSelected();

        /* TODO: Try to use the interpolate function
         * to eliminate the need for different alliance waypoints.
         */
        switch (DriverStation.getAlliance().get()) {
            case Blue:
            /* changes the alliance config to grab the blue alliance waypoints. */
            isRedSide = false;
            allianceConfig = new BlueAlliance();
            break;
            case Red:
            /*  changes the alliance config to grab the red alliance waypoints. */
            isRedSide = true;
            allianceConfig = new RedAlliance();
            break;
        }

        /* grab the definite waypoints */
        startPosition = allianceConfig.getStartingPositions().get(startChoice);
        if (endingChoice != -1) {
        endPosition = allianceConfig.getEndPositions().get(endingChoice);
        }
        notePositions = allianceConfig.getNotePositions();
        shootPositions = allianceConfig.getNoteShootPositions();
    }

    /**
     * Uses math to get the interior points of a trajectory
     * @param startPose Pose at the start of the trajectory
     * @param endPose Pose at the end of the trajectory
     * @return All waypoints to create trajectory
     */
    private List<Pose2d> getWaypoints(Pose2d startPose, Pose2d endPose){
        Pose2d trueEndPose = new Pose2d(endPose.getX(), endPose.getY(), endPose.getRotation());
        boolean differentEndPose = false;
        if(endPose.getTranslation() == notePositions.get(2)){
            endPose = allianceConfig.getPoleAvoidPoint();
            differentEndPose = true;
        } else {
            endPose = trueEndPose;
        }
        
        // can change percision based on testing
        Precision.DoubleEquivalence precision = Precision.doubleEquivalenceOfEpsilon(1e-6);

        List<Translation2d> waypointTranslations = new ArrayList<>();
        List<Pose2d> waypoints = new ArrayList<>();
        
        Vector2D startPoint = Vector2D.of(startPose.getX(), startPose.getY());
        Vector2D endPoint = Vector2D.of(endPose.getX(), endPose.getY());

        List<Circle> circleToAvoid = new ArrayList<>();
        List<List<Vector2D>> intersections = new ArrayList<>();

        Integer first = 0;
        Integer second = 1;
        double perpAngleToPole;

        waypointTranslations.add(startPose.getTranslation());

        // sorts poles from closest to start to furthest
        poles.sort((a, b) -> {
            
            double aDist = a.distance(startPoint);
            double bDist = b.distance(startPoint);

            return (int) (aDist - bDist);
        });

        Vector2D currentStartPoint = startPoint;

        // gets the interior points without headings
        for(int i = 0; i < poles.size(); i++){
            Vector2D currentPole = poles.get(i);
            Line currentLine = Lines.fromPoints(currentStartPoint, endPoint, precision);
            Segment currentSegment = currentLine.segment(currentStartPoint, endPoint);
            Vector2D currentMidpoint = currentSegment.getCentroid();
            double acceptableDist = currentStartPoint.distance(currentMidpoint);

            circleToAvoid.add(Circle.from(currentPole, circleRaduis, precision));
            intersections.add(circleToAvoid.get(i).intersections(currentLine));

            List<Vector2D> currentIntersectionPoints = intersections.get(i);

            if(!currentIntersectionPoints.isEmpty()){
                // If the intersection is past where were going get rid of that intersection
                double distToFirstIntersection = currentMidpoint.distance(currentIntersectionPoints.get(first));
                double distToSecondIntersection = currentMidpoint.distance(currentIntersectionPoints.get(second));
                if(distToSecondIntersection > acceptableDist || distToFirstIntersection > acceptableDist){
                    intersections.set(i, List.of());
                } 
                // If there is one intersection we end near pole which doesn't need interior waypoint (hopefully)
                else if(currentIntersectionPoints.size() > 1){
                    // only two intersections between a line and a circle
                    Vector2D firstIntersection = currentIntersectionPoints.get(first);
                    Vector2D secondIntersection = currentIntersectionPoints.get(second);
                    Vector2D intersectionMidpoint = currentLine.segment(firstIntersection, secondIntersection).getCentroid();

                    // gives error if the bot goes striaght through a pole instead makes it go right
                    if(currentPole.vectorTo(intersectionMidpoint).isZero(precision)){
                        perpAngleToPole = currentLine.getAngle() + (Math.PI/2);
                    } else {
                        Line perpendicularLine = Lines.fromPoints(currentPole, intersectionMidpoint, precision);
                        perpAngleToPole = perpendicularLine.getAngle();
                    }
                    Translation2d translationToAdd = new Translation2d(currentPole.getX() + (circleRaduis * Math.cos(perpAngleToPole)), currentPole.getY() + (circleRaduis * Math.sin(perpAngleToPole)));
                    if (translationToAdd.getDistance(waypointTranslations.get(waypointTranslations.size() - 1)) > 1.1){
                        waypointTranslations.add(translationToAdd);
                        // changes start point so we dont get all intersections from the start of trajectory
                        currentStartPoint = Vector2D.of(currentPole.getX() + (circleRaduis * Math.cos(perpAngleToPole)), currentPole.getY() + (circleRaduis * Math.sin(perpAngleToPole)));
                    }
                }
            }
        }

        waypointTranslations.add(endPose.getTranslation());

        // gives translations headings
        double angleToFirstPoint = AngleUtil.angleToPoint(waypointTranslations.get(first), waypointTranslations.get(second));

        if(startPose.getX() < endPose.getX() || (isRedSide && startPose.getX() == endPose.getX())){
            // may need to change percentages
            waypoints.add(new Pose2d(startPose.getTranslation(), 
            new Rotation2d(Math.toRadians(AngleUtil.getPercentageOfAngles(0, angleToFirstPoint, 0.8, 0.2)))));   
        } else {
            waypoints.add(new Pose2d(startPose.getTranslation(), 
            new Rotation2d(Math.toRadians(AngleUtil.getPercentageOfAngles(180, angleToFirstPoint, 0.8, 0.2)))));
        }

        for(int i = 1; i < waypointTranslations.size() - 1; i++){
            Translation2d currentVector = waypointTranslations.get(i);
            Translation2d nextVector = waypointTranslations.get(i+1);
            Translation2d lastVector = waypointTranslations.get(i-1);
            double angleToNext;
            double angleFromLast;
            double angleBetweenPoints;

            angleToNext = AngleUtil.angleToPoint(currentVector, nextVector);
            angleFromLast = AngleUtil.angleToPoint(lastVector, currentVector);

            // makes start to first point less angled
            if(i == 1){
                angleBetweenPoints = Math.toRadians(AngleUtil.getPercentageOfAngles(angleFromLast, angleToNext, 0.75, 0.25));
                waypoints.add(new Pose2d(currentVector.getX() - (curveFix * Math.cos(angleBetweenPoints)), currentVector.getY() - (curveFix * Math.sin(angleBetweenPoints)), new Rotation2d(angleBetweenPoints)));
            } else {
                angleBetweenPoints = Math.toRadians(AngleUtil.getPercentageOfAngles(angleFromLast, angleToNext, 0.5, 0.5));
                waypoints.add(new Pose2d(currentVector.getX() + (curveFix * Math.cos(angleBetweenPoints)), currentVector.getY() + (curveFix * Math.sin(angleBetweenPoints)), new Rotation2d(angleBetweenPoints)));
            }
        }

        double lastAngle;
        // makes sure bot goes straighter on the sides
        if(waypointTranslations.size() == 2){
            if (startPose.getX() > trueEndPose.getX() || (isRedSide && startPose.getX() == endPose.getX())){
                lastAngle = Math.PI;
            } else {
                lastAngle = 0;
            }
        } else {
            Translation2d secondToLast = waypointTranslations.get(waypointTranslations.size() - 2);
            lastAngle = Math.toRadians(AngleUtil.angleToPoint(secondToLast, trueEndPose.getTranslation()));
        }
        if(differentEndPose){
            waypoints.add(endPose);
        }

        waypoints.add(new Pose2d(trueEndPose.getTranslation(), new Rotation2d(lastAngle)));

        return waypoints;
    }

    public Pose2d getPose(Translation2d translation2d) {
        Rotation2d newRotation = new Rotation2d(0);
        return new Pose2d(translation2d, newRotation);
    }

    private void updateNoteOrderRoutine() {
        List<Integer> orderList = new ArrayList<>();
        double[] noteSequence = noteWidget.getNoteSequence();

        for (double noteI: noteSequence) {
            orderList.add((int) noteI);
        }

        for (int i = 1; i <= notePositions.size(); i++) { /* index MUST start at 1 because zeroes are skipped. */
            int noteIndex = orderList.indexOf(i); 
            if (noteIndex != -1) { /* filters null values */
                System.out.println("adding note" + noteIndex + "to order" + i);
                schedule.add(getPose(notePositions.get(noteIndex)));
                schedule.add(shootPositions.get(noteIndex));
            }
        }
    }

    public Trajectory getTrajectory(Pose2d startPos, Pose2d endPos) {
        /* creates a list of poses with interior waypoint logic integrated. */
        List<Pose2d> waypoints = getWaypoints(startPos, endPos);

        /* interior points gives start and end becuase there headings need changed based on path */
        Trajectory newTrajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

        return newTrajectory;
    }

    public List<Pose2d> getNoteOrderRoutine() {
        List<Integer> orderList = new ArrayList<>();
        List<Pose2d> posList = new ArrayList<>();
        double[] noteSequence = noteWidget.getNoteSequence();

        for (double noteI: noteSequence) {
            orderList.add((int) noteI);
        }

        for (int i = 1; i <= notePositions.size(); i++) { /* index MUST start at 1 because zeroes are skipped. */
            int noteIndex = orderList.indexOf(i); 
            if (noteIndex != -1) { /* filters null values */
                System.out.println("adding note" + noteIndex + "to order" + i);
                posList.add(getPose(notePositions.get(noteIndex)));

                /* doesn't add the position to the list if it's the same pos */
                // if (!shootPositions.get(noteIndex).getTranslation().equals(notePositions.get(noteIndex))) {
                posList.add(shootPositions.get(noteIndex));
                // }
            }
        }

        return posList;
    }

    /* temp auton, remove after kentwood */
    private List<Pose2d> getEvilRoutine() {
        List<Pose2d> orderList = new ArrayList<>();

        orderList.add(startPosition);

        orderList.add(new Pose2d(13.71, 2.84, new Rotation2d(0)));
        orderList.add(new Pose2d(10.72, 1.28, new Rotation2d(0)));
        orderList.add(getPose(notePositions.get(7)));
        orderList.add(getPose(notePositions.get(6)));

        return orderList;
    }

    public Pose2d getInitialPose() {
        return startPosition;
    }

    public Command getAutonCommand() {
        if (typeChooser.getSelected() == 0) {
            schedule.clear(); /* prevents routines from adding previous runs*/
            autonRoutine = new SequentialCommandGroup();
            updateShuffleboardValues();

            schedule.add(startPosition); /* adds start pos (duh) */
            updateNoteOrderRoutine(); /* adds the note positions */

            int iStart = 0;

            if (initialShotChoice == 2){ /* moving and shooting! oh my gah */
                List<Pose2d> initialWaypoints = getWaypoints(schedule.get(0), schedule.get(1));
                Trajectory initialTrajectory = TrajectoryGenerator.generateTrajectory(initialWaypoints, config);
                autonRoutine.addCommands(movingShotCommand(initialTrajectory));
                autonRoutine.addCommands(intakeCommand());
                iStart = 1;
            } else {
                autonRoutine.addCommands(getInitialShotCommand());
            }

            if (endingChoice != -1 ) { /* so you don't have to choose an ending. */
                schedule.add(getPose(endPosition)); /* adds endpos but honestly I still think we'll delete this */
            }

            for (int i = iStart; i < schedule.size() - 1; i++) {
                boolean isIntaking = (i % 2 == 0);
                Pose2d currentPose = schedule.get(i);
                Pose2d nextPose = schedule.get(i + 1);

                /* prevents a move command if no movement is needed. */
                if (!currentPose.getTranslation().equals(nextPose.getTranslation())) {
                    trajectory = getTrajectory(currentPose, nextPose);

                    /* accesses the feedfollow command. */
                    autonRoutine.addCommands(TrajectoryIntoCommand(trajectory, isIntaking, nextPose));
                }

                if (isIntaking) {
                    autonRoutine.addCommands(intakeCommand());
                } else {
                    autonRoutine.addCommands(newShootCommand(3500));
                }
            }

            return autonRoutine;
            
        } else if (typeChooser.getSelected() == 1){
            updateShuffleboardValues();
            ParallelCommandGroup newAutonCommandGroup = new ParallelCommandGroup();
            notesShotCount = 0;

            newAutonCommandGroup.addCommands(new AutonChassisCommand(
                this, 
                swerveSubsystem,
                compliantWheelsSubsystem
            ));

            newAutonCommandGroup.addCommands(new AutonToppyCommand(
                this,
                swerveSubsystem, 
                shooterSubsystem, 
                intakeSubsystem, 
                compliantWheelsSubsystem
            ));

            /* shoot command is the initial shot. Cannot do move away shot yet. */
            return getInitialShotCommand().andThen(newAutonCommandGroup);
        } else {
            // schedule.clear(); /* prevents routines from adding previous runs*/
            // autonRoutine = new SequentialCommandGroup();
            // updateShuffleboardValues();

            // schedule.add(startPosition); /* adds start pos (duh) */
            // updateNoteOrderRoutine(); /* adds the note positions */

            // autonRoutine.addCommands(getInitialShotCommand());

            // Trajectory firstTrajectory = TrajectoryGenerator.generateTrajectory(List.of(startPosition, getPose(notePositions.get(0))), config);
            // autonRoutine.addCommands(TrajectoryIntoCommand(firstTrajectory, true, startPosition));

            // autonRoutine.addCommands(newShootCommand(3500));

            // TrajectoryConfig demonConfig = new TrajectoryConfig(6, 3.5);

            // Trajectory demonTrajectory = TrajectoryGenerator.generateTrajectory(getEvilRoutine(), demonConfig);
            // autonRoutine.addCommands(demonTrajectory(demonTrajectory, startPosition));

            return autonRoutine;
        } 
    }
}