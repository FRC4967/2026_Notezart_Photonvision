package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.apache. commons.geometry.euclidean.twod.Line;
import org.apache.commons.geometry.euclidean.twod.Lines;
import org.apache.commons.geometry.euclidean.twod.Segment;
import org.apache.commons.geometry.euclidean.twod.Vector2D;
import org.apache.commons.geometry.euclidean.twod.shape.Circle;
import org.apache.commons.numbers.core.Precision;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.AngleUtil;
import frc.robot.TrajectoryWaypoints.AllianceConfig;
import frc.robot.TrajectoryWaypoints.BlueAlliance;

public class AutonSubsystemTest {
    AllianceConfig allianceConfig = new BlueAlliance();
    
    // Constants for interior waypoints and needs changed
    private double robotX = 0.46;
    private double robotY = 0.46;
    private double robotRadius = Math.sqrt(Math.pow(robotX, 2) + Math.pow(robotY, 2));
    // private double poleRadius = 0.59397;
    private double poleRadius = 0.55;

    private double circleRaduis = poleRadius + robotRadius;
    
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

    @Test
    public void testGettingPoints(){
        // Pose2d startPose = new Pose2d(allianceConfig.getNotePositions().get(3), new Rotation2d());
        // Pose2d endPose = allianceConfig.getStartingPositions().get(0);
        Pose2d startPose = allianceConfig.getStartingPositions().get(2);
        Pose2d endPose = new Pose2d(allianceConfig.getNotePositions().get(5), new Rotation2d());
        List<Pose2d> interiorWaypoints = getWaypoints(startPose, endPose);

        // System.out.println("startX " + startPose.getX());
        // System.out.println("startY " + startPose.getY());
        // Assertions.assertEquals(2, interiorWaypoints.size());
        
        if(interiorWaypoints.size() == 2){
            System.out.println("Start X: " + interiorWaypoints.get(0).getX());
            System.out.println("Start Y: " + interiorWaypoints.get(0).getY());
            System.out.println("Start heading: " + interiorWaypoints.get(0).getRotation().getDegrees());
            System.out.println("End X: " + interiorWaypoints.get(1).getX());
            System.out.println("End Y: " + interiorWaypoints.get(1).getY());
            System.out.println("End heading: " + interiorWaypoints.get(1).getRotation().getDegrees());
        } else if(interiorWaypoints.size() == 3){
            System.out.println("Start X: " + interiorWaypoints.get(0).getX());
            System.out.println("Start Y: " + interiorWaypoints.get(0).getY());
            System.out.println("Start heading: " + interiorWaypoints.get(0).getRotation().getDegrees());
            System.out.println("X1: " + interiorWaypoints.get(1).getX());
            System.out.println("Y1: " + interiorWaypoints.get(1).getY());
            System.out.println("Heading1 " + interiorWaypoints.get(1).getRotation().getDegrees());
            System.out.println("End X: " + interiorWaypoints.get(2).getX());
            System.out.println("End Y: " + interiorWaypoints.get(2).getY());
            System.out.println("End heading: " + interiorWaypoints.get(2).getRotation().getDegrees());
        } else if(interiorWaypoints.size() == 4){
            System.out.println("Start X: " + interiorWaypoints.get(0).getX());
            System.out.println("Start Y: " + interiorWaypoints.get(0).getY());
            System.out.println("Start heading: " + interiorWaypoints.get(0).getRotation().getDegrees());
            System.out.println("X1: " + interiorWaypoints.get(1).getX());
            System.out.println("Y1: " + interiorWaypoints.get(1).getY());
            System.out.println("Heading1 " + interiorWaypoints.get(1).getRotation().getDegrees());
            System.out.println("X2: " + interiorWaypoints.get(2).getX());
            System.out.println("Y2: " + interiorWaypoints.get(2).getY());
            System.out.println("Heading2 " + interiorWaypoints.get(2).getRotation().getDegrees());
            System.out.println("End X: " + interiorWaypoints.get(3).getX());
            System.out.println("End Y: " + interiorWaypoints.get(3).getY());
            System.out.println("End heading: " + interiorWaypoints.get(3).getRotation().getDegrees());
        } else if(interiorWaypoints.size() == 5){
            System.out.println("Start X: " + interiorWaypoints.get(0).getX());
            System.out.println("Start Y: " + interiorWaypoints.get(0).getY());
            System.out.println("Start heading: " + interiorWaypoints.get(0).getRotation().getDegrees());
            System.out.println("X1: " + interiorWaypoints.get(1).getX());
            System.out.println("Y1: " + interiorWaypoints.get(1).getY());
            System.out.println("Heading1 " + interiorWaypoints.get(1).getRotation().getDegrees());
            System.out.println("X2: " + interiorWaypoints.get(2).getX());
            System.out.println("Y2: " + interiorWaypoints.get(2).getY());
            System.out.println("Heading2 " + interiorWaypoints.get(2).getRotation().getDegrees());
            System.out.println("X3: " + interiorWaypoints.get(3).getX());
            System.out.println("Y3: " + interiorWaypoints.get(3).getY());
            System.out.println("Heading3 " + interiorWaypoints.get(3).getRotation().getDegrees());
            System.out.println("End X: " + interiorWaypoints.get(4).getX());
            System.out.println("End Y: " + interiorWaypoints.get(4).getY());
            System.out.println("End heading: " + interiorWaypoints.get(4).getRotation().getDegrees());
        }
        // System.out.println("X3: " + interiorWaypoints.get(2).getX());
        // System.out.println("Y3: " + interiorWaypoints.get(2).getY());
        // System.out.println("Heading3 " + interiorWaypoints.get(2).getRotation().getDegrees());
        // System.out.println("X4: " + interiorWaypoints.get(3).getX());
        // System.out.println("Y4: " + interiorWaypoints.get(3).getY());
        // System.out.println("Heading4 " + interiorWaypoints.get(3).getRotation().getDegrees());
        // System.out.println("X5: " + interiorWaypoints.get(4).getX());
        // System.out.println("Y5: " + interiorWaypoints.get(4).getY());
        // System.out.println("Heading5 " + interiorWaypoints.get(4).getRotation().getDegrees());
        // System.out.println("X6: " + interiorWaypoints.get(5).getX());
        // System.out.println("Y6: " + interiorWaypoints.get(5).getY());
        // System.out.println("Heading6 " + interiorWaypoints.get(5).getRotation().getDegrees());
    }

    @Test
    private List<Pose2d> getWaypoints(Pose2d startPose, Pose2d endPose){
        double angleToEnd = Math.toRadians(AngleUtil.angleToPoint(startPose.getTranslation(), endPose.getTranslation()));
        double xChange = ((robotX + robotY) / 7) * Math.cos(angleToEnd);
        double yChange = ((robotX + robotY) / 7) * Math.sin(angleToEnd);
        // startPose = new Pose2d(startPose.getX() - xChange, startPose.getY() - yChange, startPose.getRotation());
        endPose = new Pose2d(endPose.getX() + xChange, endPose.getY() + yChange, endPose.getRotation());
        
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

        if(startPose.getX() < endPose.getX()){
            // may need to change percentages
            waypoints.add(new Pose2d(startPose.getTranslation(), 
            new Rotation2d(Math.toRadians(AngleUtil.getPercentageOfAngles(0, angleToFirstPoint, 0.5, 0.5)))));   
        } else {
            waypoints.add(new Pose2d(startPose.getTranslation(), 
            new Rotation2d(Math.toRadians(AngleUtil.getPercentageOfAngles(180, angleToFirstPoint, 0.5, 0.5)))));
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
            if (startPose.getX() <= endPose.getX()){
                lastAngle = 0;
            } else {
                lastAngle = Math.PI;
            }
        } else {
            Translation2d secondToLast = waypointTranslations.get(waypointTranslations.size() - 2);
            lastAngle = Math.toRadians(AngleUtil.angleToPoint(secondToLast, endPose.getTranslation()));
        }

        waypoints.add(new Pose2d(endPose.getTranslation(), new Rotation2d(lastAngle)));

        return waypoints;
    }
}
