package frc.robot;

import org.apache.commons.geometry.euclidean.twod.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

// for methods that are totally enclosed
public class AngleUtil {
    private static double precision = 0.000001;

    /**
     * Uses atan and a little math to get an angle we can use
     * @param centerPoint the center point
     * @param nextPoint the point to aim towards
     * @return angle from center point to next point from -180 to 180 in degrees
     */
    public static double angleToPoint(Translation2d centerPoint, Translation2d nextPoint) {
        double angleToPoint;
        double xDisplacement = nextPoint.getX() - centerPoint.getX();
        double yDisplacement = nextPoint.getY() - centerPoint.getY();

        // if y is zero automaticly sets angle to zero this makes it go all the way behind
        if (Math.abs(yDisplacement) < precision && xDisplacement < 0){
            angleToPoint = Math.PI;
        } else {
            // this makes angle go from - PI to PI instead of based from x axis
            if (xDisplacement < 0){
                if (yDisplacement > 0){
                    angleToPoint = Math.atan(yDisplacement / xDisplacement) + Math.PI;
                } else {
                    angleToPoint = Math.atan(yDisplacement / xDisplacement) - Math.PI;
                }
            } else {
                angleToPoint = Math.atan(yDisplacement / xDisplacement);
            }
        }
        
        return Math.toDegrees(angleToPoint);
    }
    
    /**
     * Uses atan and a little math to get an angle we can use
     * @param centerPoint the center point
     * @param nextPoint the point to aim towards
     * @return angle from center point to next point from -180 to 180 in degrees
     */
    public static double angleToPoint(Pose2d centerPoint, Pose2d nextPoint) {
        return angleToPoint(centerPoint.getTranslation(), nextPoint.getTranslation());
    }

    /**
     * Uses math to get the angle from percentages of two angles
     * @param angle1 first angle in degrees to get a percentage of
     * @param angle2 second angle in degrees to get a percentage of
     * @param percentage1 what percent to take from first angle from 0 to 1
     * @param percentage2 what percent to take from second angle from 0 to 1
     * @return An angle in degrees using percentages of two diffrent angles 
     */
    public static double getPercentageOfAngles(double angle1, double angle2, double percentage1, double percentage2){
        double percentageOfAngles;
        if(Math.abs(angle1 - angle2) > 180){
            // changes angles to go from 0 to 360
            if(angle1 < 0){
                angle1 = 360 + angle1;
            } else {
                angle2 = 360 + angle2;
            }
            percentageOfAngles = toUsableAngle((angle1 * percentage1) + (angle2 * percentage2));
        } else {
            percentageOfAngles = (angle1 * percentage1) + (angle2 * percentage2);
        }
        return percentageOfAngles;
    }

    /**
     * uses math to change a 0 to 360 angle to a -180 to 180 angle with 0 being same for both
     * @param angle angle from 0 to 360 in degrees
     * @return angle from -180 to 180 in degrees
     */
    public static double toUsableAngle(double angle){
        if(angle > 180){
            return angle - 360;
        } else {
            return angle;
        }
    }

    /**
     * uses math to change a -180 to 180 angle to a 0 to 360 angle with 0 being same for both
     * @param angle angle from -180 to 180 in degrees
     * @return angle from 0 to 360 in degrees
     */
    public static double toFullAngle(double angle){
        if(angle < 0){
            return angle + 360;
        } else {
            return angle;
        }
    }

    /**
     * uses math to change a -180 to 180 angle to a 0 to 360 angle with 0 being same for both
     * @param angle angle from -180 to 180 in radians
     * @return angle from 0 to 360 in raidians
     */
    public static double toFullAngleRadians(double angle){
        if(angle < 0){
            return angle + (2 * Math.PI);
        } else {
            return angle;
        }
    }

    /**
     * Translates a Vector2D into a Translation2D
     * @param vector the Vector2D to translate
     * @return the Translation2D translated
     */
    public static Translation2d toTranslation(Vector2D vector){
        return new Translation2d(vector.getX(), vector.getY());
    }

    /**
     * Translates a Translation2D into a Vector2D
     * @param translation the Translation2D to translate
     * @return the Vector2D translated
     */
    public static Vector2D toVector(Translation2d translation){
        return Vector2D.of(translation.getX(), translation.getY());
    }
}
