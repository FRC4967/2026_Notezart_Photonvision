package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;

public interface BotPoseSourceType {
    // This defines swerve, direct vision, and combined field as the same type of thing (A source of the robot position)

    /**
     * @return the BotPoseSource location of the robot.
    */
    public Pose2d getPose();

    /**
    * sets the BotPoseSource location of the robot.
    * @param newPose the pose to set the robot to
    */
    public void setPose(Pose2d newPose);
    
    }