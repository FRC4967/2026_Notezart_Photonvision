package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;

public interface BotPoseSource {
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