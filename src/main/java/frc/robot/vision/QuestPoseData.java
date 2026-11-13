package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public class QuestPoseData {
    /* This file is just the unit format for quest pose data
     * For details on quest code, see here:
     * https://questnav.gg/docs/getting-started/robot-code */

    private Optional<Pose2d> botEstimatedPose;
    private Optional<Double> timestampSeconds;

    /** Puts info into the unit format for quest pose data
     * @param estimatedRobotPose Quest's estimated robot pose
     * @param timestampSeconds Quest timestamp from turning on to now
     */
    public QuestPoseData(Pose3d estimatedRobotPose, Double timestampSeconds){
        this.botEstimatedPose = Optional.ofNullable(estimatedRobotPose.toPose2d());
        this.timestampSeconds = Optional.ofNullable(timestampSeconds);
    }

    /** @return The estimated pose given by the quest */
    public Optional<Pose2d> getEstimatedPose() {
        return botEstimatedPose;
    }

    /** @return The quest timestamp from turning on to now */
    public Optional<Double> getTimestamp() {
        return timestampSeconds;
    }

    /** @return true if timestamp and estimated pose values both exist */
    public boolean isPresent() {
        return getTimestamp().isPresent() && getEstimatedPose().isPresent();
    }
}
