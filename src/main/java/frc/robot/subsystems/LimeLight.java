package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimeLight extends SubsystemBase{
    private final NetworkTable limelightTable;

    // explanations for each and every variable are in their respective getters.
    private final NetworkTableEntry botPoseEntry;
    private final NetworkTableEntry botPoseTargetSpaceEntry;
    private final NetworkTableEntry camPoseRobotSpaceEntry;
    private final NetworkTableEntry camPoseTargetSpaceEntry;
    private final NetworkTableEntry targetPoseCameraSpaceEntry;
    private final NetworkTableEntry targetPoseRobotSpaceEntry;
    private final NetworkTableEntry clEntry;
    private final NetworkTableEntry hbEntry;
    private final NetworkTableEntry taEntry;
    private final NetworkTableEntry tcEntry;
    private final NetworkTableEntry thorEntry;
    private final NetworkTableEntry tidEntry;
    private final NetworkTableEntry tlEntry;
    private final NetworkTableEntry tlongEntry;
    private final NetworkTableEntry tsEntry;
    private final NetworkTableEntry tshortEntry;
    private final NetworkTableEntry tvEntry;
    private final NetworkTableEntry tvertEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final double[] defaultValue = {0.0,0.0,0.0};
    private NetworkTableEntry ledModeEntry;
    private final double fieldWidth = 8.211;
    private final double fieldLength = 16.451;

    // Note for the positioning of the bot:
    // Green Cylinder: Individual per-tag bot pose
    // Blue Cylinder: Old BotPose
    // White Cylinder: MegaTag Botpose

    // this constructor is if the default isn't working, otherwise use the one with no parameters.
    public LimeLight(NetworkTableInstance networkTable, String tableName) {
        this.limelightTable = networkTable.getTable(tableName);

        this.botPoseEntry = limelightTable.getEntry("botpose");
        this.botPoseTargetSpaceEntry = limelightTable.getEntry("botpose_targetspace");
        this.camPoseRobotSpaceEntry = limelightTable.getEntry("camerapose_robotspace");
        this.camPoseTargetSpaceEntry = limelightTable.getEntry("camerapose_targetspace");
        this.targetPoseCameraSpaceEntry = limelightTable.getEntry("targetpose_cameraspace");
        this.targetPoseRobotSpaceEntry = limelightTable.getEntry("targetpose_robotspace");

        /*  Capture pipeline latency (ms) */
        this.clEntry = limelightTable.getEntry("cl"); 


        this.hbEntry = limelightTable.getEntry("hb");

        // Target Area
        this.taEntry = limelightTable.getEntry("ta"); 

        // Get the average HSV color underneath the crosshair region (numberarray)
        this.tcEntry = limelightTable.getEntry("tc");

        // Horizontal sidelength of the bounding box
        this.thorEntry = limelightTable.getEntry("thor");

        //ID of the most visible april tag
        this.tidEntry = limelightTable.getEntry("tid"); 

        //The pipeline's latency
        this.tlEntry = limelightTable.getEntry("tl");

        //Sidelength of longest side of the bounding box
        this.tlongEntry = limelightTable.getEntry("tlong");


        this.tsEntry = limelightTable.getEntry("ts");

        //Sidelength of shortest side of the bounding box
        this.tshortEntry = limelightTable.getEntry("tshort");

        //Whether the limelight has any valid targets
        this.tvEntry = limelightTable.getEntry("tv");

        //Vertical sidelength of the bounding box
        this.tvertEntry = limelightTable.getEntry("tvert");

        //Horizontal Offset From Crosshair To Target
        this.txEntry = limelightTable.getEntry("tx");

        //Vertical Offset From Crosshair To Target
        this.tyEntry = limelightTable.getEntry("ty");

        //LED mode: can be 0-3
        this.ledModeEntry = limelightTable.getEntry("ledMode");
        
    }

    // use this constructor unless the network table cannot be found.
    public LimeLight() {
        this(NetworkTableInstance.getDefault(), "limelight-tot");
    }

    /**
    * Returns the robot pose as a Pose2d.
    * @return Pose2d
    */ 
    public Pose2d getBotPose2d() {
        double[] poseArray = botPoseEntry.getDoubleArray(defaultValue);
        // Takes the value from the limelight and adds half of the field dimensions
        // because the 0,0 point on limelight is in the middle of the field.
        if (poseArray.length <= 3) {
            return new Pose2d(
            poseArray[0] + (fieldLength / 2), 
            poseArray[1] + (fieldWidth / 2), 
            new Rotation2d());
        } 
        return new Pose2d(
        poseArray[0] + (fieldLength / 2), 
        poseArray[1] + (fieldWidth / 2), 
        new Rotation2d(Math.toRadians(poseArray[5])));
        
        
    }

    /**
    * Returns the robot pose as a Pose3d.
    * @return Pose3d
    */ 
    public Pose3d getBotPose3d() {
        double[] poseArray = botPoseEntry.getDoubleArray(defaultValue);
        return new Pose3d(
            poseArray[0] + (fieldLength / 2), 
            poseArray[1] + (fieldWidth / 2), 
            poseArray[2], 
            new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));
    }

    /**
    * Returns the robot pose as a Pose2d relative to the target space.
    * @return Pose2d
    */ 
    public Pose2d getBotPose2dTargetSpace() {
        double[] poseArray = botPoseTargetSpaceEntry.getDoubleArray(defaultValue);
        return new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[5]));
    }

    /**
    * Returns the robot pose as a Pose3d relative to the target space.
    * @return Pose3d
    */ 
    public Pose3d getBotPose3dTargetSpace() {
        double[] poseArray = botPoseTargetSpaceEntry.getDoubleArray(defaultValue);
        return new Pose3d(poseArray[0], poseArray[1], poseArray[2], 
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));
    }

    /**
    * Returns the camera pose as a Pose2d relative to the robot space.
    * @return Pose2d
    */ 
    public Pose2d getCameraPose2dRobotSpace() {
        double[] poseArray = camPoseRobotSpaceEntry.getDoubleArray(defaultValue);
        return new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[5]));
    }

    /**
    * Returns the camera pose as a Pose3d relative to the robot space.
    * @return Pose3d
    */ 
    public Pose3d getCameraPose3dRobotSpace() {
        double[] poseArray = camPoseRobotSpaceEntry.getDoubleArray(defaultValue);
        return new Pose3d(poseArray[0], poseArray[1], poseArray[2], 
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));
    }

    /**
    * Returns the camera pose as a Pose2d relative to the target space.
    * @return Pose2d
    */ 
    public Pose2d getCameraPose2dTargetSpace() {
        double[] poseArray = camPoseTargetSpaceEntry.getDoubleArray(defaultValue);
        return new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[5]));
    }

    /**
    * Returns the camera pose as a Pose3d relative to the target space.
    * @return Pose3d
    */ 
    public Pose3d getCameraPose3dTargetSpace() {
        double[] poseArray = camPoseTargetSpaceEntry.getDoubleArray(defaultValue);
        return new Pose3d(poseArray[0], poseArray[1], poseArray[2], 
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));
    } 

    /**
    * Returns the target pose as a Pose2d relative to the camera space.
    * @return Pose2d
    */ 
    public Pose2d getTargetPose2dCameraSpace() {
        double[] poseArray = targetPoseCameraSpaceEntry.getDoubleArray(defaultValue);
        return new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[5]));
    }

    /**
    * Returns the target pose as a Pose3d relative to the camera space.
    * @return Pose3d
    */ 
    public Pose3d getTargetPose3dCameraSpace() {
        double[] poseArray = targetPoseCameraSpaceEntry.getDoubleArray(defaultValue);
        return new Pose3d(poseArray[0], poseArray[1], poseArray[2], 
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));
    }

    /**
    * Returns the target pose as a Pose2d relative to the robot space.
    * @return Pose2d
    */ 
    public Pose2d getTargetPose2dRobotSpace() {
        double[] poseArray = targetPoseRobotSpaceEntry.getDoubleArray(defaultValue);
        return new Pose2d(poseArray[0], poseArray[1], new Rotation2d(poseArray[5]));
    }

    /**
    * Returns the target pose as a Pose3d relative to the robot space.
    * @return Pose3d
    */ 
    public Pose3d getTargetPose3dRobotSpace() {
        double[] poseArray = targetPoseRobotSpaceEntry.getDoubleArray(defaultValue);
        return new Pose3d(poseArray[0], poseArray[1], poseArray[2], 
                new Rotation3d(poseArray[3], poseArray[4], poseArray[5]));
    }
    
    public double[] getTc() {
        return tcEntry.getDoubleArray(defaultValue);
    }

    /**
    * Gets the total capture latency in milliseconds.
    * @return double
    */ 
    public double getTotalLatency() {
        return clEntry.getDouble(0.0);
    }

    public double getHb() {
        return hbEntry.getDouble(0.0);
    }

    /**
    * Gets the target area as a percent (0-100).
    * @return double
    */ 
    public double getTargetAreaPercent() {
        return taEntry.getDouble(0.0);
    }

    /**
    * Gets the horizonal length of the rough bounding box in pixels (0-320).
    * @return double
    */ 
    public double getHorizontalLength() {
        return thorEntry.getDouble(0.0);
    }

    /**
    * Gets the ID of the main tag in view.
    * @return double
    */ 
    public double getTagID() {
        return tidEntry.getDouble(0.0);
    }

    /**
    * Gets the total pipeline latency in milliseconds.
    * @return double
    */ 
    public double getPipelineLatency() {
        return tlEntry.getDouble(0.0);
    }

    /**
    * Gets the length of the longest side of the fitted bounding box in pixels.
    * @return double
    */ 
    public double getLongestSide() {
        return tlongEntry.getDouble(0.0);
    }
    
    /**
    * Gets the length of the shortest side of the fitted bounding box in pixels.
    * @return double
    */ 
    public double getShortestSide() {
        return tshortEntry.getDouble(0.0);
    }
      
    /**
    * Gets the vertical length of the rough bounding box in pixels (0-320).
    * @return double
    */ 
    public double getVerticalLength() {
        return tvertEntry.getDouble(0.0);
    }

    public double getTs() {
        return tsEntry.getDouble(0.0);
    }

    /**
    * Returns true if there is a target/April Tag in view.
    * If there isn't a target, this returns false.
    * @return boolean
    */ 
    public boolean hasValidTargets() {
        return this.tvEntry.getInteger(0) != 0;
    }
    
    /**
    * Horizontal offset from crosshair to target. (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
    * @return double
    */ 
    public double getTx() {
        return txEntry.getDouble(0.0);
    }
    
    /**
    * Vertical offset from crosshair to target. (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
    * @return double
    */ 
    public double getTy() {
        return tyEntry.getDouble(0.0);
    }

    enum LEDMode {
        USE_UI_SETTING,
        FORCE_OFF,
        FORCE_BLINK,
        FORCE_ON
    }

    public void setLedMode(LEDMode mode) {
        this.ledModeEntry.setNumber(mode.ordinal());
    }



} 
    

