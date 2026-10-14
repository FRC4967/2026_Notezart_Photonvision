// package frc.robot.subsystems;

// import org.photonvision.PhotonCamera;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class PhotonVision extends SubsystemBase {
//     private PhotonCamera camera;
//     private PhotonPipelineResult result;
//     private PhotonTrackedTarget bestTarget; 

//     /** Instantiates the photonvision camera. */
//     public PhotonVision() {
//         camera = new PhotonCamera("noteCamera");
//     }

//     @Override
//     public void periodic() {
//         result = camera.getLatestResult();
//         bestTarget = result.getBestTarget();
//     }
    
//     /**
//      * returns the yaw distance from the detected target to the center of the camera view.
//      * @return the yaw value recieved.
//      */
//     public double getYaw() {
//         if (ifhasTargets()) {
//             bestTarget = result.getBestTarget();
//             System.out.println(bestTarget.getYaw());
//             SmartDashboard.putNumber("Yaw", bestTarget.getYaw());
//             return bestTarget.getYaw();
//         }
//         else {
//             return 0;
//         }
//     }
    
//     /**
//      * returns the pitch distance from the detected target to the center of the camera view.
//      * @return the pitch value recieved
//      */
//     public double getPitch() {
//         if (ifhasTargets()) {
//             return bestTarget.getPitch();
//         }
//         else {
//             return 0;
//         }
//     }

//     /**
//      * @return if the camera has a valid target detected.
//      */
//     public boolean ifhasTargets() {
//         SmartDashboard.putBoolean("has note target", result.hasTargets());
//         return result.hasTargets();
//     }

    
// }