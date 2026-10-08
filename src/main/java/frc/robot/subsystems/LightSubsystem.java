package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimeLight.LEDMode;

public class LightSubsystem extends SubsystemBase {
     private LimeLight limeLight = new LimeLight();

     public enum LightShow {
          NONE,
          COLOR,
          ALLIANCE_COLOR
     }

     private CAN lightController = new CAN(Constants.LED_CAN_ID);

     /** sets the lights to the alliance color */
     public void setAllianceColor() {
          lightController.writePacket(new byte[3], LightShow.ALLIANCE_COLOR.ordinal());
     }

     public void blinkLimeLight() {
          limeLight.setLedMode(LEDMode.FORCE_BLINK);
     }

     public void stopLimeLight(){
          limeLight.setLedMode(LEDMode.FORCE_OFF);
     }

     /**
      * sets the lights to the color given (will appear as white if controller cannot achieve that color.)
      * @param color the color to set to
      */
     public void setLEDColor(Color color) {
          this.setLEDColor(color, false);
     }

     /**
      * don't try to use this. It breaks the lights lol
      * @param color the color to set the lights to 
      * @param smoothTrans whether the lights will smoothly transition between each other
      */
     public void setLEDColor(Color color, boolean smoothTrans) {

          if(smoothTrans == false) {
               byte[] byteColor = new byte[] {
               (byte)(color.red * 255),
               (byte)(color.green * 255),
               (byte)(color.blue * 255)
               };

               try {
                    lightController.writePacket(byteColor, LightShow.COLOR.ordinal());
               }
               catch(Exception e) {
                    System.err.println("LED data transfer failed" + e);
               }
          }
          else {
               //TODO: use Sigmoid function for a smooth transition
          }
     }
     
}
