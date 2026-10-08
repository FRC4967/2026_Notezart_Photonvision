package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CompliantWheelsSubsystem;

public class DefaultCompliantWheelCommand extends Command {
   private CompliantWheelsSubsystem compliantWheelsSubsystem;
   
   private Supplier<Boolean> isSpinButtonPressed;
   private Supplier<Boolean> reverseWheelsButton;
   
   public DefaultCompliantWheelCommand(CompliantWheelsSubsystem compliantWheelsSubsystem, 
   Supplier<Boolean> spinWheelButton, Supplier<Boolean> reverseWheelsButton) {
      this.compliantWheelsSubsystem = compliantWheelsSubsystem;
      this.isSpinButtonPressed = spinWheelButton;
      this.reverseWheelsButton = reverseWheelsButton;

      addRequirements(compliantWheelsSubsystem);
   }

   public void execute() {
      if (isSpinButtonPressed.get()) {
         compliantWheelsSubsystem.startCompliantWheels();
      } else if (reverseWheelsButton.get()){
         compliantWheelsSubsystem.reverseCompliantWheels();
      } else {
         compliantWheelsSubsystem.stopCompliantWheels();
      }
   }  

   public boolean isFinished() {
      return false;
   }
}


