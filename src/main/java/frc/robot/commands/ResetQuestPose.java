package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class ResetQuestPose extends InstantCommand {

  private SwerveSubsystem swerveSubsystem;

  public ResetQuestPose(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
    swerveSubsystem.resetQuestEstimate(swerveSubsystem.getPose());
  }
}
