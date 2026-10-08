package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.ButtonMap;
import frc.robot.commands.CompliantWheelsUntilBreakCommand;
import frc.robot.commands.DefaultClimberCommand;
import frc.robot.commands.DefaultCompliantWheelCommand;
import frc.robot.commands.DefaultIntakeCommand;
import frc.robot.commands.DefaultLightCommand;
import frc.robot.commands.ShooterModesCommand;
import frc.robot.commands.autonomous.AutonTrajectoryTestCommand;
import frc.robot.commands.DefaultSwerveCommand;
import frc.robot.commands.ForceShootCommand;
import frc.robot.commands.GyroResetCommand;
import frc.robot.commands.ManualShooterCommand;
import frc.robot.commands.ShootCommand;

import frc.robot.commands.OdometryResetCommand;

import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.Aprilometry;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.LightSubsystem;

public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
  private final Aprilometry aprilometry = new Aprilometry(aprilTagSubsystem, swerveSubsystem);
  private final TrajectoryWaypoints trajectoryWaypoints = new TrajectoryWaypoints();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final CompliantWheelsSubsystem compliantWheelsSubsystem = new CompliantWheelsSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final PhotonVision photonVision = new PhotonVision();
  private final AutonSubsystem autonSubsystem = new AutonSubsystem(swerveSubsystem, shooterSubsystem, intakeSubsystem, compliantWheelsSubsystem, /*photonVision,*/ null, aprilometry, trajectoryWaypoints);
  private final CommandJoystick driverJoystick = new CommandJoystick(ButtonMap.DRIVER_JOYSTICK_ID);
  private final CommandJoystick manipulatorPanel = new CommandJoystick(ButtonMap.MANIPULATOR_PANEL_ID);
  private final LightSubsystem lightSubsystem = new LightSubsystem();


  public RobotContainer() { 
    // Configure the trigger bindings

    swerveSubsystem.setDefaultCommand(new DefaultSwerveCommand(
    swerveSubsystem,
    photonVision,
    () -> driverJoystick.getRawAxis(ButtonMap.JOYSTICK_Y_AXIS),
    () -> driverJoystick.getRawAxis(ButtonMap.JOYSTICK_X_AXIS),
    () -> driverJoystick.getRawAxis(ButtonMap.JOYSTICK_Z_AXIS),
    () -> driverJoystick.getHID().getRawButton(ButtonMap.CHASSIS_SPEAKER_AIM_BUTTON),
    () -> driverJoystick.getHID().getRawButton(ButtonMap.CHASSIS_NOTE_AIM_BUTTON),
    () -> driverJoystick.getHID().getRawButton(ButtonMap.CHASSIS_AMP_AIM_BUTTON)
    ));

    shooterSubsystem.setDefaultCommand(new ShooterModesCommand(
    shooterSubsystem, 
    this::getDistanceToSpeaker, 
    this::ifHasNote,
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.AUTO_ANGLE_MODE),
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.SUBWOOFER_MODE),
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.AMP_MODE),
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.PASSING_MODE),
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.IDLE_MODE)
    ));
  
    intakeSubsystem.setDefaultCommand(new DefaultIntakeCommand(
    intakeSubsystem, 
    () -> driverJoystick.getHID().getRawButton(ButtonMap.INTAKE_BUTTON), 
    () -> driverJoystick.getHID().getRawButton(ButtonMap.FORCE_INTAKE_OUT_BUTTON)
    ));

    compliantWheelsSubsystem.setDefaultCommand(new DefaultCompliantWheelCommand(
    compliantWheelsSubsystem, 
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_COMPLIANT_WHEELS_IN_BUTTON),
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_COMPLIANT_WHEELS_OUT_BUTTON)
    ));

    climberSubsystem.setDefaultCommand(new DefaultClimberCommand(
    climberSubsystem, 
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.CLIMBER_UP_BUTTON),
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.CLIMBER_DOWN_BUTTON)
    ));
    
    lightSubsystem.setDefaultCommand(new DefaultLightCommand(
    lightSubsystem, 
    intakeSubsystem,
    compliantWheelsSubsystem,
    () -> manipulatorPanel.getHID().getRawButton(ButtonMap.SHOOT_BUTTON),
    () -> driverJoystick.getHID().getRawButton(ButtonMap.NO_SOURCE_LIGHTS)
    ));

    configureBindings();
  }

  private void configureBindings() {
    manipulatorPanel.button(ButtonMap.SHOOT_BUTTON).whileTrue(new ShootCommand(this::isReadyToShoot, compliantWheelsSubsystem));
 
    manipulatorPanel.button(ButtonMap.FORCE_SHOOT_BUTTON).onTrue(new ForceShootCommand(shooterSubsystem, compliantWheelsSubsystem));

    manipulatorPanel.button(ButtonMap.ANGULAR_UP_BUTTON).whileTrue(new ManualShooterCommand(
      shooterSubsystem, 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_OUT_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_IN_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_UP_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_DOWN_BUTTON)
    ));

    manipulatorPanel.button(ButtonMap.ANGULAR_DOWN_BUTTON).whileTrue(new ManualShooterCommand(
      shooterSubsystem, 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_OUT_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_IN_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_UP_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_DOWN_BUTTON)
    ));

    manipulatorPanel.button(ButtonMap.FORCE_SHOOTER_OUT_BUTTON).whileTrue(new ManualShooterCommand(
      shooterSubsystem, 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_OUT_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_IN_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_UP_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_DOWN_BUTTON)
    ));

    manipulatorPanel.button(ButtonMap.FORCE_SHOOTER_IN_BUTTON).whileTrue(new ManualShooterCommand(
      shooterSubsystem, 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_OUT_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_SHOOTER_IN_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_UP_BUTTON), 
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.ANGULAR_DOWN_BUTTON)
    ));

    manipulatorPanel.button(ButtonMap.COMPLIANT_WHEELS_BUTTON).toggleOnTrue(new CompliantWheelsUntilBreakCommand(
      compliantWheelsSubsystem,
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_COMPLIANT_WHEELS_IN_BUTTON),
      () -> manipulatorPanel.getHID().getRawButton(ButtonMap.FORCE_COMPLIANT_WHEELS_OUT_BUTTON)
    ));

    driverJoystick.button(9).onTrue(new AutonTrajectoryTestCommand(
      autonSubsystem, 
      swerveSubsystem,
      aprilometry
    ));

    driverJoystick.button(ButtonMap.RESET_GYRO_BUTTON).onTrue(new GyroResetCommand(swerveSubsystem));
    driverJoystick.button(ButtonMap.RESET_ODOMETRY_BUTTON).onTrue(new OdometryResetCommand(aprilometry, swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return autonSubsystem.getAutonCommand();
  
  }

  public void autonomousInit() {
    swerveSubsystem.setAutonCurrentLimits();
    swerveSubsystem.setPose(autonSubsystem.getInitialPose());
    shooterSubsystem.setAutoAngularPID();
    aprilometry.setOdometryCorrection(false);
  }

  public void teleopInit() {
    swerveSubsystem.setTeleopCurrentLimits();
    shooterSubsystem.setDefaultAngularPID();
    aprilometry.setOdometryCorrection(true);
  }

  private boolean ifHasNote() {
    return compliantWheelsSubsystem.isShooterLineBreakBroken();
  }
  private double getDistanceToSpeaker() {
    return swerveSubsystem.getDistanceFromBotToSpeaker();
  }

  private boolean isReadyToShoot() {
    return shooterSubsystem.isReadyToShoot();
  }

}

