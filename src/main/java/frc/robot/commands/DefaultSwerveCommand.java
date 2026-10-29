package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.SwerveSubsystem;

public class DefaultSwerveCommand extends Command{
    private SwerveSubsystem swerveSubsystem;
    // private PhotonVision photonVision;

    private Supplier<Double> xSupplier;
    private Supplier<Double> ySupplier;
    private Supplier<Double> zSupplier;
    private Supplier<Boolean> speakerAssistSupplier;
    private Supplier<Boolean> noteAssistSupplier;
    private Supplier<Boolean> ampAssistSupplier;
    private double xSpeed;
    private double ySpeed;
    private double zSpeed;

    private final PIDController yawController = new PIDController(20, 0, 0.5);
    private final double deadBand = 0.3;

    /**
     * 
     * @param swerveSubsystem imports the swerve subsystem.
     * @param xAxis the % value which increases the robot's position across the x axis of the field.
     * @param yAxis the % value which increases the robot's position across the y axis of the field.
     * @param zAxis the z axis on the controller
     * @param speakerAssistButton while held the chassis angle will automatically angle to the speaker based on odometry
     * @param noteAssistButton while held the chassis angle will automatically angle to the note detected by photonvision
     */
    public DefaultSwerveCommand(SwerveSubsystem swerveSubsystem,/* PhotonVision photonVision,*/ Supplier<Double> yAxis, 
    Supplier<Double> xAxis, Supplier<Double> zAxis, Supplier<Boolean> speakerAssistButton, Supplier<Boolean> noteAssistButton, Supplier<Boolean> ampAssistButton) {
        this.swerveSubsystem = swerveSubsystem;
        // this.photonVision = photonVision;
        this.xSupplier = xAxis;
        this.ySupplier = yAxis;
        this.zSupplier = zAxis;
        this.speakerAssistSupplier = speakerAssistButton;
        this.noteAssistSupplier = noteAssistButton;
        this.ampAssistSupplier = ampAssistButton;

        addRequirements(swerveSubsystem);
    }
    
    @Override
    public void execute() {

        ySpeed = MathUtil.applyDeadband(ySupplier.get(), deadBand);
        xSpeed = MathUtil.applyDeadband(xSupplier.get(), deadBand);

        SmartDashboard.putNumber("JS Axis0 (X/strafe)", xSupplier.get());
        SmartDashboard.putNumber("JS Axis1 (Y/forward)", ySupplier.get());
        SmartDashboard.putNumber("JS Axis2 (Z/rotate)", zSupplier.get());

        // if (speakerAssistSupplier.get()) {
        //     zSpeed = swerveSubsystem.getPercentageToAngleToSpeaker();
        // } else if (noteAssistSupplier.get() && photonVision.ifhasTargets()) {
        //     zSpeed = MathUtil.clamp(photonVision.getYaw() * swerveSubsystem.getBusVolts() / 60, -swerveSubsystem.getBusVolts(), swerveSubsystem.getBusVolts()) / swerveSubsystem.getBusVolts();
        // } else if (ampAssistSupplier.get()){
        //     zSpeed = MathUtil.clamp(yawController.calculate(swerveSubsystem.getGyroRad(), Math.toRadians(-90)), -swerveSubsystem.getBusVolts(), swerveSubsystem.getBusVolts()) / swerveSubsystem.getBusVolts();
        // } else {
            zSpeed = MathUtil.applyDeadband(zSupplier.get(), deadBand);
        // }


        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(-ySpeed/5, -xSpeed/5, zSpeed/5, swerveSubsystem.getGyroRotationRaw());
        SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setTeleopModuleStates(swerveModuleStates);
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
