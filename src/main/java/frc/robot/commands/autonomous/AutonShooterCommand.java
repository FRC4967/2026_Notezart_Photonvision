package frc.robot.commands.autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonShooterCommand extends Command {
    private SwerveSubsystem swerveSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private CompliantWheelsSubsystem compliantWheelsSubsystem;
    private double minSpeed;

    public AutonShooterCommand(AutonSubsystem autonSubsystem, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem, double minSpeed) {
        this.shooterSubsystem = shooterSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;
        this.minSpeed = minSpeed;

        addRequirements(autonSubsystem);
        addRequirements(shooterSubsystem);
        addRequirements(compliantWheelsSubsystem);
    }

    @Override
    public void execute() {
        // shooterSubsystem.setShooterPowerToDistance(0);
        // double distToSpeaker = swerveSubsystem.getDistanceFromBotToSpeaker();
        // shooterSubsystem.autonAngleToSpeaker(distToSpeaker);
        // shooterSubsystem.angleToTargetAngle();

        // double turnPercent = -swerveSubsystem.getPercentageToAngleToSpeaker();
        // double radiansPerSecond = swerveSubsystem.getMaxAngularSpeed() * turnPercent;

        // // /* aims the chassis to the speaker. */
        // // ChassisSpeeds aimChassisSpeeds = 
        // // ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, radiansPerSecond, swerveSubsystem.getGyroRotationRaw());
        // // SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(aimChassisSpeeds);
        // // swerveSubsystem.setAutonModuleStates(swerveModuleStates);

        // /* sends the note out once the shooter is angled and ramped up */
        // if(shooterSubsystem.isReadyToAutonShoot(Math.abs(swerveSubsystem.getPercentageToAngleToSpeaker()) <= .1)){
        // // if (shooterSubsystem.isReadyToAutonShoot(Math.abs(swerveSubsystem.getAngleDiferrenceToSpeaker()) < 1.2)) {
        //     compliantWheelsSubsystem.startCompliantWheels();
        // } 
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -swerveSubsystem.getPercentageToAngleToSpeaker(), swerveSubsystem.getGyroRotationRaw());
        SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setTeleopModuleStates(swerveModuleStates);
        
        double distToSpeaker = swerveSubsystem.getDistanceFromBotToSpeaker();
        shooterSubsystem.autonAngleToSpeaker(distToSpeaker);
        shooterSubsystem.angleToTargetAngle();
        shooterSubsystem.shootToWhiteLine();
        if(shooterSubsystem.isReadyToSpedShoot(minSpeed, Math.abs(swerveSubsystem.getPercentageToAngleToSpeaker()) < 0.05)){
            compliantWheelsSubsystem.startCompliantWheels();
        }
    }

    @Override
    public void end(boolean interrupted) {
        compliantWheelsSubsystem.stopCompliantWheels();
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        /* unfortunately the shooter shoots too fast for the line break to detect! */
        return !compliantWheelsSubsystem.isShooterLineBreakBroken();
    }
    
}
