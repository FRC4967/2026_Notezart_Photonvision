package frc.robot.commands.autonomous;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonAngleToSpeakerCommand extends Command {
    SwerveSubsystem swerveSubsystem;
    
    Supplier<Boolean> ifHasNote;
    double type;

    public AutonAngleToSpeakerCommand(SwerveSubsystem swerveSubsystem, Supplier<Boolean> ifHasNote, double type) {
        this.swerveSubsystem = swerveSubsystem;
        this.ifHasNote = ifHasNote;
        this.type = type;
        
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        /* aims the chassis to the speaker. */

        // double turnPercent = -swerveSubsystem.getPercentageToAngleToSpeaker();
        // double radiansPerSecond = swerveSubsystem.getMaxAngularSpeed() * turnPercent;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, -swerveSubsystem.getPercentageToAngleToSpeaker(), swerveSubsystem.getGyroRotationRaw());
        SwerveModuleState[] swerveModuleStates = swerveSubsystem.getKinematics().toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setTeleopModuleStates(swerveModuleStates);
    }

    @Override
    public boolean isFinished() {
        /* in the case that shooter happens to be in the middle and pick up the note. */
        if (type != 1) {
            return !ifHasNote.get();
        } else {
            return ifHasNote.get() && !(swerveSubsystem.isInAutonShootingRange());
        }
    }
}