package frc.robot.commands.autonomous;

import java.util.List;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GyroResetCommand;
import frc.robot.commands.OdometryResetCommand;
import frc.robot.subsystems.Aprilometry;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonTrajectoryTestCommand extends Command {
    private AutonSubsystem autonSubsystem;
    private SwerveSubsystem swerveSubsystem;
    private Aprilometry aprilometry;

    private SequentialCommandGroup sequence;

    private final Pose2d startPos = new Pose2d(1, 1, new Rotation2d(0));

    private final TrajectoryConfig testingConfig = new TrajectoryConfig(4, 2.5);

    public AutonTrajectoryTestCommand(AutonSubsystem autonSubsystem, SwerveSubsystem swerveSubsystem, Aprilometry aprilometry) {
        this.autonSubsystem = autonSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.aprilometry = aprilometry;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        sequence = new SequentialCommandGroup();
        
        Pose2d endPos = new Pose2d(
            SmartDashboard.getNumber("X meters to test", 1) + 3,
            SmartDashboard.getNumber("Y meters to test", 0) + 1, 
            Rotation2d.fromDegrees(SmartDashboard.getNumber("degree rotations to test", 0))
        );

        List<Pose2d> poseList = List.of(startPos, endPos);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(poseList, testingConfig);

        sequence.addCommands(new OdometryResetCommand(aprilometry, swerveSubsystem, startPos));
        sequence.addCommands(new GyroResetCommand(swerveSubsystem));
        sequence.addCommands(autonSubsystem.TrajectoryIntoCommand(trajectory, true, endPos));
        
        sequence.initialize();
    }

    @Override
    public void execute() {
        sequence.execute();
    }

    @Override
    public boolean isFinished() {
        return sequence.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        sequence.end(interrupted);
    }

}

        // SequentialCommandGroup sequence = new SequentialCommandGroup();
        // startPosition = new Pose2d(1, 1, Rotation2d.fromDegrees(0));
        // Pose2d endPose = new Pose2d(
        //     XDist + 1, 
        //     YDist + 1, 
        //     Rotation2d.fromDegrees(degrees)
        // );

        // List<Pose2d> waypoints = List.of(startPosition, endPose);
        // Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

        // sequence.addCommands(new OdometryResetCommand(aprilometry, startPosition));
        // sequence.addCommands(new GyroResetCommand(swerveSubsystem));
        // sequence.addCommands(TrajectoryIntoCommand(trajectory, true, endPose));

        // return sequence;
