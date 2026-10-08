package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.CompliantWheelsSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonChassisCommand extends Command {
    AutonSubsystem autonSubsystem;
    SwerveSubsystem swerveSubsystem;
    CompliantWheelsSubsystem compliantWheelsSubsystem;
    private int currentPositionIndex;
    private List<Pose2d> poseList;
    private Command currentDriveCommand;

    boolean hasNotIntaked = true;

    SequentialCommandGroup test;
 
    public AutonChassisCommand(AutonSubsystem autonSubsystem, SwerveSubsystem swerveSubsystem, CompliantWheelsSubsystem compliantWheelsSubsystem) {
        this.autonSubsystem = autonSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.compliantWheelsSubsystem = compliantWheelsSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        poseList = new ArrayList<>();
        currentPositionIndex = 0;
        autonSubsystem.updateShuffleboardValues();
        poseList.add(autonSubsystem.getInitialPose());

        List<Pose2d> noteOrderRoutine = autonSubsystem.getNoteOrderRoutine();
        for (Pose2d posI : noteOrderRoutine) {
            poseList.add(posI);
        }        

        updateCurrentDriveCommand(0);
    }

    @Override
    public void execute() {
        currentDriveCommand.execute();

        if (currentDriveCommand.isFinished() || isShtewpid()) {
            currentDriveCommand.end(false);
            if (needsToIntakeWait() && hasNotIntaked) {
                hasNotIntaked = false;
                System.out.println("Chassis command called");
                currentDriveCommand = autonSubsystem.waitToIntakeCommand();
                currentDriveCommand.initialize();
            } else {
                hasNotIntaked = true;
                updateCurrentDriveCommand(1);
            }
        }
    }

    /**
     * updates the current trajectory command accordingly.
     * @param increment the trajectory to go to.
     */
    private final void updateCurrentDriveCommand(int increment) {
        currentPositionIndex = currentPositionIndex + increment;

        if (hasMovesLeft() && (!isNextTrajectoryEqual())) {
            /* checks whether it's going to intake or not, and adds time to intake accordingly. */
            currentDriveCommand = autonSubsystem.AdaptiveTrajectoryCommand(poseList.get(currentPositionIndex), poseList.get(currentPositionIndex + 1)); 
            currentDriveCommand.initialize();
        } else if (!hasMovesLeft()) {
            /* just angles towards speaker if there's nowhere left to move. 
             * There IS an extra caveat in the case that the robot's last note is in the middle,
             * doesn't grab the note in time, but somehow picks the note up during the final angling. 
             * Hence there is an extra andThen to move to the shoot position again anyways.
            */
            currentDriveCommand = autonSubsystem.AngleToSpeakerCommand(1).andThen(autonSubsystem.AdaptiveTrajectoryCommand(swerveSubsystem.getPose(), poseList.get(poseList.size() - 1)));
            currentDriveCommand.initialize();
        } else {
            /* runs again but skips the pos that's equal. */
            updateCurrentDriveCommand(1);
        }
    }

    /* i made a bunch of boolean calls because the if statements look like a mess if they're direct. */

    /**
     * @return if the trajectory is equal
     */
    private final boolean isNextTrajectoryEqual() {
        return (poseList.get(currentPositionIndex).getTranslation().equals(poseList.get(currentPositionIndex + 1).getTranslation()));
    }
    
    /**
     * @return if there are any trajectories left to make
     */
    private final boolean hasMovesLeft() {
        return (currentPositionIndex < (poseList.size() - 1));
    }

    /**
     * @return if the robot is going to shoot but doesn't have a note
     */
    private final boolean isShtewpid() {
        return ((!isIntaking()) && (!compliantWheelsSubsystem.isShooterLineBreakBroken()));
    }

    /**
     * @return if the current trajectory is going to a note or shooting.
     */
    private final boolean isIntaking() {
        return (currentPositionIndex % 2 == 0);
    }

    /**
     * @return if there needs to be an intake wait command.
     */
    private final boolean needsToIntakeWait() {
        return (autonSubsystem.getNotesShotCount() < (currentPositionIndex / 2)) && isIntaking();
    }
}
