package frc.robot;

import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TrajectoryWaypoints {
    /** robot width from bumper to bumper divided by 2. */
    private static final double robotX = 0.46;
    /** robot lenght from bumper to bumper divided by 2. */
    private static final double robotY = 0.46;

    /** against the inside of the community zone line */
    private static final double redStartingX = 14.65 + robotX;
    private static final double redAgainstSpeakerX = 16.09;
    /** against the inside of the community zone line */
    private static final double blueStartingX = 1.91 - robotX;
    private static final double blueAgainstSpeakerX = 0.47;

    /** Start Position Right of speaker */
    private static final double lowerSpeakerY = 4.47 - robotY;
    private static final double lowerAgainstSpeakerY = 4.76;
    /** Start Position Left of speaker */
    private static final double upperSpeakerY = 6.58 + robotY;
    private static final double upperAgainstSpeakerY = 6.33;
    /** Start Position Center of speaker */
    private static final double centerSpeakerY = 5.55;
    /**  Start Position Left of (opponent) Source Area */
    private static final double squaredOpponentAreaY = 1.6 + robotY;

    private static final double blueInnerNoteX = 2.9;
    private static final double redInnerNoteX = 13.68;
    private static final double middleNoteX = 8.3;

    /* descending from the top of the field as depicted in game manual. */
    private static final double innerNoteOneY = 7;
    private static final double innerNoteTwoY = 5.55;
    private static final double innerNoteThreeY = 4.1;

    /** just a placeholder for zero */
    private static final Rotation2d potato = new Rotation2d(0);
    private static final Rotation2d rotatoRed = new Rotation2d(Math.PI);
    private static final Rotation2d upperAgainstSpeakerAngle = new Rotation2d(-60);
    private static final Rotation2d lowerAgainstSpeakerAngle = new Rotation2d(60);

    public interface AllianceConfig {
        /**
         * @return the pre-set starting positions for the autonomous.
         */
        public List<Pose2d> getStartingPositions();

        /**
         * @return the starting note positions on the field.
         */
        public List<Translation2d> getNotePositions();

        /**
         * @return the pre-set ending positions for the autonomous.
         */
        public List<Translation2d> getEndPositions();

        /**
         * @return the pose2d to avoid hitting the polw while grabbing evil note.
         */
        public Pose2d getPoleAvoidPoint();

        /**
         * A pose list for each shot position, indexed in the same order as the notes.
         * Shoot positions where we don't need to move from the note are the same as the note positions.
         * @return the list of shoot positions 
         */
        public List<Pose2d> getNoteShootPositions();
    }

    public static class RedAlliance implements AllianceConfig {
        private static List<Pose2d> startPositions = List.of(
            new Pose2d(3, 0, rotatoRed), /* position for just move forward. */
            new Pose2d(redStartingX, lowerSpeakerY, rotatoRed), /* left of speaker */
            new Pose2d(redStartingX, upperSpeakerY, rotatoRed), /* right of speaker */
            new Pose2d(redStartingX, centerSpeakerY, rotatoRed), /* center of speaker */
            new Pose2d(redStartingX, squaredOpponentAreaY, rotatoRed), /* squared to opponent pickup */
            new Pose2d(redAgainstSpeakerX, lowerAgainstSpeakerY, lowerAgainstSpeakerAngle), /* against the left side of the speaker */
            new Pose2d(redAgainstSpeakerX, upperAgainstSpeakerY, upperAgainstSpeakerAngle) /* against the right of the speaker*/
        );

        private static List<Translation2d> notePositions = List.of(
            new Translation2d(redInnerNoteX, innerNoteOneY),
            new Translation2d(redInnerNoteX, innerNoteTwoY),
            new Translation2d(redInnerNoteX, innerNoteThreeY),
            new Translation2d(middleNoteX, 7.425), // 7.425 7.525
            new Translation2d(middleNoteX, 5.77), // 5.77 5.87
            new Translation2d(middleNoteX, 4.1), // 4.1 4.2
            new Translation2d(middleNoteX, 2.43), // 2.43 2.63
            new Translation2d(middleNoteX, 0.78) // 0.78 0.8
        );

        private static List<Pose2d> noteShootPositions = List.of(
            new Pose2d(redInnerNoteX, innerNoteOneY, rotatoRed),
            new Pose2d(redInnerNoteX, innerNoteTwoY, rotatoRed),
            new Pose2d(redInnerNoteX + .8, innerNoteThreeY, rotatoRed),
            new Pose2d(11, 7.3, rotatoRed),
            new Pose2d(11, 6.5, rotatoRed),
            new Pose2d(12.27, 5.03, rotatoRed),
            new Pose2d(12.27, 5.03, rotatoRed),
            new Pose2d(13.11, 1.795, rotatoRed)
        );

        private static List<Translation2d> endPositions = List.of(
            /* integrated but we might not use endpositions at all. */
            new Translation2d(0,0), /* for just move out of zone */
            new Translation2d(redStartingX, centerSpeakerY), /* towards speaker */
            new Translation2d(middleNoteX, 4.1) /* towards source */
        );
        

        @Override
        public Pose2d getPoleAvoidPoint() {
            return new Pose2d(redInnerNoteX + robotX, innerNoteThreeY, rotatoRed);
        }

        @Override
        public List<Pose2d> getStartingPositions() {
            return startPositions;
        }

        @Override
        public List<Translation2d> getNotePositions() {
            return notePositions;
        }   

        @Override
        public List<Pose2d> getNoteShootPositions() {
            return noteShootPositions;
        }

        @Override
        public List<Translation2d> getEndPositions() {
            return endPositions;
        }

    }

    public static class BlueAlliance implements AllianceConfig {
        private static List<Pose2d> startPositions = List.of(
            new Pose2d(0, 0, potato), /* in case of just move forward */
            new Pose2d(blueStartingX, upperSpeakerY, potato), /* left of speaker */
            new Pose2d(blueStartingX, lowerSpeakerY, potato),/* right of speaker */
            new Pose2d(blueStartingX, centerSpeakerY, potato), /* center of speaker */
            new Pose2d(blueStartingX, squaredOpponentAreaY, potato), /* squared to opponent pickup */
            new Pose2d(blueAgainstSpeakerX, upperAgainstSpeakerY, upperAgainstSpeakerAngle), /* against the left side of the speaker */
            new Pose2d(blueAgainstSpeakerX, lowerAgainstSpeakerY, lowerAgainstSpeakerAngle) /* against the right of the speaker*/
        );

        private static List<Translation2d> notePositions = List.of(
            new Translation2d(blueInnerNoteX, innerNoteOneY),
            new Translation2d(blueInnerNoteX, innerNoteTwoY),
            new Translation2d(blueInnerNoteX, innerNoteThreeY),
            new Translation2d(middleNoteX, 7.425), // 7.425 7.525
            new Translation2d(middleNoteX, 5.77), // 5.77 5.87
            new Translation2d(middleNoteX, 4.1), // 4.1 4.2
            new Translation2d(middleNoteX, 2.43), // 2.43 2.63
            new Translation2d(middleNoteX, 0.78), // 0.78 0.8
            new Translation2d(blueInnerNoteX - robotX, innerNoteThreeY) /* position to aviod pole robotX is placeholder */
        );

        private static List<Pose2d> noteShootPositions = List.of(
            new Pose2d(blueInnerNoteX, innerNoteOneY, potato),
            new Pose2d(blueInnerNoteX, innerNoteTwoY, potato),
            new Pose2d(blueInnerNoteX - .8, innerNoteThreeY, potato),
            new Pose2d(5.5, 6.7, potato),
            new Pose2d(5.5, 6.7, potato),
            new Pose2d(4.2, 5.07, potato),
            new Pose2d(4.2, 5.07, potato),
            new Pose2d(3.95, 2.64, potato)
        );

        private static List<Translation2d> endPositions = List.of(
            /* its integrated but we still might not use endpositions at all. */
            new Translation2d(3, 0), /* just move out of zone */
            new Translation2d(blueStartingX, centerSpeakerY), /* towards speaker */
            new Translation2d(middleNoteX, 4.1) /* towards source */
        );

        @Override
        public Pose2d getPoleAvoidPoint() {
            return new Pose2d(blueInnerNoteX - robotX, innerNoteThreeY, potato);
        }

        @Override
        public List<Pose2d> getStartingPositions() {
            return startPositions;
        }

        @Override
        public List<Translation2d> getNotePositions() {
            return notePositions;
        }

        @Override
        public List<Pose2d> getNoteShootPositions() {
            return noteShootPositions;
        }

        @Override
        public List<Translation2d> getEndPositions() {
        return endPositions;
        }
    }
}