package frc.robot.subsystems;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AprilometryTest {

    private Aprilometry aprilometry;
    private MockPoseSource aprilSource;
    private MockPoseSource odoSource;

    static class MockPoseSource implements BotPoseSource {

        Pose2d pose = new Pose2d();

        @Override
        public Pose2d getPose() {
            return pose;
        }

        public double getX() {
            if (pose == null) {
                return 0;
            }
            return pose.getX();
        }

        public double getY() {
            if (pose == null) {
                return 0;
            }
            return pose.getY();
        }

        @Override
        public void setPose(Pose2d newPose) {
            this.pose = newPose;
        }
    }

    @BeforeEach
    public void setup() {

        aprilSource = new MockPoseSource();
        odoSource = new MockPoseSource();

        // aprilometry = new Aprilometry(aprilSource, odoSource);
    }

    // @Test
    public void testOnlyOdometry() {

        aprilSource.pose = null;

        Pose2d[] odoPoses = new Pose2d[] {
                new Pose2d(6, 7, Rotation2d.fromDegrees(8)),
                new Pose2d(6, 7, Rotation2d.fromDegrees(8)),
                new Pose2d(6, 7, Rotation2d.fromDegrees(8))
        };

        for (Pose2d pose2d : odoPoses) {
            odoSource.pose = pose2d;
            aprilometry.periodic();
        }
        Pose2d result = aprilometry.getPose();
        // Assertions.assertEquals(result.getX(), 6);
    }

    // @Test
    public void testAprilTagSandwich() {
        Pose2d[] aprilPoses = new Pose2d[] {
                new Pose2d(1, 5.219, Rotation2d.fromDegrees(90)),
                new Pose2d(1.345, 6, Rotation2d.fromDegrees(90)),
                new Pose2d(1.25, 5.7777, Rotation2d.fromDegrees(100)),
                null,
                null,
                null,
                new Pose2d(2, 8, Rotation2d.fromDegrees(200)),
                new Pose2d(1.96, 7.93, Rotation2d.fromDegrees(200)),
                new Pose2d(2.1111, 7.5, Rotation2d.fromDegrees(270)),
                new Pose2d(2.026, 6, Rotation2d.fromDegrees(200)),
                new Pose2d(1.884, 7.443, Rotation2d.fromDegrees(180))
        };

        Pose2d[] odoPoses = new Pose2d[] {
                new Pose2d(2.3333, 6.0004, Rotation2d.fromDegrees(80)),
                new Pose2d(2.468, 6.123, Rotation2d.fromDegrees(89)),
                new Pose2d(3, 6.4103, Rotation2d.fromDegrees(81)),
                new Pose2d(2.226, 6.7, Rotation2d.fromDegrees(180)),
                new Pose2d(3.1, 7, Rotation2d.fromDegrees(100)),
                new Pose2d(2.987, 6.991, Rotation2d.fromDegrees(90)),
                new Pose2d(2.7, 7.00001, Rotation2d.fromDegrees(85)),
                new Pose2d(3.33, 7.26, Rotation2d.fromDegrees(82)),
                new Pose2d(2.9983, 7.411, Rotation2d.fromDegrees(70)),
                new Pose2d(2.87, 7.3, Rotation2d.fromDegrees(74)),
                new Pose2d(2.735, 7.36, Rotation2d.fromDegrees(67))
        };

        for (int index = 0; index < aprilPoses.length; index++) {
            odoSource.pose = odoPoses[index];
            aprilSource.pose = aprilPoses[index];
            aprilometry.periodic();
            Pose2d newPose = aprilometry.getPose();
            System.out.println(String.format("%f,%f,%f,%f,%f,%f",
                    odoSource.getX(),
                    odoSource.getY(),
                    aprilSource.getX(),
                    aprilSource.getY(),
                    newPose.getX(),
                    newPose.getY()));
        }

        Pose2d result = aprilometry.getPose();
        // Assertions.assertEquals(result.getX(), 6);
    }

}
