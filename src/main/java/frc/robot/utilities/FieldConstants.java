package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLogOutput;

public class FieldConstants {
    public static final double
            FIELD_WIDTH = 8.052, HALF_FIELD_WIDTH = FIELD_WIDTH / 2,
            FIELD_LENGTH = 16.541, HALF_FIELD_LENGTH = FIELD_LENGTH / 2;

    @AutoLogOutput
    public static final Pose3d hubPose = new Pose3d(5.0, 5.0, 1.6, new Rotation3d());

}