package frc.robot.utilities;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.function.Predicate;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.FieldConstants.HALF_FIELD_LENGTH;
import static frc.robot.utilities.FieldConstants.HALF_FIELD_WIDTH;
import static java.lang.Math.abs;

public class ZoneUtilities {
    private static final double LOOKAHEAD_TIME_SECONDS = 0.01;

    private static boolean checkZone(Debouncer debouncer, Predicate<Translation2d> zone) {
        Translation2d current = POSE_ESTIMATOR.getPose().getTranslation();
        Translation2d future = POSE_ESTIMATOR.predictFuturePose(LOOKAHEAD_TIME_SECONDS).getTranslation();
        Translation2d mid = current.interpolate(future, 0.5);

        return debouncer.calculate(zone.test(current) || zone.test(mid) || zone.test(future));
    }

    private static Translation2d fold(Translation2d pose) {
        return new Translation2d(
                HALF_FIELD_LENGTH - abs(pose.getX() - HALF_FIELD_LENGTH),
                HALF_FIELD_WIDTH - abs(pose.getY() - HALF_FIELD_WIDTH)
        );
    }
}