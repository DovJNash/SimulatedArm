package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorFactory;

import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;
import frc.robot.RobotContainer;

public class ArmConstants {

    protected static final Motor ARM_MOTOR = MotorFactory.createTalonFX("ArmMotor", 1);
    public static final SingleJointedArmMechanism2d ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Arm", 5);

    protected final static double MAX_ANGLE = 90;
    protected final static double MIN_ANGLE = -90;

    protected final static Pose3d[] DotPose = new Pose3d[]{new Pose3d(0, 4, 5, new Rotation3d())};
        protected static Pose2d[] RobotPose = new Pose2d[]{RobotContainer.POSE_ESTIMATOR.getPose()};



}
