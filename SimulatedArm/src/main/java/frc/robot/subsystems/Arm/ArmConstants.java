package frc.robot.subsystems.Arm;

import frc.lib.generic.hardware.motor.Motor;
import frc.lib.generic.hardware.motor.MotorFactory;

import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;
public class ArmConstants {

    protected static final Motor ARM_MOTOR = MotorFactory.createTalonFX("armMotor", 1);

    public static final SingleJointedArmMechanism2d ARM_MECHANISM = new SingleJointedArmMechanism2d("Arm", 67);

}
