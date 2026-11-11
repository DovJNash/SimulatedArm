package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;


public class ArmConstants {
    protected static final Motor ARM_MOTOR = MotorFactory.createTalonFX("Osama", 1);
    public static final SingleJointedArmMechanism2d ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Arm", 2);

    static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(90);
    static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-90);

    static final Pose3d dotPose = new Pose3d(5.0, 5.0, 1.6, new Rotation3d());

    static {
        configureArmMotor();
    }

    private static void configureArmMotor() {
        final MotorConfiguration ArmMotorConfiguration = new MotorConfiguration();

        ArmMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        ArmMotorConfiguration.slot = new MotorProperties.Slot(0.145, 0, 0, 0.084973, 0, 0.13081, 0, Feedforward.Type.ARM);

        ArmMotorConfiguration.simulationSlot = new MotorProperties.Slot(80, 0, 1, 0, 0, 0);
        ArmMotorConfiguration.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.ARM,
                DCMotor.getFalcon500(1),
                1,
                0.5,
                0.01,
                Rotation2d.fromRotations(MIN_ANGLE.getRotations()),
                Rotation2d.fromRotations(MAX_ANGLE.getRotations()),
                true);

        ARM_MOTOR.configure(ArmMotorConfiguration);

        ARM_MOTOR.setMotorEncoderPosition(0);

        ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

    }
}