package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;

import frc.lib.generic.simulation.SimProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;
import static frc.lib.generic.simulation.SimProperties.SimulationType.SIMPLE_MOTOR;

public class ArmConstants {

    protected static final Motor ARM_MOTOR = MotorFactory.createTalonFX("ArmMotor", 1);
    public static final SingleJointedArmMechanism2d ARM_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Arm", 5);

    protected final static Rotation2d
        MAX_ANGLE = Rotation2d.fromDegrees(90),
        MIN_ANGLE = Rotation2d.fromDegrees(-90);

    protected final static Pose3d[] dotPose = new Pose3d[]{new Pose3d(0, 4, 5, new Rotation3d())};


    private static void armMotorConfig() {
        final MotorConfiguration config = new MotorConfiguration();
        config.idleMode = MotorProperties.IdleMode.BRAKE;
        config.inverted = false;

        config.gearRatio = 67.69; // Example
        config.profileMaxVelocity = 5;
        config.profileMaxAcceleration = 2;

        config.forwardSoftLimit = MAX_ANGLE.getDegrees();
        config.reverseSoftLimit = MIN_ANGLE.getDegrees();

        config.simulationSlot = new MotorProperties.Slot(2, 4, 0.5, 0.1, 0.01, 0.001);
        config.simulationProperties = new SimProperties.Slot(SIMPLE_MOTOR, DCMotor.getKrakenX60(1), 0.01, 0.001);

        ARM_MOTOR.configure(config);

        ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.TEMPERATURE);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        ARM_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);

    }


}
