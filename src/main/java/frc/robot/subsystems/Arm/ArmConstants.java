package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.Feedforward;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;


public class ArmConstants {
    protected static final Motor EXAMPLE_ARM_MOTOR = MotorFactory.createTalonFX("Example Motor", 1);

    private static final double EXAMPLE_ARM_MINIMUM_ROTATION = -0.5;
    private static final double EXAMPLE_ARM_MAXIMUM_ROTATION = 0.5;

    static {
        configureExampleArmMotorConfiguration();
    }

    private static void configureExampleArmMotorConfiguration() {
        final MotorConfiguration exampleArmMotorConfiguration = new MotorConfiguration();

        exampleArmMotorConfiguration.idleMode = MotorProperties.IdleMode.BRAKE;

        exampleArmMotorConfiguration.slot = new MotorProperties.Slot(0.145, 0, 0, 0.084973, 0, 0.13081, 0, Feedforward.Type.ARM);

        exampleArmMotorConfiguration.simulationSlot = new MotorProperties.Slot(80, 0, 1, 0, 0, 0);
        exampleArmMotorConfiguration.simulationProperties = new SimProperties.Slot(
                SimProperties.SimulationType.ARM,
                DCMotor.getFalcon500(1),
                1,
                0.5,
                0.01,
                Rotation2d.fromRotations(EXAMPLE_ARM_MINIMUM_ROTATION),
                Rotation2d.fromRotations(EXAMPLE_ARM_MAXIMUM_ROTATION),
                true);

        EXAMPLE_ARM_MOTOR.configure(exampleArmMotorConfiguration);

        EXAMPLE_ARM_MOTOR.setMotorEncoderPosition(0);

        EXAMPLE_ARM_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        EXAMPLE_ARM_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        EXAMPLE_ARM_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        EXAMPLE_ARM_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        EXAMPLE_ARM_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);

    }
}