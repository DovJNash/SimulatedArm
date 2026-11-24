package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.generic.hardware.motor.*;
import frc.lib.generic.simulation.SimProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;

public class TurretConstants {

    public static final Motor TURRET_MOTOR = MotorFactory.createTalonFX("Turret Motor", 2);
    public static final SingleJointedArmMechanism2d TURRET_MECHANISM = MechanismFactory.createSingleJointedArmMechanism("Turret Mechanism", 1);

    static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(270);
    static final Rotation2d MIN_ANGLE = Rotation2d.fromDegrees(-270);

    static {
        turretMotorConfig();
    }

    private static void turretMotorConfig() {
        MotorConfiguration turretConfig = new MotorConfiguration();

        turretConfig.idleMode = MotorProperties.IdleMode.BRAKE;
        turretConfig.slot = new MotorProperties.Slot(1, 0, 0);

        turretConfig.simulationProperties = new SimProperties.Slot(SimProperties.SimulationType.ARM,
                DCMotor.getKrakenX60(1),
                10,
                2,
                0.001,
                MIN_ANGLE,
                MAX_ANGLE,
                true
                );
        TURRET_MOTOR.setMotorEncoderPosition(0);
        TURRET_MOTOR.configure(turretConfig);

        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VOLTAGE);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CURRENT);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.CLOSED_LOOP_TARGET);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.POSITION);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.VELOCITY);
        TURRET_MOTOR.setupSignalUpdates(MotorSignal.ACCELERATION);
    }
}
