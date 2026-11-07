package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import frc.lib.generic.visualization.mechanisms.MechanismFactory;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

import static frc.robot.subsystems.Arm.ArmConstants.EXAMPLE_ARM_MOTOR;

public class Arm extends GenericSubsystem {
    public Command setArmPosition(double position) {
        return new FunctionalCommand(
                () -> {},
                () -> EXAMPLE_ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, position),
                interrupt -> EXAMPLE_ARM_MOTOR.stopMotor(),
                () -> false,
                this
        );
    }

    private Rotation2d getCurrentArmPosition() {
        return Rotation2d.fromRotations(EXAMPLE_ARM_MOTOR.getSystemPosition());
    }

    private Rotation2d getTargetArmPosition() {
        return Rotation2d.fromRotations(EXAMPLE_ARM_MOTOR.getClosedLoopTarget());
    }

    public Command stopExampleArm() {
        return Commands.runOnce(EXAMPLE_ARM_MOTOR::stopMotor, this);
    }

    SingleJointedArmMechanism2d mechanism = MechanismFactory.createSingleJointedArmMechanism("Osama", 5);

    @Override
    public void periodic() {

        mechanism.updateCurrentAngle(Rotation2d.fromDegrees(EXAMPLE_ARM_MOTOR.getMotorPosition()));
        // Log Pose3d objects for articulated components in AdvantageScope
        ArrayList<Pose3d> a = mechanism.armMechanism.generate3dMechanism();
        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = a.get(i);
        }
        Logger.recordOutput("Example/MechanismPoses", poses);
    }
}