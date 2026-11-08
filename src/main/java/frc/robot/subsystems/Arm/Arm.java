package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;


import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.Arm.ArmConstants.*;

public class Arm extends GenericSubsystem {

    public Command setTargetAngle(double targetAngle) {
        return Commands.runOnce(() -> {
            ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetAngle / 360); // Convert degrees to rotations
        });
    }

    public Command stopArm() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    // "Give volts" to the arm until theres a mechanical stop, then reset the encoder to 0.
    public Command resetArmEncoder() {
        return Commands.runOnce(() -> {
            ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, -5);

            if (ARM_MOTOR.getCurrent() != -5) {
                ARM_MOTOR.stopMotor();
                ARM_MOTOR.setMotorEncoderPosition(0);
            }
        }, this);
    }

    public Command rotateArmMaxToMin() {
        return Commands.sequence(
                setTargetAngle(MAX_ANGLE.getDegrees()),
                setTargetAngle(MIN_ANGLE.getDegrees())
        );
    }

    public Command rotateArmToDot() {
        Pose2d robotPose = POSE_ESTIMATOR.getPose();
        double dotPoseX = dotPose.getX(),
         dotPoseY = dotPose.getY(),

         armPoseX = robotPose.getX(),
         armPoseY = robotPose.getY(),

         deltaX = dotPoseX - armPoseX,
         deltaY = dotPoseY - armPoseY,

         targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return setTargetAngle(targetAngle);

    }


    @Override
    public void periodic() {
        ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromRotations(ARM_MOTOR.getMotorPosition()));
        ArrayList<Pose3d> a = ARM_MECHANISM.armMechanism.generate3dMechanism();
        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = a.get(i);
        }
        Logger.recordOutput("OSAMA/MechanismPoses", poses);
    }
}
