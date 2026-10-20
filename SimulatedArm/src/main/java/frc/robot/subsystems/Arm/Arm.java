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
import static frc.robot.subsystems.Arm.ArmConstants.*;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;

public class Arm extends GenericSubsystem {

    public Command setTargetAngle(Rotation2d Angle) {
        return Commands.runOnce(() -> {
            ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, Angle.getRotations());
        }, this);
    }

    public Command stopArm() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    // TODO: Implement a method to reset actual position of the arm to 0 and THEN reset the encoder to 0.
    // "Give volts" to the arm until theres a mechanical stop, then reset the encoder to 0.
    public Command resetArmEncoder() {
        return Commands.runOnce(() -> {
            ARM_MOTOR.setOutput(MotorProperties.ControlMode.VOLTAGE, 0.5);

            if (ARM_MOTOR.getCurrent() > 5) {
                ARM_MOTOR.stopMotor();
                ARM_MOTOR.setMotorEncoderPosition(0);
            }
        }, this);
    }

    public Command rotateArmMaxToMin() {

        Command rotateArmToMax = Commands.run(() -> {
            ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, MAX_ANGLE.getRotations());
        }, this);

        Command rotateArmToMin = Commands.run(() -> {
            ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, MIN_ANGLE.getRotations());
        }, this);

        return rotateArmToMax.andThen(rotateArmToMin);
    }

    public Command rotateArmToDot() {
        Pose2d[] robotPose = new Pose2d[]{POSE_ESTIMATOR.getPose()};
        double dotPose_X = dotPose[0].getX(),
         dotPose_Y = dotPose[0].getY(),

         armPose_X = robotPose[0].getX(),
         armPose_Y = robotPose[0].getY(),

         deltaX = dotPose_X - armPose_X,
         deltaY = dotPose_Y - armPose_Y,

         targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return Commands.run(() -> {ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetAngle);}, this);

    }

    @Override
    public void periodic() {
        ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(ARM_MOTOR.getMotorPosition()));
        ArrayList<Pose3d> a = ARM_MECHANISM.armMechanism.generate3dMechanism();
        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = a.get(i);
        }
        Logger.recordOutput("Example/MechanismPoses", poses);
    }
}
