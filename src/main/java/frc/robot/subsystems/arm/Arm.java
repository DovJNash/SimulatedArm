package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;


import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.arm.ArmConstants.*;
import static frc.robot.utilities.FieldConstants.hubPose;

public class Arm extends GenericSubsystem {
    public Command setTargetAngle(Rotation2d targetAngle) {
        return Commands.run(() -> ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetAngle.getRotations()), this);
    }

    public Command stopArm() {
        return Commands.runOnce(ARM_MOTOR::stopMotor, this);
    }

    // Fix: make it run in sequence but not using Commands.sequence
    public Command rotateArmMaxToMin() {
        return new SequentialCommandGroup(
                setTargetAngle(Rotation2d.fromDegrees(MAX_ANGLE.getDegrees())), setTargetAngle(Rotation2d.fromDegrees(MIN_ANGLE.getDegrees()))
        );
    }

    private double calcAngleToDot() {
        Pose2d robotPose = POSE_ESTIMATOR.getPose();
//        double dotPoseX = dotPose.getX(),
//                dotPoseY = dotPose.getY(),
//
//                armPoseX = robotPose.getX(),
//                armPoseY = robotPose.getY(),
//
//                deltaX = dotPoseX - armPoseX,
//                deltaY = dotPoseY - armPoseY,
//
//                target = Math.toDegrees(Math.atan2(deltaY, deltaX)),
//                current = ARM_MOTOR.getMotorPosition() * 360,
//                angle = Math.IEEEremainder(target - current, 360);
//        System.out.println(angle);
//        return angle + current;
        return 0;
    }

    // instead of simple lambda that I don't use, I just wrote setArmAngle here...
    public Command rotateArmToDot() {
        return Commands.run(() ->
                        ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, trackPosition(hubPose.getTranslation()).getRotations()),
                this
        );
    }

    @Override
    public void periodic() {
        if (ARM_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(
                    new Translation3d(0, 0, 0.5),
                    new Rotation3d(
                            0,
                            Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition()).getRadians(),
                            0)
            );

            Logger.recordOutput("Components/HoodPose", current3dPose);

            ARM_MECHANISM.updateCurrentAngle(getCurrentPosition());
            ARM_MECHANISM.updateTargetAngle(getTargetPosition());
        }
    }

    private Rotation2d trackPosition(Translation3d targetPosition) {
        final Pose2d currentPose = POSE_ESTIMATOR.getPose();
        final double dx = targetPosition.getX() - currentPose.getX();
        final double dz = targetPosition.getZ();
        return Rotation2d.fromRadians(Math.atan2(dz, dx));
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getSystemPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(ARM_MOTOR.getClosedLoopTarget());
    }
}
