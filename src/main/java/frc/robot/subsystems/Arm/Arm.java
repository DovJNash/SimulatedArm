package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.*;
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
            ARM_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, targetAngle / 360);
            stopArm();// Convert degrees to rotations
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

    private double calcAngleToDot(){
        Pose2d robotPose = POSE_ESTIMATOR.getPose();
        double dotPoseX = dotPose.getX(),
                dotPoseY = dotPose.getY(),

                armPoseX = robotPose.getX(),
                armPoseY = robotPose.getY(),

                deltaX = dotPoseX - armPoseX,
                deltaY = dotPoseY - armPoseY,

                target = Math.toDegrees(Math.atan2(deltaY, deltaX)),
                current = ARM_MOTOR.getMotorPosition() * 360,
                angle = Math.IEEEremainder(target - current, 360);
        System.out.println(angle);
        return angle + current;
    }

    // instead of simple lambda that I don't use, I just wrote setArmAngle here...
    public Command rotateArmToDot() {
        return Commands.run(
                () -> ARM_MOTOR.setOutput(
                        MotorProperties.ControlMode.POSITION,
                        calcAngleToDot() / 360
                ),
                this
        );
    }

    //TODO: delete armPos and add a component to the robot instead
    @Override
    public void periodic() {
        ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromRotations(ARM_MOTOR.getMotorPosition()));
        Pose3d armPos = new Pose3d(POSE_ESTIMATOR.getPose().getX(), POSE_ESTIMATOR.getPose().getY(), 5, new Rotation3d(0, 0, 0));

        ArrayList<Pose3d> a = ARM_MECHANISM.armMechanism.generate3dMechanism();

        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = armPos.transformBy(new Transform3d(a.get(0), a.get(1)));
        }
        Logger.recordOutput("OSAMA/MechanismPoses", poses);
        Logger.recordOutput("HUB/HubPose", dotPose);
    }
}
