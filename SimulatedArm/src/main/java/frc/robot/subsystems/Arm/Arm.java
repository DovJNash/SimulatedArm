package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

import static frc.robot.subsystems.Arm.ArmConstants.ARM_MOTOR;

public class Arm extends GenericSubsystem {

    public void SetTargetAngle(Rotation2d Angle) {
        ArmConstants.ARM_MECHANISM.updateCurrentAngle(Angle);
    }

    public Command StopArm() {
        return Commands.run(ARM_MOTOR::stopMotor);
    }

    // TODO: Implement a method to reset actual position of the arm to 0 and THEN reset the encoder to 0.
    public void ResetArmEncoder() {
        ARM_MOTOR.setMotorEncoderPosition(0);
    }

    public Command RotateArmMaxToMin() {

        Command RotateArmToMax = Commands.run(() -> {
            ArmConstants.ARM_MECHANISM.updateCurrentAngle(ArmConstants.MAX_ANGLE);
        }, this);

        Command RotateArmToMin = Commands.run(() -> {
            ArmConstants.ARM_MECHANISM.updateCurrentAngle(ArmConstants.MIN_ANGLE);
        }, this);

        return RotateArmToMax.andThen(RotateArmToMin);
    }

    public Command RotateArmToDot() {
        Pose2d[] robotPose = new Pose2d[]{RobotContainer.POSE_ESTIMATOR.getPose()};

        double DotPose_X = ArmConstants.DotPose[0].getX();
        double DotPose_Y = ArmConstants.DotPose[0].getY();

        double ArmPose_X = robotPose[0].getX();
        double ArmPose_Y = robotPose[0].getY();

        double deltaX = DotPose_X - ArmPose_X;
        double deltaY = DotPose_Y - ArmPose_Y;
        double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return Commands.run(() -> {
            ArmConstants.ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(targetAngle));
        }, this);

    }


    @Override
    public void periodic() {

        ArmConstants.ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(ARM_MOTOR.getMotorPosition()));
        ArrayList<Pose3d> a = ArmConstants.ARM_MECHANISM.armMechanism.generate3dMechanism();
        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = a.get(i);
        }
        Logger.recordOutput("Example/MechanismPoses", poses);


    }
}
