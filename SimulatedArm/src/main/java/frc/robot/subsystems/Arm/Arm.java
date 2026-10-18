package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.generic.GenericSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

public class Arm extends GenericSubsystem {

    public void SetTargetAngle(Rotation2d Angle) {
        ArmConstants.ARM_MECHANISM.updateCurrentAngle(Angle);
    }

    public void StopArm() {
        ArmConstants.ARM_MOTOR.stopMotor();
    }

    public void ResetArmEncoder() {
        ArmConstants.ARM_MOTOR.setMotorEncoderPosition(0);
    }

    public Command RotateArmMaxToMin() {

        Command RotateArmToMax = new RunCommand(() -> {
            ArmConstants.ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(ArmConstants.MAX_ANGLE));
        }, this);

        Command RotateArmToMin = new RunCommand(() -> {
            ArmConstants.ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(ArmConstants.MIN_ANGLE));
        }, this);

        return RotateArmToMax.andThen(RotateArmToMin);
    }

    public Command RotateArmToDot() {
        double DotPose_X = ArmConstants.DotPose[0].getX();
        double DotPose_Y = ArmConstants.DotPose[0].getY();

        double ArmPose_X = ArmConstants.RobotPose[0].getX();
        double ArmPose_Y = ArmConstants.RobotPose[0].getY();

        double deltaX = DotPose_X - ArmPose_X;
        double deltaY = DotPose_Y - ArmPose_Y;
        double targetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        return new RunCommand(() -> {
            ArmConstants.ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(targetAngle));
        }, this);

    }


    @Override
    public void periodic() {

        ArmConstants.ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(ArmConstants.ARM_MOTOR.getMotorPosition()));
        ArrayList<Pose3d> a = ArmConstants.ARM_MECHANISM.armMechanism.generate3dMechanism();
        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = a.get(i);
        }
        Logger.recordOutput("Example/MechanismPoses", poses);


    }
}
