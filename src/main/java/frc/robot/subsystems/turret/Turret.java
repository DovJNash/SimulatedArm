package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.hardware.motor.MotorProperties;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.robot.utilities.FieldConstants.hubPose;

public class Turret extends GenericSubsystem {

    public Command setTargetPos(Rotation2d angle) {
        return Commands.run(() -> TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION, angle.getRotations()), this);
    }

    public Command trackHub() {
        return Commands.run(() -> {
            double dy = hubPose.getY() - POSE_ESTIMATOR.getPose().getY();
            double dx = hubPose.getX() - POSE_ESTIMATOR.getPose().getX();

            Rotation2d angle = Rotation2d.fromRadians(Math.atan2(dy, dx));
            TURRET_MOTOR.setOutput(MotorProperties.ControlMode.POSITION,angle.getRotations());
        }, this);
    }

    public Rotation2d getCurrentPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getMotorPosition());
    }

    public Rotation2d getTargetPosition() {
        return Rotation2d.fromRotations(TURRET_MOTOR.getClosedLoopTarget());
    }


    @Override
    public void periodic() {
        if (TURRET_MECHANISM != null) {
            final Pose3d current3dPose = new Pose3d(
                    new Translation3d(0, 0, 0.5),
                    new Rotation3d(
                            0,
                            0,
                            Rotation2d.fromRotations(TURRET_MOTOR.getSystemPosition()).getRadians())
            );

            Logger.recordOutput("Components/TurretPose", current3dPose);

            TURRET_MECHANISM.updateCurrentAngle(getCurrentPosition());
            TURRET_MECHANISM.updateTargetAngle(getTargetPosition());
        }
    }
}
