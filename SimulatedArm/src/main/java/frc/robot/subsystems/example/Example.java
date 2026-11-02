package frc.robot.subsystems.example;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.visualization.mechanisms.SingleJointedArmMechanism2d;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;

public class Example extends GenericSubsystem {
    private SingleJointedArmMechanism2d mechanism;

    public Example() {
        mechanism = new SingleJointedArmMechanism2d("maayan", 3);
    }

    int n = 0;

    public Command doi() {
        return Commands.run(() -> n = (n + 1) % 135);
    }

    @Override
    public void periodic() {

        mechanism.updateCurrentAngle(Rotation2d.fromDegrees(n));
        // Log Pose3d objects for articulated components in AdvantageScope
        ArrayList<Pose3d> a = mechanism.armMechanism.generate3dMechanism();
        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = a.get(i);
        }
        Logger.recordOutput("Example/MechanismPoses", poses);
    }
}