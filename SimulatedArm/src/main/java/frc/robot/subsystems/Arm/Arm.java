package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.generic.GenericSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

public class Arm extends GenericSubsystem {

    @Override
    public void periodic() {

        ArmConstants.ARM_MECHANISM.updateCurrentAngle(Rotation2d.fromDegrees(ArmConstants.ARM_MOTOR.getCurrent()));
        ArrayList<Pose3d> a = ArmConstants.ARM_MECHANISM.armMechanism.generate3dMechanism();
        Pose3d[] poses = new Pose3d[a.size()];
        for (int i = 0; i < a.size(); i++) {
            poses[i] = a.get(i);
        }
        Logger.recordOutput("Example/MechanismPoses", poses);


    }
}
