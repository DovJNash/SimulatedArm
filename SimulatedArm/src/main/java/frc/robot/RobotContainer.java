package frc.robot;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.Questionnaire;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.poseestimation.camera.Camera;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;

import java.util.concurrent.atomic.AtomicInteger;

import static edu.wpi.first.wpilibj.RobotController.getBatteryVoltage;

public class RobotContainer {
    public static final BuiltInAccelerometer ACCELEROMETER = new BuiltInAccelerometer();

    public static final PoseEstimator POSE_ESTIMATOR = new PoseEstimator(
            new Camera[]{},
            null
    );

    public static final Swerve SWERVE = new Swerve();
    public static final Leds LEDS = new Leds();
    public static final Questionnaire QUESTIONNAIRE = new Questionnaire();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);

        Flippable.init();
        setupLEDsForBattery();
      
        ButtonControls.initializeButtons(ButtonControls.ButtonLayout.TELEOP);
    }

    public Command getAutonomousCommand() {
        return QUESTIONNAIRE.getCommand();
    }

    public String getAutoName() {
        return QUESTIONNAIRE.getSelected();
    }

    private void setupLEDsForBattery() {
        final int BATTERY_LOW_THRESHOLD = 150;
        final AtomicInteger lowBatteryCounter = new AtomicInteger(0);

        final Trigger batteryLowTrigger = new Trigger(() -> {
            if (getBatteryVoltage() < 11.7) lowBatteryCounter.set(lowBatteryCounter.get() + 1);
            else lowBatteryCounter.set(0);

            return BATTERY_LOW_THRESHOLD < lowBatteryCounter.get();
        });

        batteryLowTrigger.onTrue(LEDS.showFor(Leds.LEDMode.BATTERY_LOW, 5));
    }

    public void updateComponentPoses() {

    }
}