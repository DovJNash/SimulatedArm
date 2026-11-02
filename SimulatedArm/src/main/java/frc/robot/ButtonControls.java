package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.generic.GenericSubsystem;
import frc.lib.generic.characterization.WheelRadiusCharacterization;
import frc.lib.generic.hardware.controllers.Controller;
import frc.lib.generic.hardware.controllers.KeyboardController;
import frc.lib.util.flippable.Flippable;
import frc.robot.commands.pathfinding.BLineTuner;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utilities.PathingConstants;

import java.util.function.DoubleSupplier;

import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_X;
import static frc.lib.generic.hardware.controllers.Controller.Axis.LEFT_Y;
import static frc.lib.generic.visualization.DrawUtils.TWO_PI;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.RobotContainer.SWERVE;
import static frc.robot.subsystems.swerve.SwerveCommands.rotateToTarget;
import static frc.robot.utilities.PathingConstants.ROBOT_CONFIG;

public class ButtonControls {
    public enum ButtonLayout {
        DEVELOPMENT,
        TELEOP,
        CHARACTERIZE_SWERVE_DRIVE_MOTORS,
        CHARACTERIZE_WHEEL_RADIUS,
        CHARACTERIZE_SWERVE_AZIMUTH,
        TUNE_BLINE,
        SYSTEM_TEST
    }


    private static final Controller DRIVER_CONTROLLER = new Controller(0);
    private static final KeyboardController OPERATOR_CONTROLLER = new KeyboardController();

    public static final DoubleSupplier DRIVE_SIGN = () -> Flippable.isRedAlliance() ? 1 : -1;

    private static final DoubleSupplier X_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_Y) * 0.5;
    private static final DoubleSupplier Y_SUPPLIER = () -> DRIVE_SIGN.getAsDouble() * DRIVER_CONTROLLER.getRawAxis(LEFT_X) * 0.5;
    private static final DoubleSupplier OMEGA_SUPPLIER = () -> -DRIVER_CONTROLLER.getRawAxis(Controller.Axis.RIGHT_X) * 10 * 0.5;

    private static final Trigger USER_BUTTON = new Trigger(RobotController::getUserButton);

    public static void initializeButtons(ButtonLayout layout) {
        switch (layout) {
            case TELEOP -> configureButtonsTeleop();
            case DEVELOPMENT -> configureButtonsDevelopment();
            case CHARACTERIZE_WHEEL_RADIUS -> configureButtonsCharacterizeWheelRadius();
            case CHARACTERIZE_SWERVE_DRIVE_MOTORS -> setupDriveMotorsCharacterization();
            case CHARACTERIZE_SWERVE_AZIMUTH -> setupAzimuthCharacterization();
            case TUNE_BLINE -> configureBLineTuning();
            case SYSTEM_TEST -> testSystems();
        }
    }

    private static void testSystems() {
        setupDriving();
    }

    private static void configureBLineTuning() {
        setupDriving();

        final BLineTuner tuner = new BLineTuner(
                PathingConstants.BLINE_TRANSLATION_PID,
                PathingConstants.BLINE_ROTATION_PID,
                PathingConstants.BLINE_CROSS_TRACK_PID
        );

        tuner.configureController(DRIVER_CONTROLLER);
    }

    private static void configureButtonsDevelopment() {
    }

    private static void configureButtonsTeleop() {
        setupDriving();

        setupOperatorKeyboardButtons();
        setupTeleopLEDs();
    }

    private static void setupOperatorKeyboardButtons() {

    }

    private static void configureButtonsCharacterizeWheelRadius() {
        setupDriving();

        final Command wheelRadiusCharacterization = new WheelRadiusCharacterization(
                SWERVE,
                ROBOT_CONFIG.moduleLocations,
                SWERVE::getDriveWheelPositionsRadians,
                () -> SWERVE.getGyroHeading() * TWO_PI,
                (speed) -> SWERVE.driveRobotRelative(new ChassisSpeeds(0, 0, speed), true)
        );

        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue((wheelRadiusCharacterization));
    }

    private static void setupSysIdCharacterization(GenericSubsystem subsystem) {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(subsystem.getSysIdQuastatic(SysIdRoutine.Direction.kReverse));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kForward));
        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(subsystem.getSysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private static void setupTeleopLEDs() {
    }

    private static void setupDriveMotorsCharacterization() {
        setupDriving();
        setupSysIdCharacterization(SWERVE);
    }

    private static void setupAzimuthCharacterization() {
        DRIVER_CONTROLLER.getButton(Controller.Inputs.A).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(90))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.B).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(180))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.X).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(270))));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.Y).whileTrue(
                rotateToTarget(POSE_ESTIMATOR.getPose().rotateBy(Rotation2d.fromDegrees(360))));
    }

    private static void setupDriving() {
        SWERVE.setDefaultCommand(SwerveCommands.driveOpenLoop(
                X_SUPPLIER,
                Y_SUPPLIER,
                OMEGA_SUPPLIER,
                ()->false
        ));

        DRIVER_CONTROLLER.getButton(Controller.Inputs.START).whileTrue(SwerveCommands.resetGyro());
        DRIVER_CONTROLLER.getButton(Controller.Inputs.BACK).whileTrue(SwerveCommands.lockSwerve());
    }
}
