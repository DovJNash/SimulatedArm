package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.revrobotics.util.StatusLogger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.generic.hardware.HardwareManager;
import frc.robot.subsystems.leds.Leds;
import org.littletonrobotics.junction.LoggedRobot;

import static frc.robot.RobotContainer.LEDS;
import static frc.robot.RobotContainer.POSE_ESTIMATOR;
import static frc.robot.utilities.PathingConstants.initializeBLine;

public class Robot extends LoggedRobot {
    private final CommandScheduler commandScheduler = CommandScheduler.getInstance();
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        SignalLogger.enableAutoLogging(false);
        StatusLogger.disableAutoLogging();

        initializeBLine();

        robotContainer = new RobotContainer();

        HardwareManager.initialize(this);
    }

    @Override
    public void robotPeriodic() {
        HardwareManager.update();
        commandScheduler.run();

        POSE_ESTIMATOR.periodic();
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null)
            commandScheduler.schedule(autonomousCommand);

        CommandScheduler.getInstance().schedule(LEDS.showFor(Leds.LEDMode.AUTO_START, 20));
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
            autonomousCommand = null;
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationPeriodic() {
        HardwareManager.updateSimulation();

        robotContainer.updateComponentPoses();
    }
}