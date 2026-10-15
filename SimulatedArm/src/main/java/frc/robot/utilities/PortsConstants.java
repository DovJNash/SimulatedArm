package frc.robot.utilities;

/**
 * Class containing constants for ports on the robot.
 * This is useful for keeping track of which ports are used, so no port is used twice.
 */
public class PortsConstants {
    public static int LEDSTRIP_PORT_PWM = 0;

    public static class SwervePorts {
        public static final int FR_DRIVE_MOTOR_PORT = 1;
        public static final int FR_STEER_MOTOR_PORT = 31;
        public static final int FR_STEER_ENCODER_PORT = 1;

        public static final int FL_DRIVE_MOTOR_PORT = 22;
        public static final int FL_STEER_MOTOR_PORT = 32;
        public static final int FL_STEER_ENCODER_PORT = 22;

        public static final int RL_DRIVE_MOTOR_PORT = 14;
        public static final int RL_STEER_MOTOR_PORT = 44;
        public static final int RL_STEER_ENCODER_PORT = 4;

        public static final int RR_DRIVE_MOTOR_PORT = 13;
        public static final int RR_STEER_MOTOR_PORT = 33;
        public static final int RR_STEER_ENCODER_PORT = 33;

        public static final int GYRO_PORT = 30;
    }
}