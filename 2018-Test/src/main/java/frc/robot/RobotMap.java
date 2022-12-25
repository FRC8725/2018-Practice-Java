package frc.robot;

public class RobotMap {

    public class DriverPort{
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 5;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 7;
    
        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 6;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 8;
        
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 9;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 11;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 10;
        public static final int kBackRightDriveAbsoluteEncoderPort = 12;

    }

    public static final class Elavator {
        public static final int lowleftMotor = 13;
        public static final int lowrightMotor = 14;
        public static final int winchMotor = 15;
    }

    public static final class Gripper {
        public static final int leftSpinMotor = 16;
        public static final int rightSpinMotor = 17;
        public static final int angleMotor = 18;
    }

    public static final class Pneumatics {
        public static final int[] leftForklift = {0, 1};
        public static final int[] rightForklift = {2, 3};
        public static final int[] climber = {4, 5};
    }
}
