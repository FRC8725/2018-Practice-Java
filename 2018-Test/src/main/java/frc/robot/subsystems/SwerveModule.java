package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.lib.LazyTalonFX;
import frc.robot.RobotMap;

public class SwerveModule {

//    private final CANSparkMax driveMotor;
    private final LazyTalonFX driveMotor, turningMotor;
//    private final CANSparkMax driveMotor;
//    private final CANSparkMax turningMotor;

//    private final RelativeEncoder driveEncoder;
//    private final RelativeEncoder turningEncoder;

    private PIDController turningPIDController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetAngle;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetAngle = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        absoluteEncoder.configFactoryDefault();
        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        absoluteEncoder.setPositionToAbsolute();

        driveMotor = new LazyTalonFX(driveMotorId, true);
        configDriveMotor(driveMotorReversed);

        turningMotor = new LazyTalonFX(turningMotorId, false);
        configTurningMotor(turningMotorReversed);

//        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
//        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

//        driveMotor.setIdleMode(IdleMode.kBrake);
//        turningMotor.setIdleMode(IdleMode.kBrake);

//         turning Motor configuration

        turningPIDController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        resetEncoders();
        putDashboard();
    }

    private void configDriveMotor(boolean reversed) {
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(reversed);
        driveMotor.gearRatio = ModuleConstants.kDriveMotorGearRatio;
    }

    private void configTurningMotor(boolean reversed) {
        turningMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setInverted(reversed);
        turningMotor.gearRatio = ModuleConstants.kTurningMotorGearRatio;
    }
    public double getDrivePosition() {
        return driveMotor.getPositionAsRad();
    }

    public double getTurningPosition() {
        return turningMotor.getPositionAsRad();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocityAsMPS(ModuleConstants.kWheelCircumference);
    }

    public double getTurningVelocity() {
        return turningMotor.getVelocityAsMPS(ModuleConstants.kWheelCircumference);
    }

    public double getAbsoluteEncoderRad() {
        double angle = (absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetAngle) / 360.;
        angle *= 2.0 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotor.setRadPosition(0);
        turningMotor.setRadPosition(getAbsoluteEncoderRad());
//        driveEncoder.setPosition(0);
//        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }


    public void stop() {
        driveMotor.set(0);

        switch (turningMotor.getDeviceID()) {
            case (RobotMap.DriverPort.kFrontLeftTurningMotorPort):
            case (RobotMap.DriverPort.kBackRightTurningMotorPort):
                turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), -Math.PI/4));
                break;
            case (RobotMap.DriverPort.kFrontRightTurningMotorPort):
            case (RobotMap.DriverPort.kBackLeftTurningMotorPort):
                turningMotor.set(ControlMode.PercentOutput, turningPIDController.calculate(getTurningPosition(), Math.PI/4));
                break;
        }

//        turningMotor.set(0);
    }

    public void putDashboard () {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderRad());
        SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());
       // SmartDashboard.putNumber("Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getPosition());
//       SmartDashboard.putNumber("Turing position " + turningMotor.getDeviceID(), turningMotor.getPositionAsDegrees());
    }
}