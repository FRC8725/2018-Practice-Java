package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.lib.LazyTalonFX;

import javax.naming.ldap.Control;

public class SwerveModule {

//    private final CANSparkMax driveMotor;
    private final LazyTalonFX driveMotor, turningMotor;
//    private final CANSparkMax driveMotor;
//    private final CANSparkMax turningMotor;

//    private final RelativeEncoder driveEncoder;
//    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetAngle;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetAngle = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        absoluteEncoder.setPositionToAbsolute();

        driveMotor = new LazyTalonFX(driveMotorId, true);
        configDriveMotor();

        turningMotor = new LazyTalonFX(turningMotorId, false);
        configTurningMotor();

//        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
//        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

//        driveMotor.setIdleMode(IdleMode.kBrake);
//        turningMotor.setIdleMode(IdleMode.kBrake);

        // turning Motor configuration

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-180, 180);
        resetEncoders();
        putDashboard();
    }

    private void configDriveMotor() {
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.gearRatio = ModuleConstants.kDriveMotorGearRatio;
    }

    private void configTurningMotor() {
        turningMotor.setNeutralMode(NeutralMode.Coast);
        turningMotor.config_kP(0, ModuleConstants.kPTurning);
        turningMotor.config_kI(0, ModuleConstants.kITurning);
        turningMotor.config_kD(0, ModuleConstants.kDTurning);
        turningMotor.gearRatio = ModuleConstants.kTurningMotorGearRatio;
    }
    public double getDrivePosition() {
        return driveMotor.getPositionAsDegrees();
    }

    public double getTurningPosition() {
        return turningMotor.getPositionAsDegrees();
    }

    public double getDriveVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    public double getTurningVelocity() {
        return turningMotor.getSelectedSensorVelocity();
    }

    public double getAbsoluteEncoderAngle() {
        double angle = (absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetAngle);
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        turningMotor.setAnglePosition(getAbsoluteEncoderAngle());
//        driveEncoder.setPosition(0);
//        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(Math.toRadians(getTurningPosition())));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(ControlMode.PercentOutput, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), state.angle.getDegrees()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());
        putDashboard();
    }


    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
    public void ResetTurningMotor() {
        if (Math.abs(turningMotor.getSelectedSensorPosition()) < 0.1){
            stop();
            return;
        }
        // turningMotor.set(ControlMode.Position, 0);
        turningMotor.set(ControlMode.PercentOutput, turningPidController.calculate(getTurningPosition(), 0));
        driveMotor.set(.0);
        putDashboard();
    }
    
    
    public void putDashboard () {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());
       // SmartDashboard.putNumber("Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getPosition());
       SmartDashboard.putNumber("Turing position " + turningMotor.getDeviceID(), turningMotor.getPositionAsDegrees());
    }
}