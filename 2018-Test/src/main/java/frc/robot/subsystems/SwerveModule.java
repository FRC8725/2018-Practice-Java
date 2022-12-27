package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.lib.LazyTalonFX;

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
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderId);

        absoluteEncoder.setPositionToAbsolute();

        driveMotor = new LazyTalonFX(driveMotorId, true);
        configDriveMotor();

        turningMotor = new LazyTalonFX(turningMotorId, true);
        configTurningMotor();

//        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
//        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

//        driveMotor.setIdleMode(IdleMode.kBrake);
//        turningMotor.setIdleMode(IdleMode.kBrake);

        // turning Motor configuration

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
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
        double angle = (absoluteEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad) / 360.;
        angle *= 2.0 * Math.PI;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        turningMotor.setAnglePosition(getAbsoluteEncoderAngle());
        driveMotor.setSelectedSensorPosition(0);
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
        driveMotor.set(ControlMode.Velocity, state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(ControlMode.Velocity, turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
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
        turningMotor.set(ControlMode.Position, 0);
//        turningMotor.set(turningPidController.calculate(turningEncoder.getPosition(), 0));
        driveMotor.set(.0);
        putDashboard();
    }
    
    
    public void putDashboard () {
        SmartDashboard.putNumber("ABS angle " + absoluteEncoder.getDeviceID(), getAbsoluteEncoderAngle());
        SmartDashboard.putNumber("Abs Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getAbsolutePosition());
       // SmartDashboard.putNumber("Position " + absoluteEncoder.getDeviceID(), absoluteEncoder.getPosition());
       // SmartDashboard.putNumber("Turing position " + turningMotor.getDeviceId(), turningEncoder.getPosition());
    }
}