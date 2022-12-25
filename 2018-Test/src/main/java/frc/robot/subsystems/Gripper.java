package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LazyTalonFX;
import frc.robot.Constants;
import frc.robot.RobotMap;

public class Gripper extends SubsystemBase {
    private final static Gripper INSTANCE = new Gripper();

    private final LazyTalonFX leftSpinMotor, rightSpinMotor;
    private final CANSparkMax angleMotor;
    private final RelativeEncoder angleEncoder;
    private final MotorControllerGroup SpinMotorGroup;
    private final PIDController anglePID;

    @SuppressWarnings("WeakerAccess")
    public static Gripper getInstance() {return INSTANCE;}

    private Gripper() {
        leftSpinMotor = new LazyTalonFX(RobotMap.Gripper.leftSpinMotor);
        configSpinMotor(leftSpinMotor, true);
        rightSpinMotor = new LazyTalonFX(RobotMap.Gripper.rightSpinMotor);
        configSpinMotor(rightSpinMotor, false);
        SpinMotorGroup = new MotorControllerGroup(leftSpinMotor, rightSpinMotor);
        angleMotor = new CANSparkMax(RobotMap.Gripper.angleMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        configAngleMotor(angleMotor);
        angleEncoder = angleMotor.getEncoder();
        anglePID = new PIDController(Constants.Gripper.kpAngle, 0, 0);
        anglePID.setTolerance(.1);
    }

    private void configSpinMotor(WPI_TalonFX motor, boolean isLeft) {
        motor.setNeutralMode(NeutralMode.Coast);
        motor.setInverted(isLeft);
    }

    private void configAngleMotor(CANSparkMax motor) {
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.getEncoder().setPositionConversionFactor(Constants.Gripper.angleMotorGearRatio);
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void periodic() {
        double voltage = anglePID.calculate(angleEncoder.getPosition());
        angleMotor.setVoltage(voltage);
    }

    public void setSpinner(double speed) {
        SpinMotorGroup.set(speed);
    }

    public void setAngleMotor(double speed) {
        angleMotor.set(speed);
    }

    public void setAngleSetpoint(double setpoint) {
        anglePID.setSetpoint(setpoint);
    }

    public boolean atSetpoint() {
        return anglePID.atSetpoint();
    }
}