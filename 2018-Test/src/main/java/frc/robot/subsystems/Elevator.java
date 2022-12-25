package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase{

    private static final Elevator INSTANCE = new Elevator();

    @SuppressWarnings("WeakerAccess")
    public static Elevator getInstantce() {return INSTANCE;}

    private final CANSparkMax lowleftMotor, lowrightMotor;
    private final RelativeEncoder elevatorEncoder;
    private final MotorControllerGroup lowMotors;

    private final PIDController elevatorPID;

    public Elevator(){
        lowleftMotor = new CANSparkMax(RobotMap.Elavator.lowleftMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowrightMotor = new CANSparkMax(RobotMap.Elavator.lowrightMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowleftMotor.setInverted(true);
        lowMotors = new MotorControllerGroup(lowleftMotor, lowrightMotor);
        elevatorEncoder = lowleftMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(Constants.Elevator.elevatorEncoderPositionFactor);
        zeroElevatorEncoder();
        elevatorPID = new PIDController(Constants.Elevator.kpElevator, Constants.Elevator.kiElevator, Constants.Elevator.kdElevator);
    }
    @Override
    public void periodic() {
        double voltage = elevatorPID.calculate(elevatorEncoder.getPosition());
        lowMotors.setVoltage(voltage);
    }

    public void zeroElevatorEncoder() {
        elevatorEncoder.setPosition(0);
    }

    public double getSetpoint() {
        return elevatorPID.getSetpoint();
    }

    public void setElevatorSetpoint(double setpoint) {
        elevatorPID.setSetpoint(setpoint);
    }

    public void stopElevator() {
        lowMotors.set(0);
        elevatorPID.setSetpoint(elevatorEncoder.getPosition());
    }

    public boolean atSetpoint() {
        return elevatorPID.atSetpoint();
    }
}
