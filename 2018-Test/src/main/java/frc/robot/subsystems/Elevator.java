package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class Elevator extends SubsystemBase{

    private static final Elevator INSTANCE = new Elevator();

    @SuppressWarnings("WeakerAccess")
    public static Elevator getInstantce() {return INSTANCE;}

    private final CANSparkMax lowleftMotor, lowrightMotor, winchMotor;
    private final RelativeEncoder winchEncoder, elevatorEncoder;
    private final MotorControllerGroup lowMotors;

    private final PIDController winchPID, elevatorPID;
    
    public Elevator(){
        lowleftMotor = new CANSparkMax(RobotMap.Elavator.lowleftMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowrightMotor = new CANSparkMax(RobotMap.Elavator.lowrightMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowleftMotor.setInverted(true);
        winchMotor = new CANSparkMax(RobotMap.Elavator.winchMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowMotors = new MotorControllerGroup(lowleftMotor, lowrightMotor);
        winchEncoder = winchMotor.getEncoder();
        winchEncoder.setPositionConversionFactor(Constants.Elavator.winchEncoerPositionFactor);
        elevatorEncoder = lowleftMotor.getEncoder();
        elevatorEncoder.setPositionConversionFactor(Constants.Elavator.elevatorEncoerPositionFactor);
        winchPID = new PIDController(Constants.Elavator.kpWinch, Constants.Elavator.kiWinch, Constants.Elavator.kdWinch);
        elevatorPID = new PIDController(Constants.Elavator.kpWinch, Constants.Elavator.kiElevator, Constants.Elavator.kdElevator);
    }

    public void liftElavator(int outputVolts) {
        lowMotors.setVoltage(outputVolts);
    }

    public void setWinch(int outputVolts) {
        winchMotor.setVoltage(outputVolts);
    }

    public void setElevator(int outputVolts) {
        lowMotors.setVoltage(outputVolts);
    }

    public void zeroWinchEncoder() {
        winchEncoder.setPosition(0);
    }

    public void setWinchSetpoint(double setpoint) {
        winchPID.setSetpoint(setpoint);
    }

    public void toWinchSetpoint() {
        winchMotor.setVoltage(winchPID.calculate(winchEncoder.getPosition()));
    }

    public void zeroElevatorEncoder() {
        elevatorEncoder.setPosition(0);
    }

    public void setElevatorSetpoint(double setpoint) {
        elevatorPID.setSetpoint(setpoint);
    }

    public void toElevatorSetpoint() {
        lowMotors.setVoltage(elevatorPID.calculate(elevatorEncoder.getPosition()));
    }
}
