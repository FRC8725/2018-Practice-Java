package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalOutput;
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
    private final RelativeEncoder winchEncoder;
    private final MotorControllerGroup lowMotors;
    private final DigitalOutput Limiter;

    private final PIDController winchPID;
    
    public Elevator(){
        lowleftMotor = new CANSparkMax(RobotMap.Elavator.lowleftMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowrightMotor = new CANSparkMax(RobotMap.Elavator.lowrightMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowleftMotor.setInverted(true);
        winchMotor = new CANSparkMax(RobotMap.Elavator.winchMotor, CANSparkMaxLowLevel.MotorType.kBrushless);
        lowMotors = new MotorControllerGroup(lowleftMotor, lowrightMotor);
        Limiter = new DigitalOutput(0);
        winchEncoder = winchMotor.getEncoder();
        winchEncoder.setPositionConversionFactor(Constants.Elavator.winchEncoerPositionFactor);
        winchPID = new PIDController(Constants.Elavator.kpWinch, Constants.Elavator.kiWinch, Constants.Elavator.kdWinch);
    }

    public void liftElavator(int outputVolts) {
        lowMotors.setVoltage(outputVolts);
    }

    public void setWinch(int outputVolts) {
        winchMotor.setVoltage(outputVolts);
    }

    public boolean getSwitch() {
        return Limiter.get();
    }

    public double getWinchEncoder() {
        return winchEncoder.getPosition();
    }

    public void zeroEncoder() {
        winchEncoder.setPosition(0);
    }

    public void setSetpoint(double setpoint) {
        winchPID.setSetpoint(setpoint);
    }

    public double toSetpoint() {
        return winchPID.calculate(winchEncoder.getPosition());
    }
}
