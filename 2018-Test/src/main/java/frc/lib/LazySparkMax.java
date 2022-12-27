package frc.lib;

import com.revrobotics.CANSparkMax;

public class LazySparkMax extends CANSparkMax {
    public LazySparkMax(int deviceNumber, MotorType type, double... configuration){
        super(deviceNumber, type);
        setSmartCurrentLimit(60, 30);
    }
}
