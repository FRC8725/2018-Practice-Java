package frc.lib;

import com.revrobotics.CANSparkMax;

public class LazySparkMax extends CANSparkMax {
    public LazySparkMax(int deviceNumber, MotorType type){
        super(deviceNumber, type);
        setSmartCurrentLimit(50, 30);
        burnFlash();
    }
}
