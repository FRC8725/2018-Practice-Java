package frc.lib;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LazyTalonFX extends WPI_TalonFX {
    public double gearRatio = 0;
    public LazyTalonFX(int deviceNumber, boolean isHighCurrent) {
        super(deviceNumber);
//         if the motor is for chassis, won't set the current limit.
         if (!isHighCurrent){
             configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 10, 0.5));
             configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 40, 20, 1));
         } else {
             configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 10, 0.5));
             configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 48, 20, 1));
         }
        configFactoryDefault();
    }

    public void setAnglePosition(double angle) {
        setSelectedSensorPosition(angle / 360. * 2048.);
    }

    public double getPositionAsDegrees() {
        return getSelectedSensorPosition() / (2048. / gearRatio) * 360.;
    }
}