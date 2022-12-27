package frc.lib;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LazyTalonFX extends WPI_TalonFX {
    public double gearRatio = 0;
    public LazyTalonFX(int deviceNumber, boolean isChassis, double... configuration) {
        super(deviceNumber);
        if (configuration.length > 0) gearRatio = configuration[0];
        // if the motor is for chassis, won't set the current limit.
        if (isChassis) return;
        configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 10, 15, 0.5));
        configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 1));
    }
    TalonFXConfiguration turningMotorConfig;

    public void setAnglePosition(double angle) {
        setSelectedSensorPosition(angle / 360. * 2048.);
    }

    public double getPositionAsDegrees() {
        return getSelectedSensorPosition() / (2048. / gearRatio) * 360.;
    }
}