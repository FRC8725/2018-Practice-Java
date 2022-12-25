package frc.lib;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class LazyTalonFX extends WPI_TalonFX {
    public double gearRatio = 0;
    public LazyTalonFX(int deviceNumber, int... configuration) {
        super(deviceNumber);
        if (configuration.length > 0) gearRatio = configuration[0];
    }
    TalonFXConfiguration turningMotorConfig;

    public void setAnglePosition(double angle) {
        setSelectedSensorPosition(angle / 360. * 2048.);
    }

    public double getPositionAsDegrees() {
        return getSelectedSensorPosition() / (2048. / gearRatio) * 360.;
    }
}