package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Paneumatics extends SubsystemBase {

    private final static Paneumatics INSTANCE = new Paneumatics();

    @SuppressWarnings("WeakerAccess")
    public static Paneumatics getInstance() {
        return INSTANCE;
    }

    DoubleSolenoid leftForklift, rightForklift;
    Compressor compressor;
    public Paneumatics() {
        leftForklift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM , RobotMap.Pneumatics.leftForklift[0], RobotMap.Pneumatics.leftForklift[1]);
        rightForklift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM , RobotMap.Pneumatics.rightForklift[0], RobotMap.Pneumatics.rightForklift[1]);
        leftForklift.set(DoubleSolenoid.Value.kReverse);
        rightForklift.set(DoubleSolenoid.Value.kReverse);
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    }

    @Override
    public void periodic() {
        compressor.enableDigital();
    }

    public void releaseFork() {
        leftForklift.set(DoubleSolenoid.Value.kForward);
        rightForklift.set(DoubleSolenoid.Value.kForward);
    }
}

