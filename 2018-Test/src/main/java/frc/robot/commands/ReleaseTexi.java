package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Paneumatics;


public class ReleaseTexi extends CommandBase {

    private final Paneumatics paneumatics;

    public ReleaseTexi(Paneumatics paneumatics) {
        addRequirements(paneumatics);
        this.paneumatics = paneumatics;
    }

    @Override
    public void initialize() {
        paneumatics.releaseFork();
    }
}
