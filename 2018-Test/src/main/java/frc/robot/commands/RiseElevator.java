package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class RiseElevator extends CommandBase{
    private final Elevator elevator;

    private final double setpoint;
    private final BooleanSupplier toggleWinch;

    public RiseElevator(Elevator elevator, BooleanSupplier toggleWinch) {
        this.elevator = elevator;
        this.toggleWinch = toggleWinch;

        setpoint = Constants.Elavator.releaseWinchSetpoint;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.zeroEncoder();
    }

    @Override
    public void execute() {
        if (toggleWinch.getAsBoolean()) {
            elevator.setSetpoint(setpoint);
        } else {
            elevator.setSetpoint(0);
        }
        elevator.toSetpoint();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.setSetpoint(0);
    }
}
