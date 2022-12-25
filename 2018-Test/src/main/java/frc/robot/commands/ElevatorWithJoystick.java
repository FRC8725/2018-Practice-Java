package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

import java.util.function.Supplier;


public class ElevatorWithJoystick extends CommandBase {

    private final Elevator elevator;
    Supplier<Boolean> lift, low;

    public ElevatorWithJoystick(Elevator elevator, Supplier<Boolean> lift, Supplier<Boolean> low) {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(elevator);
        this.elevator = elevator;
        this.lift = lift;
        this.low = low;
    }

    @Override
    public void initialize() {
        elevator.zeroElevatorEncoder();
        elevator.setElevatorSetpoint(0);
    }

    @Override
    public void execute() {
        double setpoint = elevator.getSetpoint();
        if (lift.get() && setpoint < .5) {
            setpoint += 0.1;
        }
        else if (low.get() && setpoint > 0) {
            setpoint -= 0.1;
        }
        elevator.setElevatorSetpoint(setpoint);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        elevator.stopElevator();
    }
}
