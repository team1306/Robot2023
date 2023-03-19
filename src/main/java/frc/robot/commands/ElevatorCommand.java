package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.utils.UserAnalog;

/**
 * command to manually run the elevator using its runElevator method
 */
public class ElevatorCommand extends CommandBase {
    // corresponding subsystem
    private Elevator elevator;
    // input for running elevator
    private UserAnalog elevInput;

    public ElevatorCommand(Elevator elevator, UserAnalog input) {
        this.elevInput = input;
        this.elevator = elevator;

        this.addRequirements(elevator);
        elevator.setDefaultCommand(this);
    }

    @Override
    public void execute() {
        elevator.runElevator(elevInput.get());
    }
}
