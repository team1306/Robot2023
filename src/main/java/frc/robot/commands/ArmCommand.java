package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.UserAnalog;

public class ArmCommand extends CommandBase {

    private Arm arm;
    private UserAnalog armInput;

    public ArmCommand(Arm arm, UserAnalog armInput) {
        this.arm = arm;
        this.armInput = armInput;
        this.addRequirements(arm);
        arm.setDefaultCommand(this);
    }

    @Override
    public void execute() {
        // arm.extend(armInput.get());
    }

}
