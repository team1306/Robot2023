package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.UserAnalog;

/**
 * command to manually run the arm using its runArm method
 */
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
        arm.runArm(MathUtil.applyDeadband(armInput.get(), 0.05));
    }

}
