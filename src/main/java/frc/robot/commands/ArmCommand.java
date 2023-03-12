package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utils.UserAnalog;

public class ArmCommand extends CommandBase{
    private Arm arm;
    private UserAnalog length;

    public ArmCommand(Arm arm, UserAnalog length) {
        this.arm = arm;
        this.length = length;
        this.addRequirements(arm);
        arm.setDefaultCommand(this);
    }

    @Override
    public void execute() {
        arm.extendArm(length.get());
    }
    
}
