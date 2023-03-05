package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.UserAnalog;

public class TestDriveCommand extends CommandBase {
    private UserAnalog left, right;
    private DriveTrain dt;

    public TestDriveCommand(DriveTrain dt, UserAnalog lefti, UserAnalog righti) {
        this.dt = dt;
        left = lefti;
        right = righti;
    }

    @Override
    public void execute() {
        dt.testDrive(left.get(), right.get());
    }
}
