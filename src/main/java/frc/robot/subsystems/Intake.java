package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;

public class Intake extends SubsystemBase {
    // our electronic components
    private WPI_TalonSRX left, right;

    // output of the motor when the intake is running
    private final double speed = 0.35;

    public Intake() {
        left = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_LEFT);
        right = MotorUtils.initWPITalonSRX(Constants.TALON_FAR_RIGHT);
        // probably should be inverted to produce net inward force
        right.setInverted(true);
    }

    public void set(int sign) {
        sign = Integer.signum(sign);
        left.set(ControlMode.PercentOutput, Integer.signum(sign) * speed);
        right.set(ControlMode.PercentOutput, Integer.signum(sign) * speed);
        SmartDashboard.putNumber("Intake", sign);
    }

    public Command runCmd(int sign) {
        return startEnd(() -> set(sign), () -> set(0));
    }
}
