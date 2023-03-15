package frc.robot.subsystems;

import static frc.robot.Constants.TALON_CENTER_ARM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.MotorUtils;

public class Arm extends SubsystemBase {

    private TalonFX elevatorMotor;
    private CANSparkMax armMotor;
    private Solenoid pincher;


    private double lowerLimitOfSensorPos;
    private double upperLimitOfSensorPos;

    public Arm() {
        this.elevatorMotor = MotorUtils.initTalonFX(TALON_CENTER_ARM);
        this.elevatorMotor.setSelectedSensorPosition(0);

        this.armMotor = MotorUtils.initSparkMax(Constants.SPARK_CENTER);

        // TODO add real pneumatic hub id to constants
        this.pincher = new Solenoid(PneumaticsModuleType.REVPH, 1);

        this.lowerLimitOfSensorPos = 5000.0;
        this.upperLimitOfSensorPos = 185000.0;
    }

    public void extend(double length) {

        double motorOutput = length;
        if (length < 0 && elevatorMotor.getSelectedSensorPosition() < lowerLimitOfSensorPos) {
            motorOutput = 0.0;
        }

        if (length > 0 && elevatorMotor.getSelectedSensorPosition() > upperLimitOfSensorPos) {
            motorOutput = 0.0;
        }

        elevatorMotor.set(ControlMode.PercentOutput, motorOutput);
    }
}
