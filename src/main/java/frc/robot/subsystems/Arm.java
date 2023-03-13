package frc.robot.subsystems;

import static frc.robot.Constants.TALON_CENTER_ARM;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.MotorUtils;

public class Arm extends SubsystemBase{

    private TalonFX motor;
    private double lowerLimitOfSensorPos;
    private double upperLimitOfSensorPos;

    public Arm() {
        this.motor = MotorUtils.initTalonFX(TALON_CENTER_ARM);
        this.motor.setSelectedSensorPosition(0);
        this.lowerLimitOfSensorPos = 5000.0;
        this.upperLimitOfSensorPos = 185000.0;    
    }

    public void extend(double length) {

        double motorOutput = length;
        if(length < 0 && motor.getSelectedSensorPosition() < lowerLimitOfSensorPos){
            motorOutput = 0.0;
        }  
        
        if(length > 0 && motor.getSelectedSensorPosition() > upperLimitOfSensorPos){
            motorOutput = 0.0;
        }

        motor.set(ControlMode.PercentOutput, motorOutput);
    }
}
