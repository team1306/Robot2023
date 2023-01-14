/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ResourceBundle.Control;
import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Controller;
import frc.robot.utils.REVDigitBoard;
import frc.robot.utils.UserAnalog;
import frc.robot.utils.UserDigital;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings("unused")
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private Command autoCommand;
    private Command driveCommand;
    private DriveTrain driveTrain;

    private final boolean RUN_AUTO = true;

    // inputs for drive train
    private UserAnalog speedDriveTrain;
    private UserAnalog backwardsTurbo;
    private UserAnalog forwardTurbo;
    private UserAnalog joystickRotationDriveTrain;

    public static double RC_MAX_SPEED;

    // The robot's inputs that it recieves from the controller are defined here

    /**
     * The container for the robot. Contains subsystems, OI devices.
     */
    public RobotContainer() {
        // initialize controller using Controller init method to ensure Controller is
        // properly initialized when the
        // Controllern is required to be initialized at the start of the game.
        // Controller represents an xbox controller
        // which controls the robot using standard controller inputs such as joysticks,
        // buttons, and triggers.
        Controller.init();
        configureButtonBindings();

        driveTrain = new DriveTrain();
        driveCommand = new DriveCommand(
            driveTrain,
            speedDriveTrain,
            backwardsTurbo,
            forwardTurbo,
            joystickRotationDriveTrain
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        speedDriveTrain = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LY);
        backwardsTurbo = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LTRIGGER);
        forwardTurbo = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RTRIGGER);
        joystickRotationDriveTrain = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);
    }

    /**
     * called when autonomous is started should create all commands that are used in auto
     */
    public void startAuto() {
        autoCommand = getAutonomousCommand();
        if (RUN_AUTO) {
            driveCommand.cancel();
            autoCommand.schedule();
        }
        // System.out.println("startAuto() IS RUNNING.");
    }

    /**
     * start off teleop period by cancelling autonomous command and switching the drivetrain command to the user driving
     * command
     * 
     */
    public void startTeleop() {
        // This makes sure that the autonomous stops running when teleop starts running.
        // If you want the autonomous to
        // continue until interrupted by another command, remove this line or comment it
        // out.
        if (RUN_AUTO)
            autoCommand.cancel();
    }

    public Command getAutonomousCommand() {
        return null;
    }

}
