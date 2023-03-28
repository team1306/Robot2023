/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.Controller;
import frc.robot.utils.UserAnalog;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // whether or not to run the autonomous command

    // The robot's subsystems and commands are defined here...
    // Subsystems
    private DriveTrain driveTrain;

    // Commands

    private DriveCommand driveCommand;

    // inputs for drive train
    private UserAnalog backwards;
    private UserAnalog forwards;
    private UserAnalog rotationDriveTrain;

    // turbo toggle button
    private JoystickButton turbo;


    private SendableChooser<Command> autos = new SendableChooser<>();

    // The robot's inputs that it recieves from the controller are defined here

    /**
     * The container for the robot. Contains subsystems, OI devices.
     */
    public RobotContainer() {
        // initalize controller (sets correct usb ports)
        Controller.init();
        // bind buttons
        configureButtonBindings();
        // create auto bindings

        // create subsytems
        driveTrain = new DriveTrain();

        // create commands
        // drive command
        driveCommand = new DriveCommand(driveTrain, backwards, forwards, rotationDriveTrain);
        driveTrain.setDefaultCommand(driveCommand);

        // turbo button
        turbo.debounce(0.05).toggleOnTrue(Commands.startEnd(() -> {
            driveCommand.turbo = false;
            SmartDashboard.putBoolean("isTurbo", false);
        }, () -> {
            driveCommand.turbo = true;
            SmartDashboard.putBoolean("isTurbo", true);
        }));

        // auto choices
        var nothing = Commands.none();
        var forward = Commands
            .startEnd(
                () -> driveTrain.arcadeDrive(0.5, 0),
                () -> driveTrain.arcadeDrive(0, 0),
                driveTrain
            )
            .withTimeout(2);
        var backward = Commands
            .startEnd(
                () -> driveTrain.arcadeDrive(-0.5, 0),
                () -> driveTrain.arcadeDrive(0, 0),
                driveTrain
            )
            .withTimeout(2);
        // forward then balance
        var balance = Commands
            .startEnd(
                () -> driveTrain.arcadeDrive(0.3, 0),
                () -> driveTrain.arcadeDrive(0, 0),
                driveTrain
            )
            .withTimeout(2)
            .andThen(new BalanceCommand(0, driveTrain));

        // forward, backward, balance
        var taxBalance = Commands.sequence(
            Commands
                .startEnd(
                    () -> driveTrain.arcadeDrive(0.5, 0),
                    () -> driveTrain.arcadeDrive(0, 0),
                    driveTrain
                )
                .withTimeout(2),
            Commands
                .startEnd(
                    () -> driveTrain.arcadeDrive(-0.3, 0),
                    () -> driveTrain.arcadeDrive(0, 0),
                    driveTrain
                )
                .withTimeout(0.75),
            new BalanceCommand(DriveTrain.gyro.getRoll(), driveTrain)
        );

        autos.setDefaultOption("do nothing", nothing);
        autos.addOption("driveForward", forward);
        autos.addOption("driveBackward", backward);
        autos.addOption("forward no taxi balance", balance);
        autos.addOption("forwar taxi balance", taxBalance);

        SmartDashboard.putData(autos);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // drivetrain inputs
        backwards = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LTRIGGER);
        forwards = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RTRIGGER);
        rotationDriveTrain = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);

        turbo = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_X);
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}
