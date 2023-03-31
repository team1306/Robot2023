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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
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
    private Intake intake;

    // Commands
    private DriveCommand driveCommand;

    // inputs for drive train
    private UserAnalog backwards;
    private UserAnalog forwards;
    private UserAnalog rotationDriveTrain;

    // toggle buttons
    private JoystickButton turbo, balance, intakeIn, intakeOut;


    private SendableChooser<Command> autos = new SendableChooser<>();

    // The robot's inputs that it recieves from the controller are defined here

    /**
     * The container for the robot. Contains subsystems, OI devices.
     */
    public RobotContainer() {
        // bind buttons and initalize controller ports
        configureButtonBindings();

        // create subsytems
        driveTrain = new DriveTrain();
        intake = new Intake();

        // =========== create commands ===========
        // auto command
        configureAutoChoices();

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

        intakeIn.whileTrue(intake.runCmd(-1));
        intakeOut.whileTrue(intake.runCmd(1));


        // TODO check with 0 instead?
        balance.whileTrue(new BalanceCommand(DriveTrain.gyro.getRoll(), driveTrain));
    }


    /**
     * configures and displays the options for autonomous commands
     */
    private void configureAutoChoices() {
        // auto choices
        // do nothing
        var nothing = Commands.none();
        // just taxi (fwd/bkwd)
        var forward = driveTrain.driveOutput(0.5, 0).withTimeout(2);
        var backward = driveTrain.driveOutput(-0.5, 0).withTimeout(2);
        // forward then balance
        var balance = driveTrain.driveOutput(0.3, 0)
            .withTimeout(2)
            .andThen(new BalanceCommand(0, driveTrain).withTimeout(5));
        // forward, backward, balance
        var taxBalance = Commands.sequence(
            driveTrain.driveOutput(0.5, 0).withTimeout(2),
            driveTrain.driveOutput(-0.3, 0).withTimeout(0.75),
            new BalanceCommand(DriveTrain.gyro.getRoll(), driveTrain).withTimeout(5)
        );
        // score and back up
        var scoreTax = Commands.sequence(
            intake.runCmd(1).withTimeout(0.5),
            driveTrain.driveOutput(-0.5, 0).withTimeout(2)
        );
        // score, backup to taxi, go forward on ramp, and balance
        var scoreTaxBal = Commands.sequence(
            intake.runCmd(1).withTimeout(0.5),
            driveTrain.driveOutput(-0.35, 0).withTimeout(3),
            new WaitCommand(1.5),
            driveTrain.driveOutput(0.3, 0).withTimeout(1),
            new BalanceCommand(0, driveTrain).withTimeout(5)
        );

        // add choices to Smartdashboard
        autos.setDefaultOption("do nothing", nothing);
        autos.addOption("driveForward", forward);
        autos.addOption("driveBackward", backward);
        autos.addOption("forward no taxi balance", balance);
        autos.addOption("forwar taxi balance", taxBalance);
        autos.addOption("score taxi", scoreTax);
        autos.addOption("score taxi balance", scoreTaxBal);
        SmartDashboard.putData("Auto commands", autos);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by instantiating a
     * {@link GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        Controller.init();

        // drivetrain inputs
        backwards = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LTRIGGER);
        forwards = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_RTRIGGER);
        rotationDriveTrain = Controller.simpleAxis(Controller.PRIMARY, Controller.AXIS_LX);

        turbo = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_X);
        balance = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_B);

        intakeIn = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_A);
        intakeOut = Controller.getJoystickButton(Controller.PRIMARY, Controller.BUTTON_Y);

    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}
