
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.apriltag.AprilTagDetector.Config;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    public static DriveTrain driveTrain;
    public static RamseteCommand autocmd;
    // private static Command testCommand = null;

    // private RobotContainer m_robotContainer;

    // messing around w/ apriltag detector & video stuff
    private AprilTagDetector detector = new AprilTagDetector();

    private CvSource out, out2;
    private VisionThread m_VisionThread;

    // public static Command driveCommand;

    // public static Intake intake = null;

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {
        // configure apriltag detector
        detector.addFamily("tag16h5");
        var config = new Config();
        config.numThreads = 4;
        config.quadDecimate = 1;
        config.quadSigma = 0;
        config.refineEdges = true;
        detector.setConfig(config);

        // get camera input
        var cam = CameraServer.startAutomaticCapture();
        cam.setResolution(480, 270);

        // output video stream, one for color & markings, one for black & white
        out = CameraServer.putVideo("out", 480, 270);
        out2 = CameraServer.putVideo("bw", 480, 270);

        m_VisionThread = new VisionThread(cam, mat -> {
            // create matrix for modifications, and copy input from camera
            Mat tempMat = Mat.zeros(mat.size(), mat.type());
            mat.copyTo(tempMat);
            // convert to grayscale and filter brightness => 120 -> white and < 120 -> black
            Imgproc.cvtColor(tempMat, tempMat, Imgproc.COLOR_BGR2GRAY);
            Imgproc.threshold(tempMat, tempMat, 120, 255, Imgproc.THRESH_BINARY);
            // detect tags
            var dets = detector.detect(tempMat);
            if (dets.length > 0) {
                // only valid tags from 1 to 8
                for (var det : dets) {
                    if (det.getId() > 8 || det.getId() < 1)
                        continue;
                    // draw center and corners on colored output matrix
                    Imgproc.drawMarker(mat, new Point(det.getCenterX(), det.getCenterY()), new Scalar(0, 255, 0));
                    for (int i = 0; i < det.getCorners().length / 2; i++) {
                        Imgproc.drawMarker(
                                mat, new Point(det.getCornerX(i), det.getCornerY(i)),
                                new Scalar(0, 0, 255));
                    }
                }
            }
            // output frames to camera server in shuffleboard
            out.putFrame(mat);
            out2.putFrame(tempMat);
        }, a -> {
            // do nothing
        });
        // run thread
        m_VisionThread.setDaemon(true);
        m_VisionThread.start();

    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for
     * items like diagnostics that you want
     * ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already- scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // m_robotContainer.startAuto();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        // m_robotContainer.startTeleop();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationPeriodic() {
        CommandScheduler.getInstance().run();
        REVPhysicsSim.getInstance().run();
    }
}