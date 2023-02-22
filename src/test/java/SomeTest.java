
import frc.robot.subsystems.DriveTrain;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
// import edu.wpi.first.wpilibj.simulation.PWMSim;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * testers probably
 */
public class SomeTest {
    private DriveTrain dtrain;

    @BeforeEach
    public void setup() {
        assert HAL.initialize(500, 0);
        // System.out.println("Setup ");
        dtrain = new DriveTrain();
    }

    @AfterEach
    public void shutdown() throws Exception {
        // System.out.println("done");
        HAL.shutdown();
        dtrain.close();
    }
}
