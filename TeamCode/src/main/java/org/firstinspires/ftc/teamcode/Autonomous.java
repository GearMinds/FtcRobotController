package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Robot")
public class Autonomous extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void run() throws InterruptedException {
        // Main autonomous code goes here...
        // See README.md for API documentation
        driveTrain.setSpeed(0.5); //half speed

        driveTrain.reverseFor(36.0); // Moves back 3 feet
        
        launcher.spinFlywheel();

        sleep(5000); // wait for 5000 ms before executing the next instruction

        launcher.feed();

        sleep(5000);

        launcher.stop();
    }
}
