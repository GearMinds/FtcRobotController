package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Shoot", group="Robot")
public class ShootBlue extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(this);
    }

    @Override
    public void run() throws InterruptedException {
        // Main autonomous code goes here...
        // See README.md for API documentation
        driveTrain.setSpeed(0.5); // half speed for movement
        driveTrain.reverseFor(36.0); // Moves back 3 feet

        launcher.launchFor(12);

        driveTrain.strafeLeftFor(18.0);
    }
}
