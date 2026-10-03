package org.firstinspires.ftc.teamcode.legacy.decode2526;

import org.firstinspires.ftc.teamcode.lib.DecodeLauncher;
import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

// @com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Shoot", group="Robot")
public class ShootRed extends Robot {
    DriveTrain driveTrain;
    DecodeLauncher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new DecodeLauncher(this);
    }

    @Override
    public void run() throws InterruptedException {
        // Main autonomous code goes here...
        // See README.md for API documentation
        driveTrain.setSpeed(0.5); // half speed for movement
        driveTrain.reverseFor(36.0); // Moves back 3 feet

        launcher.launchFor(12);

        driveTrain.strafeRightFor(18.0);
    }
}
