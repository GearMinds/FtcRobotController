package org.firstinspires.ftc.teamcode.legacy.decode2526;

import org.firstinspires.ftc.teamcode.lib.DecodeLauncher;
import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Leave", group="Robot")
public class Leave extends Robot {
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
        sleep(27000);
        driveTrain.reverseFor(12);
    }
}
