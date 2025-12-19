package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.Robot;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.OmniDriveTrain;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Robot")
public class Autonomous extends Robot {

    OmniDriveTrain driveTrain;
    Launcher launcher;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new OmniDriveTrain(hardwareMap);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void run() throws InterruptedException {
        // Start program

        // 1. Move backward
        driveTrain.moveBackward();
        sleep(500);
        driveTrain.stop();

        // 2. Spin up flywheel
        launcher.spinFlywheel();
        sleep(3000);

        // 3. Start the feeder
        launcher.feed();
        sleep(5000);

        // 4. Stop feeding, stop the flywheel
        launcher.stopFeeder();
        launcher.stopFlywheel();
    }
}
