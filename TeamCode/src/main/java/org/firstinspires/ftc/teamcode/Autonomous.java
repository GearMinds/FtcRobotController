package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Launcher;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Autonomous", group="Robot")
public class Autonomous extends Robot {
    DriveTrain driveTrain;
    Launcher launcher;

    double speed = 0.2;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(hardwareMap);
    }

    @Override
    public void run() throws InterruptedException {
        driveTrain.setSpeed(0.25);

        // Start program
        driveTrain.driveForwardFor(12);
        driveTrain.strafeRightFor(12);
        driveTrain.strafeLeftFor(12);
        driveTrain.driveBackwardFor(12);

        driveTrain.turnRightFor(90);
        driveTrain.turnLeftFor(180);
    }
}
