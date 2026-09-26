package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.lib.DriveTrain;
import org.firstinspires.ftc.teamcode.lib.Robot;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Example", group="Robot")
public class ExampleAuto extends Robot {
    DriveTrain driveTrain;

    @Override
    public void setup() throws InterruptedException {
        driveTrain = new DriveTrain(this);
    }

    @Override
    public void run() throws InterruptedException {
        // Challenge:
        // 1. Make it a rectangle
        // 2. Repeat the rectangle 4 times

        int length = 12;

        driveTrain.setSpeed(0.5);

        driveTrain.forwardFor(length);
        driveTrain.strafeLeftFor(length);
        driveTrain.reverseFor(length);
        driveTrain.strafeRightFor(length);
    }
}
