package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Robot extends LinearOpMode {
    public abstract void setup();
    public abstract void cycle(double delta);

    @Override
    public void runOpMode() throws InterruptedException {

        this.setup();

        waitForStart();

        double delta = System.currentTimeMillis();
        while (opModeIsActive()) {
            this.cycle(System.currentTimeMillis() - delta);
        }
    }
}
