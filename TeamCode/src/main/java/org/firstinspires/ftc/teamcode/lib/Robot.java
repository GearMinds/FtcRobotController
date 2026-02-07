package org.firstinspires.ftc.teamcode.lib;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Robot extends LinearOpMode {

    private boolean cycleShouldRun = false;

    public abstract void setup() throws InterruptedException;

    public void cycle(double delta) throws InterruptedException {}

    public void run() throws InterruptedException {
        // If this method is overridden, the cycle should not be run (Unless they explicitly set it)
        cycleShouldRun = true;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        this.setup();
        waitForStart();
        this.run();

        if (cycleShouldRun) {
            double delta = System.currentTimeMillis();
            while (opModeIsActive()) {
                delta = System.currentTimeMillis() - delta;
                this.cycle(delta);
            }
        }
    }
}