package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Launcher {

    public CRServo rightFeeder, leftFeeder;
    public DcMotor flywheel;

    private int feedDirection = 0;
    private boolean spinning = false;

    private final Robot robot;
    private final double DEFAULT_FEED_THRESHOLD = 2100.0;
    public double feedThreshold = DEFAULT_FEED_THRESHOLD;

    public Launcher(Robot robot) {
        this.flywheel = robot.hardwareMap.get(DcMotor.class, "flywheel");
        this.leftFeeder = robot.hardwareMap.get(CRServo.class, "feeder_l");
        this.rightFeeder = robot.hardwareMap.get(CRServo.class, "feeder_r");
        this.robot = robot;
    }

    private void setFeederState(double leftPower, double rightPower, int direction) {
        if (this.feedDirection != direction) {
            this.leftFeeder.setPower(leftPower);
            this.rightFeeder.setPower(rightPower);
            this.feedDirection = direction;
        }
    }

    public void feed() {
        setFeederState(1.0, -1.0, 1);
    }

    public void backFeed() {
        setFeederState(-1.0, 1.0, -1);
    }

    public void stopFeeder() {
        setFeederState(0.0, 0.0, 0);
    }

    public void feedIfReady() {
        if (!this.spinning || ((DcMotorEx) this.flywheel).getVelocity() > this.feedThreshold) {
            this.feed();
        } else {
            this.stopFeeder();
        }
    }

    public boolean isSpinning() {
        return this.spinning;
    }

    public void spinFlywheel() {
        flywheel.setPower(1.0);
        this.spinning = true;
    }

    public void stopFlywheel() {
        flywheel.setPower(0.0);
        this.spinning = false;
    }

    public void toggleFlywheel() {
        if (!this.spinning) {
            spinFlywheel();
        } else {
            stopFlywheel();
        }
    }

    public void launchFor(double seconds) {
        long milliseconds = ((long) seconds) * 1000;
        final long sample = 100;

        this.spinFlywheel();

        while (milliseconds > 0) {
            milliseconds -= sample;
            this.feedIfReady();
            this.robot.sleep(sample);
        }

        this.stop();
    }

    public void stop() {
        this.stopFlywheel();
        this.stopFeeder();
    }
}
