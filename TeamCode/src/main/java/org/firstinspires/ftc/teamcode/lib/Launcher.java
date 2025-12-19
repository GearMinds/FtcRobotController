package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Launcher {

    public CRServo rightFeeder, leftFeeder;
    public DcMotor flywheel;

    private boolean isSpinning = false;

    public Launcher(HardwareMap hardwareMap) {
        this(
                hardwareMap.get(DcMotor.class, "flywheel"),
                hardwareMap.get(CRServo.class, "feeder_l"),
                hardwareMap.get(CRServo.class, "feeder_r")
        );
    }

    public Launcher(DcMotor flywheel, CRServo leftFeeder, CRServo rightFeeder) {
        this.flywheel = flywheel;
        this.leftFeeder = leftFeeder;
        this.rightFeeder = rightFeeder;
    }

    public void feed() {
        leftFeeder.setPower(1.0);
        rightFeeder.setPower(-1.0);
    }

    public void unfeed() {
        leftFeeder.setPower(-1.0);
        rightFeeder.setPower(1.0);
    }

    public void stopFeeder() {
        leftFeeder.setPower(0.0);
        rightFeeder.setPower(0.0);
    }

    public void spinFlywheel() {
        isSpinning = true;
        flywheel.setPower(1.0);
    }

    public void stopFlywheel() {
        isSpinning = false;
        flywheel.setPower(0.0);
    }

    public void toggleFlywheel() {
        isSpinning = !isSpinning;
        if (isSpinning) {
            spinFlywheel();
        } else {
            stopFlywheel();
        }
    }
}
