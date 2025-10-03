package org.firstinspires.ftc.teamcode;

public class Autonomous extends org.firstinspires.ftc.teamcode.lib.Robot {

    @Override
    public void setup() {}

    @Override
    public void cycle(double delta) {
        telemetry.addData("delta", delta);
        telemetry.update();
    }
}
