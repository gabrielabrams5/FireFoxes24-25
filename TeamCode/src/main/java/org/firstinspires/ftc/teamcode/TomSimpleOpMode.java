package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TomSimpleOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        this.waitForStart();
        if(this.opModeIsActive()) {
            while(opModeIsActive()) {
                telemetry.addData("runtime", this.getRuntime());
                telemetry.update();
            }
        }
    }
}
