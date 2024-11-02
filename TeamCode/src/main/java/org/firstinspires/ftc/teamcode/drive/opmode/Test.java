package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            double test = -gamepad1.right_stick_y;

            DcMotorEx lf, lb, rf, rb;
            lb       = hardwareMap.get(DcMotorEx.class, "lb");
            lf       = hardwareMap.get(DcMotorEx.class, "lf");
            rb       = hardwareMap.get(DcMotorEx.class, "rb");
            rf       = hardwareMap.get(DcMotorEx.class, "rf");

            telemetry.addData("rf: ", rf.getPower());
            telemetry.update();

            rf.setPower(test);
        }
    }
}
