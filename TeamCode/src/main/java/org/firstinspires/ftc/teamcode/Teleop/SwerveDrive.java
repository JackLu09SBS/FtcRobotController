package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import org.firstinspires.ftc.robotcontroller.Control.PID_Controller;

public class SwerveDrive {
    private DcMotorEx lf, lb, rf, rb, intake, arm;
    private Servo drone, grip, gripturn,servo;

    public void fourWheelDrive(Vector3D input) {
        double max;
        double fLeft = gamepad1.left_stick_y + gamepad1.right_stick_x;
        double fRight = gamepad1.left_stick_y - gamepad1.right_stick_x;
        double bRight = gamepad1.left_stick_y - gamepad1.right_stick_x;
        double bLeft = gamepad1.left_stick_y + gamepad1.right_stick_x;

        max = Math.max(Math.max(Math.abs(fLeft), Math.abs(fRight)), Math.max(Math.abs(bLeft), Math.abs(bRight)));
        if (max > 1.0) {
            fLeft /= max;
            fRight /= max;
            bLeft /= max;
            bRight /= max;
        }

        setPower(fLeft, fRight, bRight, bLeft);
    }
    public void setPower(double fLeft, double fRight, double bRight, double bLeft)
    {
        lf.setPower(fLeft);
        lb.setPower(bLeft);
        rf.setPower(fRight);
        rb.setPower(bRight);
    }
    public DcMotorEx podServo;

    public  PID_Controller PodPID = new PID_Controller(0, 0, 0, 0);


    public void setAngle(double angle) {
        double servoPower = PodPID.PID_Power(currAngle(), angle);
        podServo.setPower(servoPower);
    }

    public void setPodServoPosition() {
        double angle = Math.tan(gamepad1.left_stick_y / gamepad1.left_stick_x);
        setAngle(angle);
    }

    double previousPos=0;
    double currentPos=0;
    double adjustedPos=0;
    public double currAngle()
    {
        double delta;
        currentPos = servo.getPosition(); //Does not get real pos, need to use 4th cable
        delta = currentPos - previousPos;

            if (delta > 180) {
                delta -= 360;
            }
            if (delta < -180) {
                delta += 360;
            }

        adjustedPos += delta;
        previousPos = currentPos;

        return adjustedPos;
    }

}
