package org.firstinspires.ftc.teamcode.Teleop;
import java.util.HashMap;
import java.lang.reflect.Field;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector3D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SwerveTeleOp extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        SwerveDrive drivetrain = new SwerveDrive();
        while (opModeIsActive()){
            drivetrain.setPodServoPosition();
            drivetrain.fourWheelDrive(new Vector3D(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x));
        }
    }
}
