package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/*
 * Constants shared between multiple drive types.
 *
 * TODO: Tune or adjust the following constants to fit your robot. Note that the non-final
 * fields may also be edited through the dashboard (connect to the robot's WiFi network and
 * navigate to https://192.168.49.1:8080/dash). Make sure to save the values here after you
 * adjust them in the dashboard; **config variable changes don't persist between app restarts**.
 *
 * These are not the only parameters; some are located in the localizer classes, drive base classes,
 * and op modes themselves.
 */
@Config
public class DriveConstants {
    public static double POSER_REST = 0.135;
    public static double POSER_LV1 = 0.695;
    public static double POSER_LV2 = 0.65;
    public static double INTAKE_POWER = 0.43;
    public static double HANGING_POWER = 0.8;
    public static double UNWIND_POWER=-0.5;

    public static double HANGING_POS = 0.5;
    public static double ARM_DOWN = 0.92;
    public static double ARM_BLOCK = 0.80;
    public static double ARM_UP = 0.55;
    public static double DELTA = 0.0003;

   // public static double HANGING_POS = 0;

    // public static double ARM_DOWN = 0.124;
    //public static double ARM_BLOCK = 0.185;
    //public static double ARM_UP = 0.59;
    //public static double DELTA = 0.0015;
    //public static double ARM_UP_AUTON = 0.6;
    //public static double ARM_GROUND = 0.81;
    public static double GRIPTURN_DOWN = 0.80;
    public static double GRIPTURN_UP = 0.19;
    public static double GRIP_OPEN = 0.27;

    public static double GRIP_CLOSE = 0.21;
    public static double DRONE_HOLD = 0.16;
    public static double DRONE_RELEASE = 0.25;
    public static int DELAY_MS = 50;
    public static int CLAW_DELAY = 500;
    public static long ARM_DELAY= 40;
    public static int DIVISION_CONSTANT= 3;
    public static double TRACK_MULTIPLIER_FAST = 2;
    public static double ROTATION_MULTIPLIER_FAST = 4;
    public static double STRAFE_MULTIPLIER_FAST = 2;

    public static double TRACK_MULTIPLIER_SLOW = .5;
    public static double ROTATION_MULTIPLIER_SLOW = 1.5;
    public static double STRAFE_MULTIPLIER_SLOW = .5;
    public static double ARM_GROUND=0;
    public static double ARM_UP_AUTON=0;
    public static int TEAMPROP_X_THRESHOLD_LEFT = 80;
    public static int TEAMPROP_X_THRESHOLD_RIGHT = 270;

    public static double ZONE2_PUSH = 41;
    public static int BACK_X = 3;
    public static int STRAFE_DISTANCE_FAR = 86;
    public static int STRAFE_DISTANCE_NEAR = 34;
    public static int BEFORE_PARK_DELAY_MS = 500;

    public static int EXPOSURE = 0;
    public static int GAIN = 0;

    public static double DESIRED_DISTANCE = 1;
    public static double SPEED_GAIN  =  0.03;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    public static double STRAFE_GAIN =  -0.012 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    public static double TURN_GAIN   =  -0.015  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    public static double MAX_AUTO_SPEED  = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    public static double MAX_AUTO_TURN   = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    /*
     * These are motor constants that should be listed online for your motors.
     */
    public static final double TICKS_PER_REV = 145.1;
    public static final double MAX_RPM = 1150;

    /*
     * Set RUN_USING_ENCODER to true to enable built-in hub velocity control using drive encoders.
     * Set this flag to false if drive encoders are not present and an alternative localization
     * method is in use (e.g., tracking wheels).
     *
     * If using the built-in motor velocity PID, update MOTOR_VELO_PID with the tuned coefficients
     * from DriveVelocityPIDTuner.
     */
    public static final boolean RUN_USING_ENCODER = false;
    public static PIDFCoefficients MOTOR_VELO_PID = new PIDFCoefficients(0, 0, 0,
            getMotorVelocityF(MAX_RPM / 60 * TICKS_PER_REV));

    /*
     * These are physical constants that can be determined from your robot (including the track
     * width; it will be tune empirically later although a rough estimate is important). Users are
     * free to chose whichever linear distance unit they would like so long as it is consistently
     * used. The default values were selected with inches in mind. Road runner uses radians for
     * angular distances although most angular parameters are wrapped in Math.toRadians() for
     * convenience. Make sure to exclude any gear ratio included in MOTOR_CONFIG from GEAR_RATIO.
     */
    public static double WHEEL_RADIUS = 3.098; // in
    public static double GEAR_RATIO = 1;
    public static double TRACK_WIDTH = 9; // in

    //15.14 for ShrikeBot
    //about 9 for DeepBot

    /*
     * These are the feedforward parameters used to model the drive motor behavior. If you are using
     * the built-in velocity PID, *these values are fine as is*. However, if you do not have drive
     * motor encoders or have elected not to use them for velocity control, these values should be
     * empirically tuned.
     */
    public static double kV = 0.0162; // 1.0 / rpmToVelocity(MAX_RPM);
    public static double kA = 0.0027;
    public static double kStatic = 0.03;

    /*
     * These values are used to generate the trajectories for you robot. To ensure proper operation,
     * the constraints should never exceed ~80% of the robot's actual capabilities. While Road
     * Runner is designed to enable faster autonomous motion, it is a good idea for testing to start
     * small and gradually increase them later after everything is working. All distance units are
     * inches.
     */
    public static double MAX_VEL = 40;
    public static double MAX_ACCEL = 45;
    public static double MAX_ANG_VEL = Math.toRadians(119.30000730410198);
    public static double MAX_ANG_ACCEL = Math.toRadians(180);

    /*
     * Adjust the orientations here to match your robot. See the FTC SDK documentation for details.
     */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_FACING_DIR =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_FACING_DIR =
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN;


    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public static double rpmToVelocity(double rpm) {
        return rpm * GEAR_RATIO * 2 * Math.PI * WHEEL_RADIUS / 60.0;
    }

    public static double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
