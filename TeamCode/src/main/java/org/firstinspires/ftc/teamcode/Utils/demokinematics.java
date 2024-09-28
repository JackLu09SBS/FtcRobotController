package org.firstinspires.ftc.teamcode.Utils;

public class demokinematics {
    // Attributes for position and orientation
    private double x;           // current x position (initially set to 0.0)
    private double y;           // current y position (initially set to 0.0)
    private double theta;       // current orientation in radians (initially set to 0.0)

    // Converted Parameters
    private double wheelRadius;     // Radius of the Wheels (37.54mm -> 0.03754m)
    private double wheelBase;       // distance between enter to wheel (270mm ->.27m)
    private double dt;              // time steps for the simulation (0.1second)

    // Gear ratio prameters
    private double inputGear;       // input gear ratio (52)
    private double outputGear;      // output gear ratio (6)
    private double gearRatio;       // Gear ratio (52:6)

    // Final RPM value for motor
    private double finalMotorRPM;   // Final RPM value for motor (669 RPM)

    public demokinematics(double dt) {
        this.wheelRadius = 0.03754;
        this.wheelBase = 0.27;
        this.dt = dt;
        this.x = 0.0;
        this.y = 0.0;
        this.theta = 0.0;
        this.inputGear = 52.0;
        this.outputGear = 6.0;
        this.gearRatio = inputGear / outputGear;
        this.finalMotorRPM = 669.0;
    }

    // Methods to convert Motor RPM to effective wheel RPM
    private double calculateWheel1RPM(double motorRPM) {
        // Calculate effective wheel RPM based on gear ratio
        return motorRPM * (outputGear / inputGear);
    }
    // Methods to update position based on wheel speed in RPM
    public void updatePosition(double rpmLeft, double rpmRight) {
        // Convert Motor RPM to effective wheel RPM
        double vLeft = calculateWheel1RPM(rpmLeft);
        double vRight = calculateWheel1RPM(rpmRight);

        //convert RPM to meters per second
        vLeft = (vLeft * 2 * Math.PI * wheelRadius / 60);
        vRight = (vRight * 2 * Math.PI * wheelRadius / 60);

        double v = (vLeft + vRight) / 2;
        double omega = (vRight - vLeft) / wheelBase;

        x += v * Math.cos(theta) * dt;
        y += v * Math.sin(theta) * dt;
        theta += omega * dt;

        theta = ( theta + 2 * Math.PI) % (2 * Math.PI);
        System.out.printf("position updated to: (%.2f, %.2f), Orientation: %.2f radians\n", x, y, theta);
    }

    // Methods to move the robot forward a certain distance using final RPM
    public void moveForward(double distance) {
        double targetSpeed = finalMotorRPM * (outputGear / inputGear);
        double rpmLeft = finalMotorRPM;
        double rpmright = finalMotorRPM;

        double targetTime = distance / ((targetSpeed * 2 * Math.PI * wheelRadius / 60));
        double elapsedTime = 0.0;

        while (elapsedTime < targetTime) {
            updatePosition(rpmLeft, rpmright);
            elapsedTime += dt;
        }
        updatePosition(0, 0);
    }

    // Methods to turn the robot by a specified angle (in radians)
    public void turn(double angle) {
        double targetAngularSpeed= Math.PI / 4;
        double rpmLeft = -targetAngularSpeed * (wheelBase / 2) * 60 / (2 * Math.PI * wheelRadius) * gearRatio;
        double rpmRight = targetAngularSpeed * (wheelBase / 2) * 60 / (2 * Math.PI * wheelRadius) * gearRatio;

        double targetTheta = (theta + angle + 2 * Math.PI) % (2 * Math.PI);

        while (Math.abs(theta - targetTheta) > 0.01) {
            updatePosition(rpmLeft, rpmRight);
        }
        updatePosition(0, 0);

        // Getter methods for position and orientation
        public void getX() {
            return x;
        }

        public void getY() {
            return y;
        }

        public void getTheta() {
            return theta;
        }
    }
    }