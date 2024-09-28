package org.firstinspires.ftc.teamcode.Utils;

public class demokinematics {
    // Attributes for position and orientation
    private double x;           // current x position (initially set to 0.0)
    private double y;           // current y position (initially set to 0.0)
    private double theta;       // current orientation in radians (initially set to 0.0)

    // Converted Parameters
    private double wheelRadius;     // Radius of the Wheels (37.54mm -> 0.03754m)
    private double wheelBase;       // distance between enter to wheel (270mm ->.27m)
    private double wheelTrack;      // distance between left adn right wheels (135mm ->.135m)
    private double dt;              // time steps for the simulation (0.1second)

    // Gear ratio prameters
    private double inputGear;       // input gear ratio (52)
    private double outputGear;      // output gear ratio (6)
    private double gearRatio;       // Gear ratio (52:6)

    // Final RPM value for motor
    private double finalMotorRPM;   // Final RPM value for motor (669 RPM)

    public demokinematics(double dt) {
        this.wheelRadius = 0.03754;     // Set the wheel radius in meters
        this.wheelBase = 0.27;          // Set the wheel base in meters
        this.wheelTrack = 0.135;            // Set the wheel track distance (optional)
        this.dt = dt;                   // Set the time step
        this.x = 0.0;                   // Initialize x position to 0.0
        this.y = 0.0;                   // Initialize y position to 0.0
        this.theta = 0.0;               // Initialize orientation to 0.0
        this.inputGear = 52.0;          // Input gear
        this.outputGear = 6.0;          // Output gear
        this.gearRatio = inputGear / outputGear;    // Calculate gear ratio
        this.finalMotorRPM = 669.0;     // Set the final RPM value
    }

    // Methods to convert Motor RPM to effective wheel RPM
    private double calculateWheel1RPM(double motorRPM) {
        // Calculate effective wheel RPM based on gear ratio
        return motorRPM * (outputGear / inputGear);
    }
    // Methods to update position and orientation based on wheel speeds and angles
    public void updatePosition(double[] wheelSpeed, double[] wheelAngles) {
        // calculate the average speed and direction
        double vx = 0.0;
        double vy = 0.0;

        for (int i = 0; i < wheelSpeed.length; i++) {
            // Calculate the x and y contributions of each wheel
            vx += wheelSpeeds[i] * Math.cos(wheelAngles[i]);
            vy += wheelSpeeds[i] * Math.sin(wheelAngles[i]);
        }

        // Update position base on average speed
        x += vx * dt;
        y += vy * dt;

        // Calculate the robot's new orientation
        theta = Math.atan2(vy, vx);
        System.out.printf("Position updated to: (%.2f, %.2f), Orientatio: %.2f radians\n", x, y, theta);
    }

    // Methods to move the robot with specific speeds and angles
    public void moveForward(double forward, double strafe, double rotation) {
        // Convert input speeds and rotation to wheel speeds and angles
        double[] wheelSpeeds = new double[4]; // 4 wheels
        double[] wheelAngles = new double[4]; // 4 angles

        // Calculate the speeds for each weheel based on forward, strafe, and rotation inputs
        wheelSpeeds[0] = forward + strafe + rotation;    // Front-left wheel
        wheelSpeeds[1] = forward - strafe - rotation;    // Front-right wheel
        wheelSpeeds[2] = forward - strafe + rotation;    // Back-right wheel
        wheelSpeeds[3] = forward + strafe - rotation;    // Back-left wheel

        // Calculate angles for each wheel
        for (int i = 0; i < wheelAngles.length; i++) {
            wheelAngles[i] = Math.atan2(wheelSpeeds[i], wheelSpeeds[i]);  // Calculate angle for each wheel
        }

        // Update position and orientation based on wheel speeds and angles
        updatePosition(wheelSpeeds, wheelAngles);
    }

    // Methods to move the robot forward a certain distance using final RPM
    public void moveForward(double distance) {
        double targetSpeed = finalMotorRPM * (outputGear / inputGear);   //Effective wheel RPM for target speed
        double rpmLeft = finalMotorRPM;   // Set left wheel RPM to final RPM
        double rpmRight = finalMotorRPM;  // Set right wheel RPM to final RPM

        double targetTime = distance / ((targetSpeed * 2 * Math.PI * wheelRadius / 60));    // Time to reach the target distance
        double elapsedTime = 0.0;   // Initialize elapsed time

        while (elapsedTime < targetTime) {
            // Update position while moving forward
            updatePosition(new double[]{rpmLeft, rpmRight, rpmLeft, rpmRight}, new double[]{0, 0, 0, 0});
            elapsedTime += dt;   // Increment the elapsed time
        }

        // Stop the robot
        updatePosition(new double[] {0, 0, 0, 0}. new double[]{0, 0, 0, 0});
    }

    // Method to turn the robot by a specified angle (in radinas)
    public void turn(double angle) {
        double targetAngularSpeed = Math.PI / 4;   // Angular speed in radinas/s
        double rpmLeft = -targetAngularSpeed * (wheelBase / 2) * 60 / (2 * Math.PI * wheelRadius) * gearRatio;   // Left wheel RPM for turning
        double rpmRight = targetAngularSpeed * (wheelBase / 2) * 60 / (2 * Math.PI * wheelRadius) * gearRatio;   // Right wheel RPM for turning

        double targetTheta = (theta + angle + 2 * Math.PI) % (2 * Math.PI);   //Normalize target angle

        while (Math.abs(theta - targetTheta) > 0.01) {
            //update position while turning
            updatePosition(new double[]{rpmLeft, rpmRight, rpmLeft, rpmRight}, new double[]{0, 0, 0, 0});
        }

        // Stop the robot after turning
        updatePosition(new double[]{0, 0, 0, 0}, new double[]{0, 0, 0, 0});
    }

    // Getter methods for position and orientation
    public double getX() {
        return x;    // Return current X position
    }

    public double getY() {
        return y;    // Return current Y position
    }

    public double getTheta() {
        return theta;   // REturn current orientation
    }
    }