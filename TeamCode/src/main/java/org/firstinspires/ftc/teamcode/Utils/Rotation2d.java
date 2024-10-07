package org.firstinspires.ftc.teamcode.Utils;

public class Rotation2d {
    private final double radians;

    private Rotation2d(double radians) {
        this.radians = radians;
    }

    public static Rotation2d fromDegrees(double degrees) {
        return new Rotation2d(Math.toRadians(degrees));
    }

    public double getRadians() {
        return radians;
    }
}
