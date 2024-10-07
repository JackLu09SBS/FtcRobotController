package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.geometry.Rotation2d;

public class SwerveModuleState {
    public final double speedMetersPerSecond;
    public final Rotation2d angle;

    public SwerveModuleState(double speedMeterPerSecond, Rotation2d angle) {
        this.speedMetersPerSecond = speedMeterPerSecond;
        this.angle = angle;
    }
}
