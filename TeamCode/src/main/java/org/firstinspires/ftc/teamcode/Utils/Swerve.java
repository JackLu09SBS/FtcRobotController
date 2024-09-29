package org.firstinspires.ftc.teamcode.Utils;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

public class Swerve
{
    // Locations for the swerve drive modules
    // relative to the robot center.
    Translation2d m_frontLeftLocation =
            new Translation2d(0.381, 0.381);
    Translation2d m_frontRightLocation =
            new Translation2d(0.381, -0.381);
    Translation2d m_backLeftLocation =
            new Translation2d(-0.381, 0.381);
    Translation2d m_backRightLocation =
            new Translation2d(-0.381, -0.381);

    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics
            (
                    m_frontLeftLocation, m_frontRightLocation,
                    m_backLeftLocation, m_backRightLocation
            );
    // Example chassis speeds: 1 meter per second forward, 3 meters
// per second to the left, and rotation at 1.5 radians per second
// counterclockwise.
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 3.0, 1.5);

    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

    // Example module states
    SwerveModuleState frontLeftState =
            new SwerveModuleState(23.43, Rotation2d.fromDegrees(-140.19));
    SwerveModuleState frontRightState =
            new SwerveModuleState(23.43, Rotation2d.fromDegrees(-39.81));
    SwerveModuleState backLeftState =
            new SwerveModuleState(54.08, Rotation2d.fromDegrees(-109.44));
    SwerveModuleState backRightState =
            new SwerveModuleState(54.08, Rotation2d.fromDegrees(-70.56));

    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
            frontLeftState, frontRightState, backLeftState, backRightState
    );

    // Getting individual speeds
    double forward = chassisSpeeds.vxMetersPerSecond;
    double sideways = chassisSpeeds.vyMetersPerSecond;
    double angular = chassisSpeeds.omegaRadiansPerSecond;
}
