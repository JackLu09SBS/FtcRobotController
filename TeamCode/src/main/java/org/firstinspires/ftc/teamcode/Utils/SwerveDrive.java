package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

public class SwerveDrive {
    // 로봇 중심에 대한 스웜 드라이브 모듈의 위치
    private final Translation2d m_frontLeftLocation = new Translation2d(0.270, 0.270);
    private final Translation2d m_frontRightLocation = new Translation2d(0.270, -0.270);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.270, 0.270);
    private final Translation2d m_backRightLocation = new Translation2d(-0.270, -0.270);

    // 스웜 드라이브 모듈 정보 관리
    private final SwerveDriveKinematics m_kinematics;

    public SwerveDrive() {
        m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    }

    public void move(double forward, double sideways, double rotation) {
        ChassisSpeeds speeds = new ChassisSpeeds(forward, sideways, rotation);

        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        SwerveModule frontLeft = new SwerveModule();
        SwerveModule frontRight = new SwerveModule();
        SwerveModule backLeft = new SwerveModule();
        SwerveModule backRight = new SwerveModule();

        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }
}