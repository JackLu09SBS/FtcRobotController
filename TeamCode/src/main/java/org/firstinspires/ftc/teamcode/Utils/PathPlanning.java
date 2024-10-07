package org.firstinspires.ftc.teamcode.Utils;

public class PathPlanning {
    private SensorData sensorData;
    private ObstacleAvoidance obstacleAvoidance;

    public PathPlanning(SensorData sensorData, ObstalceAvoidance obstacleAvoidance) {
        this.sensorData = sensorData;
        this.obstacleAvoidance = obstacleAvoidance;
    }

    public void planPath(SwerveDrive swerveDrive, double currentX, double currentY) {
        obstacleAvoidance.avoidObstacle(swerveDrive, currentX, currentY);

        swerveDrive.move(1, 0, 0);
    }
}
