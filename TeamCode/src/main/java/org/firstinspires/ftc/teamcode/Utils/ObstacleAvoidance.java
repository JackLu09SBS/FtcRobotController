package org.firstinspires.ftc.teamcode.Utils;

public class ObstacleAvoidance {
    private double thresholdDistance;
    private SensorData sensorData;

    public ObstacleAvoidance(SensorData sensorData, double thresholdDistance) {
        this.sensorData = sensorData;
        this.thresholdDistance = thresholdDistance;
    }

    public void avoidObstacle(SwerveDrive swerveDrive, double currentX, double currentY) {
        if (sensorData.isObstacleDetected()) {
            double obstacleX = currentX + sensorData.getDistance();
            double obstacleY = currentY;

            double distanceToObstacle = Math.sqrt(Math.pow(obstacleX - currentX, 2) + Math.pow(obstacleY - currentY, 2));

            double angleToObstacle = Math.atan2(obstacleY - currentY, obstacleX - currentX);
            double newDirection = angleToObstacle + Math.PI / 2;

            double forward = Math.cos(newDirection);
            double sideways = Math.sin(newDirection);
            swerveDrive.move(forward, sideways, 0);
        } else {
            swerveDrive.move(1, 0, 0);
        }
    }
}
