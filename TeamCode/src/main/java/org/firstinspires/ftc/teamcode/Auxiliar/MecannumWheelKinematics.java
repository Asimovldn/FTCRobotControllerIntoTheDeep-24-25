package org.firstinspires.ftc.teamcode.Auxiliar;

public class MecannumWheelKinematics
{
    // recebe na ordem: vy, vx, angular
    // returna na ordem: esquerda frontal, direita frontal, direita trazeira, esquerda trazeira

    public static double[] inverseKinematics(double[] robotVelocities)
    {
        double frontLeft = (robotVelocities[0] - robotVelocities[1] -
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        double frontRight = (robotVelocities[0] + robotVelocities[1] +
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        double backRight = (robotVelocities[0] - robotVelocities[1] +
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        double backLeft = (robotVelocities[0] + robotVelocities[1] -
                DriveConstants.trackWidth * robotVelocities[2]) / DriveConstants.wheelRadius;

        return new double[] {frontLeft, frontRight, backRight, backLeft};
    }
}