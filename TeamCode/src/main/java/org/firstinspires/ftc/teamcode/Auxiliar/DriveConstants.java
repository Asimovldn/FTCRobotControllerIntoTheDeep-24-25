package org.firstinspires.ftc.teamcode.Auxiliar;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

@Config
public class DriveConstants
{
    public static IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP)
    );

    public static double MAX_VEL = 30; // CM/S
    public static double MAX_ACCEL = 30; // CM/S^2

    public static double trackWidth; // CM
    public static double wheelRadius; // CM
}