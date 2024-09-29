package org.firstinspires.ftc.teamcode.DriveAndLift;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

public class DriveConstants
{
    public static IMU.Parameters imuParameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)
    );

    public static double MAX_VEL; // CM/S
    public static double MAX_ACCEL; // CM/S^2

    public static double trackWidth; // CM
    public static double wheelRadius; // CM
}
