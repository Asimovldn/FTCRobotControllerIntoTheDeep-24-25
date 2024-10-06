package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auxiliar.Controls.TrapezoidalMotionProfile;
import org.firstinspires.ftc.teamcode.Auxiliar.DriveConstants;
import org.firstinspires.ftc.teamcode.Auxiliar.MecannumDriveHandler;
import org.firstinspires.ftc.teamcode.Auxiliar.Vector2D;

@Config
@TeleOp
public class TesteMotionProfile extends LinearOpMode
{
    public static double distance = 100;
    TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(DriveConstants.MAX_VEL, DriveConstants.MAX_ACCEL, distance);

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("pos", 0);
        telemetry.addData("vel", 0);
        telemetry.addData("accel", 0);
        telemetry.update();

        MecannumDriveHandler drive = new MecannumDriveHandler(hardwareMap, telemetry, this);


        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        waitForStart();

        timer.reset();


        while (opModeIsActive()) {
            drive.MoveOnStraightLine(new Vector2D(0, distance));
            drive.MoveOnStraightLine(new Vector2D(0, -distance));
        }

    }
}
