package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.Outtake;


@TeleOp
public class TesteMotionProfileOuttakeSlide extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Outtake outtake = new Outtake(hardwareMap, this, telemetry);

        waitForStart();

        telemetry.addData("pos ", 0);
        telemetry.addData("vel ", 0);
        telemetry.addData("accel ", 0);
        telemetry.addData("motorVel", 0);
        telemetry.addData("motorPos,", 0);

        telemetry.update();

        while (!gamepad1.a)
        {
            if (isStopRequested())
            {
                break;
            }
        }

        outtake.MoveSlide(1500);

        while (opModeIsActive())
        {
            while (!gamepad1.a)
            {
                outtake.HoldPosition();
                if (isStopRequested())
                {
                    break;
                }
            }

            outtake.MoveSlide(-1000);

            while (!gamepad1.a)
            {
                outtake.HoldPosition();
                if (isStopRequested())
                {
                    break;
                }
            }

            outtake.MoveSlide(1000);
        }

    }
}
