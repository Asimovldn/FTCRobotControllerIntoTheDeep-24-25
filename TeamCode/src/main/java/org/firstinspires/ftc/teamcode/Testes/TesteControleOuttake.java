package org.firstinspires.ftc.teamcode.Testes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.GamepadInOuttake;
import org.firstinspires.ftc.teamcode.Auxiliar.Outtake;

@TeleOp
public class TesteControleOuttake extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Outtake outtake = new Outtake(hardwareMap, this, telemetry);
        GamepadInOuttake gamepadInOuttake = new GamepadInOuttake(gamepad1, outtake);

        waitForStart();

        while (opModeIsActive())
        {
            gamepadInOuttake.update();
        }
    }
}
