package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.Intake;

@TeleOp
public class TesteLinearSlideIntake extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Intake intake = new Intake(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {
            intake.setSlidePower(-gamepad1.left_stick_y);
        }
    }
}
