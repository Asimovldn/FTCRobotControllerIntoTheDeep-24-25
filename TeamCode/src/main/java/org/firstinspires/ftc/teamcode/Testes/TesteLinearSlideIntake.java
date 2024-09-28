package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auxiliar.Intake;

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
