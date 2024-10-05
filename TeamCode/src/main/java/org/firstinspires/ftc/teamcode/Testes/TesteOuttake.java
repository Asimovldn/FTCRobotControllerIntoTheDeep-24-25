package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Auxiliar.Outtake;

@TeleOp
public class TesteOuttake extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Outtake outtake = new Outtake(hardwareMap);
        waitForStart();

        while (opModeIsActive())
        {
            if (gamepad1.a)
            {
                outtake.MoveServo(Outtake.OuttakePosition.IN);
            }

            if (gamepad1.b)
            {
                outtake.MoveServo(Outtake.OuttakePosition.OUT);
            }

            outtake.Move(-gamepad1.left_stick_y / 2);

            double[] slidePos = outtake.getCurrentSlidePosition();
            telemetry.addData("left slide", slidePos[0]);
            telemetry.addData("right_slide", slidePos[1]);

            telemetry.addData("goalPos,", outtake.getCurrentGoalPosition());
            telemetry.update();
        }
    }
}
