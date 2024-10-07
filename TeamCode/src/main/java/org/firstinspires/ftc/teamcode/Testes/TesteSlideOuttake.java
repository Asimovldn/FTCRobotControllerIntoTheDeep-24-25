package org.firstinspires.ftc.teamcode.Testes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class TesteSlideOuttake extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"a");
        DcMotorEx motor1 = hardwareMap.get(DcMotorEx.class, "b");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive())
        {
            if (motor.getCurrentPosition() < 3000)
                motor.setVelocity(45 * -gamepad1.left_stick_y, AngleUnit.DEGREES);
            else
                motor.setPower(0.0);

            if (motor1.getCurrentPosition() < 3000)
                motor1.setVelocity(45 * -gamepad1.left_stick_y, AngleUnit.DEGREES);
            else
                motor1.setPower(0.0);
            telemetry.addData("a", motor.getCurrentPosition());
            telemetry.addData("b", motor1.getCurrentPosition());
            telemetry.update();
        }
    }
}
