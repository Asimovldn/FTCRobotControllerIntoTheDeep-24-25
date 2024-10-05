package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Outtake
{
    DcMotorEx leftSlide;
    DcMotorEx rightSlide;

    Servo leftServo;
    Servo rightServo;

    double inLeftPosition = 1.0;
    double inRightPosition = 0.0;

    double outLeftPosition = 0.5;
    double outRightPosition = 0.5;

    public enum OuttakePosition
    {
        OUT, IN
    };

    public Outtake(HardwareMap hardwareMap)
    {
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_outtake_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_outtake_slide");

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "left_outtake_servo");
        rightServo = hardwareMap.get(Servo.class, "right_outtake_servo");
    }

    public void Move(double power)
    {
        leftSlide.setPower(power);
        rightSlide.setPower(power);
    }

    public void MoveServo(OuttakePosition outtakePosition)
    {
        switch (outtakePosition)
        {
            case IN:
                leftServo.setPosition(inLeftPosition);
                rightServo.setPosition(inRightPosition);
                break;

            case OUT:
                leftServo.setPosition(outLeftPosition);
                rightServo.setPosition(outRightPosition);
                break;
        }
    }

    public double[] getCurrentSlidePosition()
    {
        return new double[] {leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition()};
    }
}
