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

    int maxSlideVel = 30; // Ticks/frame

    int currentSlidesPosition = 0;

    public enum OuttakePosition
    {
        OUT, IN
    };

    PIDControl leftCorrectionSlideControl = new PIDControl(0.02,0.002,0);
    PIDControl rightCorrectionSlideControl = new PIDControl(0.02,0.002,0);

    public Outtake(HardwareMap hardwareMap)
    {
        leftSlide = hardwareMap.get(DcMotorEx.class, "left_outtake_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_outtake_slide");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "left_outtake_servo");
        rightServo = hardwareMap.get(Servo.class, "right_outtake_servo");
    }

    public void Move(float power)
    {
        if (power >= 0)
        {
            if (currentSlidesPosition >= 0)
                currentSlidesPosition += Math.round(power * maxSlideVel);
            else
                currentSlidesPosition = 0;
        } else {
            if (currentSlidesPosition >= 0)
                currentSlidesPosition += Math.round(power * 40);
            else {
                currentSlidesPosition = 0;
            }
        }

        double leftCorrection = leftCorrectionSlideControl.calculate(currentSlidesPosition, leftSlide.getCurrentPosition());
        double rightCorrection = rightCorrectionSlideControl.calculate(currentSlidesPosition, rightSlide.getCurrentPosition());

        if (currentSlidesPosition < 5 && power < 0)
            return;

        leftSlide.setPower(leftCorrection);
        rightSlide.setPower(rightCorrection);

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

    public double getCurrentGoalPosition()
    {
        return currentSlidesPosition;
    }
}
