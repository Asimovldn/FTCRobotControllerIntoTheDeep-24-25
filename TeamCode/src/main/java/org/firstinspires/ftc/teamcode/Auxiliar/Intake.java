package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake
{
    double outLeftPosition = 1.0;
    double outRightPosition = 0.0;
    double inLeftPosition = 0.0;
    double inRightPosition = 1.0;;

    Servo leftWrist;
    Servo rightWrist;

    CRServo intakeServo;

    DcMotorEx intakeSlideMotor;

    public int linearSlideLimit = 1600; // Ticks || 1760 o maximo



    public enum IntakePosition
    {
        OUT, IN
    }


    public Intake(HardwareMap hardwareMap)
    {
        leftWrist = hardwareMap.get(Servo.class, "left_wrist_intake");
        rightWrist = hardwareMap.get(Servo.class, "right_wrist_intake");

        intakeServo = hardwareMap.get(CRServo.class, "intake_servo");

        intakeSlideMotor = hardwareMap.get(DcMotorEx.class, "slide_intake");

        intakeSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        intakeSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MoveServo(IntakePosition intakePosition)
    {
        switch (intakePosition)
        {
            case IN:
                leftWrist.setPosition(inLeftPosition);
                rightWrist.setPosition(inRightPosition);
                break;

            case OUT:
                leftWrist.setPosition(outLeftPosition);
                rightWrist.setPosition(outRightPosition);
                break;
        }
    }

    public void setIntakePower(double power)
    {
        intakeServo.setPower(power);
    }

    public void setSlidePower(double power)
    {
        if (intakeSlideMotor.getCurrentPosition() >= linearSlideLimit && power > 0.0) {
            intakeSlideMotor.setPower(0.0);
            return;
        }

        intakeSlideMotor.setPower(power);
    }

    public void resetSlideEncoder()
    {
        intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getSlidePosition()
    {
        return intakeSlideMotor.getCurrentPosition();
    }
}
