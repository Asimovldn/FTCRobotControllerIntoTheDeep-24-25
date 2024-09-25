package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake
{
    double outLeftPosition = 0.0;
    double outRightPosition = 0.0;
    double inLeftPosition = 0.0;
    double inRightPosition = 0.0;

    Servo leftWrist;
    Servo rightWrist;

    public enum IntakePosition
    {
        OUT, IN
    }


    public Intake(HardwareMap hardwareMap)
    {
        leftWrist = hardwareMap.get(Servo.class, "left_wrist_intake");
        rightWrist = hardwareMap.get(Servo.class, "right_wrist_intake");

    }

    public void Move(IntakePosition intakePosition)
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
}
