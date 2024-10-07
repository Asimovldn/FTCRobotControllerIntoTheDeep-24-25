package org.firstinspires.ftc.teamcode.Auxiliar;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auxiliar.Controls.FeedForward;
import org.firstinspires.ftc.teamcode.Auxiliar.Controls.PIDControl;
import org.firstinspires.ftc.teamcode.Auxiliar.Controls.TrapezoidalMotionProfile;

@Config
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

    double MAX_VEL_OUTTAKE = 200; // Ticks/s
    double MAX_ACCEL_OUTTAKE = 200; // Ticks/s^2

    public enum OuttakePosition
    {
        OUT, IN
    };

    LinearOpMode opMode;
    Telemetry telemetry;

    public Outtake(HardwareMap hardwareMap, LinearOpMode opMode, Telemetry telemetry)
    {
        this.telemetry = telemetry;
        this.opMode = opMode;

        leftSlide = hardwareMap.get(DcMotorEx.class, "left_outtake_slide");
        rightSlide = hardwareMap.get(DcMotorEx.class, "right_outtake_slide");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftServo = hardwareMap.get(Servo.class, "left_outtake_servo");
        rightServo = hardwareMap.get(Servo.class, "right_outtake_servo");
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

    public void MoveSlide(double distance)
    {
        TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(MAX_VEL_OUTTAKE, MAX_ACCEL_OUTTAKE, distance);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

        while (motionProfile.isBusy && !opMode.isStopRequested())
        {
            double[] values = motionProfile.calculateMotionProfile(timer.time());

            leftSlide.setVelocity(values[1]);

            telemetry.addData("pos ", values[0]);
            telemetry.addData("vel ", values[1]);
            telemetry.addData("accel ", values[2]);
            telemetry.addData("motorVel", leftSlide.getVelocity());
            telemetry.addData("motorPos,", leftSlide.getCurrentPosition());

            telemetry.update();

        }
    }

    public double[] getCurrentSlidePosition()
    {
        return new double[] {leftSlide.getCurrentPosition(), rightSlide.getCurrentPosition()};
    }


}
