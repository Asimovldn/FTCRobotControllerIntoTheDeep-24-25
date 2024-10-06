package org.firstinspires.ftc.teamcode.Auxiliar;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Auxiliar.Controls.FeedForward;
import org.firstinspires.ftc.teamcode.Auxiliar.Controls.TrapezoidalMotionProfile;

@Config
public class MecannumDriveHandler
{
    DcMotorEx lBD;
    DcMotorEx lFD;
    DcMotorEx rBD;
    DcMotorEx rFD;

    IMU imu;

    public static double kv = 0.035;
    public static double ka = 0.01;
    public static double ks = 0.01;



    Telemetry telemetry;

    LinearOpMode opMode;

    public MecannumDriveHandler(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode)
    {
        this.opMode = opMode;
        this.telemetry = telemetry;

        // Seta os motores à suas respectivas variáveis
        lBD = hardwareMap.get(DcMotorEx.class, "left_back");
        lFD = hardwareMap.get(DcMotorEx.class, "left_front");
        rBD = hardwareMap.get(DcMotorEx.class, "right_back");
        rFD = hardwareMap.get(DcMotorEx.class, "right_front");

        // Reseta os encoderes
        lBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rBD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Seta o modo dos motores
        lBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rBD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Inverte direção dos motores
        lBD.setDirection(DcMotorSimple.Direction.FORWARD);
        lFD.setDirection(DcMotorSimple.Direction.FORWARD);
        rBD.setDirection(DcMotorSimple.Direction.REVERSE);
        rFD.setDirection(DcMotorSimple.Direction.REVERSE);

        // Parar quando não há potência
        lBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rBD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Inicializa imu
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(DriveConstants.imuParameters);
        imu.resetYaw();

    }


    public void Analog(double x, double y, double r)
    {
        double denominator = Math.max(Math.abs (y) + Math.abs(x) + Math.abs(r), 1);

        lBD.setPower ( (y - x + r) / denominator);
        lFD.setPower ( (y + x + r) / denominator);
        rFD.setPower ( (y - x - r) / denominator);
        rBD.setPower ( (y + x - r) / denominator);

    }

    public void FieldCentric(double x , double y , double r)
    {
        Vector2D vector = new Vector2D(x,y);
        double angle = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw());

        Vector2D rotVector = Vector2D.rotateVector(vector, -angle);

        Analog(rotVector.x, rotVector.y, r );

    }

    public void MoveOnStraightLine(Vector2D finalPos)
    {
        double distance = finalPos.magnitude;

        TrapezoidalMotionProfile motionProfile = new TrapezoidalMotionProfile(DriveConstants.MAX_VEL,
                DriveConstants.MAX_ACCEL, distance);

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);


        while (motionProfile.isBusy && !opMode.isStopRequested())
        {
            FeedForward ffWheels = new FeedForward(kv,ka,ks);

            double[] profileValues = motionProfile.calculateMotionProfile(timer.time());

            Vector2D direction = Vector2D.normalizeVector(finalPos);

            double[] robotVelocities = new double[] {direction.y * profileValues[1], direction.x * profileValues[1], 0}; // colocar velocidade angular
            double[] robotAccels = new double[] {direction.y * profileValues[2], direction.x * profileValues[2], 0}; // colocar velocidade angular

            double[] wheelVelocities = MecannumWheelKinematics.inverseKinematics(robotVelocities);
            double[] wheelAccels = MecannumWheelKinematics.inverseKinematics(robotAccels);

            // ordem: lBD, lFD, rBD, rFD

            double[] motorPowers = new double[]
                    {
                            ffWheels.calculate(wheelVelocities[3], wheelAccels[3], (int)Math.signum(wheelVelocities[3])),
                            ffWheels.calculate(wheelVelocities[0], wheelAccels[0], (int)Math.signum(wheelVelocities[0])),
                            ffWheels.calculate(wheelVelocities[2], wheelAccels[2], (int)Math.signum(wheelVelocities[2])),
                            ffWheels.calculate(wheelVelocities[1], wheelAccels[1], (int)Math.signum(wheelVelocities[1]))
                    };

            setMotorPowers(motorPowers[0], motorPowers[1], motorPowers[2], motorPowers[3]);


            telemetry.addData("lBD", lBD.getVelocity(AngleUnit.RADIANS));


            telemetry.addData("alvo 0", wheelVelocities[3]);
            telemetry.addData("alvo 1", wheelVelocities[0]);
            telemetry.addData("alvo 2", wheelVelocities[2]);
            telemetry.addData("alvo 3", wheelVelocities[1]);

            telemetry.addData("time", timer.time());
            telemetry.update();

        }

    }

    void setMotorPowers(double lBDPower, double lFDPower, double rBDPower, double rFDPower)
    {
        lBD.setPower(lBDPower);
        lFD.setPower(lFDPower);
        rBD.setPower(rBDPower);
        rFD.setPower(rFDPower);
    }
}
