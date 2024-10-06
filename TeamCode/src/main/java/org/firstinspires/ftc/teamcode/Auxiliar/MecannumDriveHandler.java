package org.firstinspires.ftc.teamcode.Auxiliar;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auxiliar.Controls.TrapezoidalMotionProfile;

public class MecannumDriveHandler
{
    DcMotorEx lBD;
    DcMotorEx lFD;
    DcMotorEx rBD;
    DcMotorEx rFD;

    IMU imu;

    public MecannumDriveHandler(HardwareMap hardwareMap)
    {
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

        while (motionProfile.isBusy)
        {
            double[] profileValues = motionProfile.calculateMotionProfile(timer.time());

            Vector2D direction = Vector2D.normalizeVector(finalPos);

            double[] robotVelocities = new double[] {direction.y * profileValues[1], direction.x * profileValues[1], 0}; // colocar velocidade angular
            double[] robotAccels = new double[] {direction.y * profileValues[2], direction.x * profileValues[2], 0}; // colocar velocidade angular

            double[] wheelVelocities = MecannumWheelKinematics.inverseKinematics(robotVelocities);
            double[] wheelAccels = MecannumWheelKinematics.inverseKinematics(robotAccels);

            // ordem: lBD, lFD, rBD, rFD
            double[] motorPowers; // colocar no FF
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
