package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class EénMotor {
    public static class drivetrain {

        //AlexDistanceSensorUtil distanceSensor = new AlexDistanceSensorUtil();
        private DcMotorEx Motor;


        public void init(HardwareMap map) {
            Motor = map.get(DcMotorEx.class, "left_front");


            Motor.setDirection(DcMotorSimple.Direction.REVERSE);


            Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }

        public void drive(double forward, double right, double rotate) {
            double leftFrontPower = forward + right + rotate;
            double maxPower = 1.0;

            maxPower = Math.max(maxPower, Math.abs(leftFrontPower));


            Motor.setPower(leftFrontPower / maxPower);

        }
    }
}