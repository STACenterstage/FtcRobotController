package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "FieldCentric") 
public class FieldCentric extends LinearOpMode {
    @Override
    public void runOpMode(){
        DcMotorSimple leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        DcMotorSimple rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        DcMotorSimple leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        DcMotorSimple rightBack = hardwareMap.get(DcMotorEx.class, "right_back");

        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        IMU imu = hardwareMap.get(IMU.class,"imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        waitForStart();
        
        while (opModeIsActive()){
            double lx = gamepad1.left_stick_x;
            double ly = gamepad1.left_stick_y;
            double rx = 0.7*gamepad1.right_stick_x;
            
            double max  = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
            double power = 0.5 + (0.6*gamepad1.right_trigger);

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            leftFront.setPower(((adjustedLy + adjustedLx + rx) / max) * power);  //TODO operatoren checken
            leftBack.setPower(((adjustedLy - adjustedLx + rx) / max) * power);
            rightFront.setPower(((adjustedLy - adjustedLx - rx) / max) * power);
            rightBack.setPower(((adjustedLy + adjustedLx - rx) / max) * power);
        }
    }
}


