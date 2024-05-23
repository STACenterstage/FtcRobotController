package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robotParts.Arm;

@TeleOp(name = "FieldCentric_STAdrive",group = "TeleOp")
public class STAdrive_FieldCentric extends LinearOpMode {
    Arm arm = new Arm();
    boolean climbMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorSimple leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        DcMotorSimple rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        DcMotorSimple leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        DcMotorSimple rightBack = hardwareMap.get(DcMotorEx.class, "right_back");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.init(hardwareMap);

        telemetry.addData("ArmPos", arm.ArmPos());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            boolean holdChange = gamepad2.dpad_down;
            boolean holdOff = gamepad2.dpad_up;

            boolean servoIntakeOn = gamepad1.a;
            boolean servoIntakeOff = gamepad1.b;

            boolean servoVliegtuigTrigger = gamepad1.left_bumper;

            double armPower = (gamepad2.right_trigger - gamepad2.left_trigger);

            boolean chopstickOn = gamepad2.y;
            boolean chopstickLOff = gamepad2.right_bumper;
            boolean chopstickROff = gamepad2.left_bumper;


            if (chopstickOn) {
                arm.chopstickL(0.45);
                arm.chopstickR(0.32);
            } else {
                if (chopstickLOff) {
                    arm.chopstickL(0.61);
                }
                if (chopstickROff) {
                    arm.chopstickR(0.19);
                }
            }

            boolean intakeOn = gamepad1.a;
            boolean intakeOff = gamepad1.b;

            if (intakeOn) {
                arm.intakeL(1);
                arm.intakeR(0);
            } else if (intakeOff) {
                arm.intakeL(0);
                arm.intakeR(1);
            }

            if (servoIntakeOn) {
                arm.servoIntake(1);
            } else if (servoIntakeOff) {
                arm.servoIntake(0);
            }

            if (servoVliegtuigTrigger) {
                arm.servoVliegtuigHouder(1);
                sleep(80);
                arm.servoVliegtuig(1);
                sleep(920);
                arm.servoVliegtuig(0);
                arm.servoVliegtuigHouder(0);
            }

            if (gamepad2.dpad_down){
                climbMode = true;
            }else if (gamepad2.dpad_up){
                climbMode = false;
            }

            if (climbMode){
                arm.moveGripper(0.12);
            } else if (arm.ArmPos() < 400) {
                arm.moveGripper(0.245 + gamepad2.left_stick_x * 0.02);
            } else if (arm.ArmPos() > 2300) {
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.18 + gamepad2.left_stick_x * 0.06);
            } else {
                arm.moveGripper(.0003 * (arm.ArmPos() - 400) + 0.26);
            }

            IMU imu = hardwareMap.get(IMU.class,"imu");
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
            ));
            imu.initialize(parameters);

            waitForStart();

            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            double max  = Math.max(Math.abs(lx) + Math.abs(ly) + Math.abs(rx), 1);
            double power = 1 - (0.5*gamepad1.right_trigger);

            double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double adjustedLx = -ly * Math.sin(heading) + lx * Math.cos(heading);
            double adjustedLy = ly * Math.cos(heading) + lx * Math.sin(heading);

            leftFront.setPower(((adjustedLy + adjustedLx + rx) / max) * power);
            leftBack.setPower(((adjustedLy - adjustedLx + rx) / max) * power);
            rightFront.setPower(((adjustedLy - adjustedLx - rx) / max) * power);
            rightBack.setPower(((adjustedLy + adjustedLx - rx) / max) * power);



            telemetry.addData("climbMode", climbMode);
            telemetry.addData("IMU", imu);
            telemetry.addData("ArmPos", arm.ArmPos());
            arm.move(armPower);
            telemetry.update();

        }
    }
}
