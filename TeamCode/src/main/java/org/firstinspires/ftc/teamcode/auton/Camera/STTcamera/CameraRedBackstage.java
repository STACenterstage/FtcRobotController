package org.firstinspires.ftc.teamcode.auton.Camera.STTcamera;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotParts.Arm;
import org.firstinspires.ftc.teamcode.robotParts.Drivetrain;


@Autonomous(name = "CameraRedBackstage")
public class CameraRedBackstage extends LinearOpMode {

    newAutonMethods methods = new newAutonMethods(this);
    OpenCVTrussIsLeft camera = new OpenCVTrussIsLeft(this);

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm1;
    private Servo servoMoveGripper;
    private Servo servoIntakeL; // dicht is 0
    private Servo servoIntakeR; // dicht is 1
    private Servo servoChopstickL;
    private Servo servoChopstickR;


    double power = .3;

    Drivetrain.drivetrain drivetrain = new Drivetrain.drivetrain();
    Arm arm = new Arm();

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotorEx.class, "left_front");
        rightFront = map.get(DcMotorEx.class, "right_front");
        leftBack = map.get(DcMotorEx.class, "left_back");
        rightBack = map.get(DcMotorEx.class, "right_back");
        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");


        servoMoveGripper = hardwareMap.get(Servo.class, "servoMoveGripper");
        servoIntakeL = hardwareMap.get(Servo.class, "servoIntakeL");
        servoIntakeR = hardwareMap.get(Servo.class, "servoIntakeR");
        servoChopstickL = hardwareMap.get(Servo.class, "servoChopstickL");
        servoChopstickR = hardwareMap.get(Servo.class, "servoChopstickR");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoChopstickL.setPosition(0);
        servoChopstickR.setPosition(1);
        servoIntakeL.setPosition(0);
        servoIntakeR.setPosition(1);
        servoMoveGripper.setPosition(.82);

    }

    public void runOpMode() {
        methods.init(hardwareMap);

        methods.calibrateEncoders();
        camera.findScoringPosition();

        init(hardwareMap);
        arm.init(hardwareMap);
        drivetrain.init(hardwareMap);

        waitForStart();

        if (opModeIsActive()) {
            int finalPos = camera.pos;
            if (finalPos == 0) {
                servoChopstickL.setPosition(0);
                servoChopstickR.setPosition(1);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(.82);
                sleep(50);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1870);

                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(1200);

                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(640);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                servoIntakeR.setPosition(0);
                sleep(200);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(4000);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(200);

                /*leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(500);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(200);*/

                //naar voren nadat tegen bord aan
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(550);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(200);

                telemetry.addData("ArmPos",arm.ArmPos());
                telemetry.update();

                //arm omhoog
                while (arm.ArmPos() < 2900){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                sleep(200);

                servoMoveGripper.setPosition(0.000275*arm.ArmPos()-0.75);
                telemetry.addData("ArmPos",arm.ArmPos());
                telemetry.update();
                sleep(400);

                //laat gele pixel los
                servoChopstickL.setPosition(1);
                servoChopstickR.setPosition(0);
                sleep(800);


                //alles uit
                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(2000);

                //klein stukje naar voren
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(1000);

                //Klein stukje naar links
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(3000);

                //Arm in
                while (arm.ArmPos() > 500){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                sleep(100);

                servoMoveGripper.setPosition(.745);
                telemetry.addData("ArmPos",arm.ArmPos());
                telemetry.update();
                sleep(400);

                //Naar achteren rijden
                arm1.setPower(0);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(3000);

                //alles uit
                leftFront.setPower(-0);
                rightFront.setPower(-0);
                leftBack.setPower(-0);
                rightBack.setPower(-0);
                sleep(400);

                arm1.setPower(0);
                servoMoveGripper.setPosition(1);



            } else if (finalPos == 1) {
                servoChopstickL.setPosition(1);
                servoChopstickR.setPosition(0);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(.82);
                sleep(50);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(3100);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                servoIntakeR.setPosition(0);
                sleep(200);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(250);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

            } else if (finalPos == 2){
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(2500);

                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(1250);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                servoIntakeR.setPosition(0);
                sleep(200);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(250);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);

            }
            sleep(30000);


        }
    }
}

