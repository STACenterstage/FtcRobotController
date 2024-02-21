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

import java.lang.reflect.Method;


@Autonomous(name = "CameraRedBackstage")
public class CameraRedBackstage extends LinearOpMode {

    EigenOdometry methods = new EigenOdometry(this);
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

        servoChopstickL.setPosition(0.61);
        servoChopstickR.setPosition(0.32);
        servoIntakeL.setPosition(0);
        servoIntakeR.setPosition(1);
        servoMoveGripper.setPosition(0.2);

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
            time = System.currentTimeMillis();
            if (finalPos == 0) {

                methods.driveDean(0,67);
                methods.rotateToHeading(-90);
                methods.driveDean(0,-20);
                methods.Stop();
                servoIntakeR.setPosition(0.3);
                sleep(300);
                methods.driveDean(-5,97);
                methods.Stop();
                servoIntakeR.setPosition(1);
                sleep(300);
                while (arm.ArmPos() < 3250 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.2);
                sleep(800);
                servoChopstickR.setPosition(0.19);
                sleep(800);
                methods.Stop();
                while (arm.ArmPos() > 400 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickR.setPosition(0.19);
                sleep(300);
                methods.driveDean(80,15);
                methods.Stop();
                terminateOpModeNow();

/*
                servoChopstickL.setPosition(0.47);
                servoChopstickR.setPosition(0.3);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(0);
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
                sleep(3000);

                servoIntakeR.setPosition(0);
                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(-power);
                sleep(500);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1000);



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
                sleep(200);* /

                //naar voren nadat tegen bord aan
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(620);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(200);

                telemetry.addData("ArmPos",arm.ArmPos());
                telemetry.update();

                //arm omhoog
                while (arm.ArmPos() < 3020 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                sleep(200);

                servoMoveGripper.setPosition((0.000275 * arm.ArmPos()*-1+1) + 0.7);
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
                sleep(3020);

                //Arm in
                while (arm.ArmPos() > 600 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                sleep(100);

                servoMoveGripper.setPosition(0);
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
                servoMoveGripper.setPosition(0);
*/
            } else if (finalPos == 1) {

                methods.driveDean(12,86);
                methods.rotateToHeading(-90);
                methods.Stop();
                servoIntakeR.setPosition(0);
                sleep(300);
                methods.driveDean(25,64);
                methods.Stop();
                servoIntakeR.setPosition(1);
                sleep(300);
                while (arm.ArmPos() < 3250 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.2);
                sleep(800);
                servoChopstickR.setPosition(0.19);
                sleep(800);
                methods.Stop();
                while (arm.ArmPos() > 400 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickR.setPosition(0.19);
                sleep(300);
                methods.driveDean(58,15);
                methods.Stop();
                terminateOpModeNow();


/*
                methods.driveY(110);
                methods.Stop();
                servoIntakeR.setPosition(0);
                sleep(300);
                methods.driveY(15);
                methods.rotateToHeading(-90);
                methods.Stop();
                sleep(300);
                methods.driveY(30);
                methods.Stop();
                servoIntakeR.setPosition(1);
                sleep(300);
                methods.driveY(40);
                methods.Stop();
                sleep(300);
                methods.driveX(65);
                methods.Stop();
                sleep(300);
                while (arm.ArmPos() < 3050 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(300);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.25);
                sleep(800);
                servoChopstickR.setPosition(0.19);
                sleep(800);
                while (arm.ArmPos() > 600 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickR.setPosition(0.19);
                sleep(300);
                methods.driveX(45);
                methods.driveY(15);
                methods.Stop();
                terminateOpModeNow();
*/

/*
                servoChopstickL.setPosition(0.47);
                servoChopstickR.setPosition(0.3);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(0);
                sleep(50);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(3000);

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
                sleep(300);

                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(1250);

                servoIntakeR.setPosition(1);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(2000);

                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(2000);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1500);

                //naar voren nadat tegen bord aan
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(620);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(300);


                //arm omhoog
                while (arm.ArmPos() < 3000 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                sleep(200);

                servoMoveGripper.setPosition((0.000275 * arm.ArmPos()*-1+1) + 0.7);
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
                sleep(2000);

                //Arm in
                while (arm.ArmPos() > 500 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                sleep(100);

                servoMoveGripper.setPosition(0);
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
                servoMoveGripper.setPosition(0);
*/

            } else if (finalPos == 2){
                methods.driveDean(35 ,72);
                methods.rotateToHeading(-90);
                methods.Stop();
                servoIntakeR.setPosition(0);
                sleep(300);
                methods.driveDean(25,40);
                methods.Stop();
                servoIntakeR.setPosition(1);
                sleep(300);
                while (arm.ArmPos() < 3250 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.2);
                sleep(800);
                servoChopstickR.setPosition(0.19);
                sleep(800);
                methods.Stop();
                while (arm.ArmPos() > 400 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickR.setPosition(0.19);
                sleep(300);
                methods.driveDean(45,15);
                methods.Stop();
                terminateOpModeNow();


/*
                methods.driveY(68);
                methods.rotateToHeading(-90);
                methods.driveY(30);
                methods.Stop();
                servoIntakeR.setPosition(0);
                sleep(300);
                methods.driveY(38);
                methods.Stop();
                servoIntakeR.setPosition(1);
                sleep(300);
                methods.driveX(30);
                methods.Stop();
                sleep(300);
                while (arm.ArmPos() < 3000 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                methods.Stop();
                sleep(300);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.25);
                sleep(800);
                servoChopstickR.setPosition(0.19);
                sleep(800);
                while (arm.ArmPos() > 600 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                methods.Stop();
                servoMoveGripper.setPosition(0.245);
                servoChopstickR.setPosition(0.19);
                sleep(300);
                methods.driveX(30);
                methods.driveY(15);
                methods.Stop();
                terminateOpModeNow();
*/
/*
                servoChopstickL.setPosition(0.47);
                servoChopstickR.setPosition(0.3);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                servoMoveGripper.setPosition(0);
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

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(200);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1000);

                servoIntakeR.setPosition(0);
                sleep(500);

                servoIntakeR.setPosition(1);
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(700);

                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(1400);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(200);

                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(500);

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
                sleep(200);* /

                //naar voren nadat tegen bord aan
                leftFront.setPower(-power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(-power);
                sleep(620);

                leftFront.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                sleep(200);

                telemetry.addData("ArmPos",arm.ArmPos());
                telemetry.update();

                //arm omhoog
                while (arm.ArmPos() < 3020 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                arm1.setPower(0);
                sleep(200);

                servoMoveGripper.setPosition((0.000275 * arm.ArmPos()*-1+1) + 0.7);
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
                sleep(600);

                //Klein stukje naar links
                leftFront.setPower(power);
                rightFront.setPower(-power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                sleep(3000);

                //Arm in
                while (arm.ArmPos() > 600 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                arm1.setPower(0);
                sleep(100);

                servoMoveGripper.setPosition(0);
                telemetry.addData("ArmPos",arm.ArmPos());
                telemetry.update();
                sleep(400);

                //Naar achteren rijden
                arm1.setPower(0);
                leftFront.setPower(power);
                rightFront.setPower(power);
                leftBack.setPower(power);
                rightBack.setPower(power);
                sleep(2600);

                //alles uit
                leftFront.setPower(-0);
                rightFront.setPower(-0);
                leftBack.setPower(-0);
                rightBack.setPower(-0);
                sleep(400);

                arm1.setPower(0);
                servoMoveGripper.setPosition(0);
*/
            }
            sleep(30000);
        }
    }
}

