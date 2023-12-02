package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomousBlueBackstage", group="LinearOpmode")
public class AutonomousBlueBackstage extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm1;
    private Servo servoGripper;
    private Servo servoMoveGripper;
    private Servo servoIntake;
    double power = .25;

    private double servoGripperMin = 0.0;  // Minimum position for servo1 (in degrees)
    private double servoGripperMax = 180.0;  // Maximum position for servo1 (in degrees)
    private double servoMoveGripperMin = 0.0;  // Minimum position for servo2 (in degrees)
    private double servoMoveGripperMax = 180.0;  // Maximum position for servo2 (in degrees)

    public void init(HardwareMap map) {
        leftFront = map.get(DcMotorEx.class, "left_front");
        rightFront = map.get(DcMotorEx.class, "right_front");
        leftBack = map.get(DcMotorEx.class, "left_back");
        rightBack = map.get(DcMotorEx.class, "right_back");

        arm1 = hardwareMap.get(DcMotorEx.class, "arm1");

        servoGripper = hardwareMap.get(Servo.class, "servoGripper");
        servoMoveGripper = hardwareMap.get(Servo.class, "servoMoveGripper");
        servoIntake = hardwareMap.get(Servo.class, "Intake");

        servoGripper.setPosition(0);
        servoIntake.setPosition(1);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    @Override
    public void runOpMode() {
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        init(hardwareMap);

        //arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        runtime.reset();


        //pak pixel en wacht
        servoGripper.setPosition(.2);
        servoIntake.setPosition(1);
        sleep(400);

        //Vanuit startpositie naar links
        leftFront.setPower(power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(power);
        sleep(3600);

        //Intake open voor loslaten paarse pixel
        servoIntake.setPosition(0);

        //stoppen
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        sleep(400);

        //stukje naar voren voor pixel
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        sleep(720);

        leftFront.setPower(0.8*power);
        rightFront.setPower(0.8*power);
        leftBack.setPower(0.8*power);
        rightBack.setPower(0.8*power);
        sleep(2000);

        //Naar achteren rijden
        leftFront.setPower(0.8*power);
        rightFront.setPower(0.8*power);
        leftBack.setPower(0.8*power);
        rightBack.setPower(0.8*power);
        sleep(2600);

        //arm omhoog
        arm1.setPower(1);
        sleep(550);

        //Klein stukje naar links
        arm1.setPower(0);
        leftFront.setPower(-0.8*power);
        rightFront.setPower(0.8*power);
        leftBack.setPower(0.8*power);
        rightBack.setPower(-0.8*power);
        sleep(1000);

        //Alles uit
        arm1.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(100);

        //gele pixel op bord plaatsen
        servoMoveGripper.setPosition(0);
        sleep(800);

        //laat gele pixel los
        servoGripper.setPosition(.45);
        sleep(800);

        //alles uit
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        sleep(3000);

        //kléin stukje naar voren
        leftFront.setPower(-power);
        rightFront.setPower(-power);
        leftBack.setPower(-power);
        rightBack.setPower(-power);
        sleep(1000);

        //Klein stukje naar links
        leftFront.setPower(-0.8*power);
        rightFront.setPower(0.8*power);
        leftBack.setPower(0.8*power);
        rightBack.setPower(-0.8*power);
        sleep(4000);

        //Arm in
        arm1.setPower(-1);
        sleep(550);

        //Naar achteren rijden
        arm1.setPower(0);
        leftFront.setPower(0.8*power);
        rightFront.setPower(0.8*power);
        leftBack.setPower(0.8*power);
        rightBack.setPower(0.8*power);
        sleep(3000);

        //alles uit
        leftFront.setPower(-0);
        rightFront.setPower(-0);
        leftBack.setPower(-0);
        rightBack.setPower(-0);
        sleep(400);

        arm1.setPower(0);
        servoMoveGripper.setPosition(1);




    }
}


          /*

            leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*    leftFront.setPower(20000);
            rightFront.setPower(20000);
            leftBack.setPower(20000);
            rightBack.setPower(20000);
            sleep(500);


            leftFront.setPower(300);
            rightFront.setPower(0);
            leftBack.setPower(3);
            rightBack.setPower(0);

            leftBack.setPower(0);
            leftFront.setPower(0);

            leftFront.setPower(7);
            rightFront.setPower(7);
            leftBack.setPower(7);
            rightBack.setPower(7);

            arm1.setPower(power);

            arm1.setPower(0);
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);*/

