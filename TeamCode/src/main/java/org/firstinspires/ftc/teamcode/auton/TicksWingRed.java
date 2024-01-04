package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="TicksWingRed", group="LinearOpmode")
public class TicksWingRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private DcMotorEx arm1;
    private Servo servoGripper;
    private Servo servoMoveGripper;
    private Servo servoIntake;
    double power = 1;

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

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }


    public void drive(int leftBackTicks, int rightBackTicks, int leftFrontTicks, int rightFrontTicks) {
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setTargetPosition(leftBackTicks);
        rightBack.setTargetPosition(rightBackTicks);
        leftFront.setTargetPosition(leftFrontTicks);
        rightFront.setTargetPosition(rightFrontTicks);

        //DriveFront
        if (leftBackTicks < 0 && leftFrontTicks < 0 && rightBackTicks < 0 && rightFrontTicks < 0) {
            leftBack.setPower(.28);  //NIET VERANDEREN
            rightBack.setPower(.67);  //NIET VERANDEREN
            leftFront.setPower(.62);  //NIET VERANDEREN
            rightFront.setPower(.67);  //NIET VERANDEREN
        }
        //DriveBack
        if (leftBackTicks > 0 && leftFrontTicks > 0 && rightBackTicks > 0 && rightFrontTicks > 0) {
            leftBack.setPower(.28);  //NIET VERANDEREN
            rightBack.setPower(.90);  //NIET VERANDEREN
            leftFront.setPower(.70);  //NIET VERANDEREN
            rightFront.setPower(.85);  //NIET VERANDEREN
        }
        //DriveLeft
        if (leftBackTicks < 0 && leftFrontTicks > 0 && rightBackTicks > 0 && rightFrontTicks < 0) {
            leftBack.setPower(.28);  //NIET VERANDEREN
            rightBack.setPower(.67);  //NIET VERANDEREN
            leftFront.setPower(.69);  //NIET VERANDEREN
            rightFront.setPower(.76);  //NIET VERANDEREN
        }
        //DriveRight
        if (leftBackTicks > 0 && leftFrontTicks < 0 && rightBackTicks < 0 && rightFrontTicks > 0) {
            leftBack.setPower(.58);  //NIET VERANDEREN
            rightBack.setPower(.95);  //NIET VERANDEREN
            leftFront.setPower(.90);  //NIET VERANDEREN (.80)
            rightFront.setPower(.95);  //NIET VERANDEREN (.85)
        }



        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(leftBack.isBusy() && rightBack.isBusy() && leftFront.isBusy() && rightFront.isBusy()) {
            idle();
        }
    }
    public void driveLeft(int Ticks) {drive(-Ticks, Ticks, Ticks, -Ticks);}
    public void driveRight(int Ticks) {drive(Ticks, -Ticks, -Ticks, Ticks);}
    public void driveForward(int Ticks) {drive(-Ticks, -Ticks, -Ticks, -Ticks);}
    public void driveBackward(int Ticks) {drive(Ticks, Ticks, Ticks, Ticks);}
    public void turnLeft (int degrees) {int Ticks = degrees * 70 / 6; drive(Ticks, -Ticks, Ticks, -Ticks); }
    public void turnRight (int degrees) {int Ticks = degrees * 70 / 6; drive (-Ticks, Ticks, -Ticks, Ticks); }

    @Override
    public void runOpMode() {

        init(hardwareMap);
        runtime.reset();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        //pak pixels en wacht
        servoGripper.setPosition(0);
        servoIntake.setPosition(1);
        sleep(100);

        //1400 Ticks = 1 Tile(60cm)
        driveRight(3000);
        turnRight(30); //Weirdo stoppositie corrigeren
        driveBackward(6700);
        //driveLeft(500);

        //uit time based gehaald
            runtime.reset();
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            init(hardwareMap);

            waitForStart();
            runtime.reset();

            arm1.setPower(power);
            sleep(400);



        }
}
