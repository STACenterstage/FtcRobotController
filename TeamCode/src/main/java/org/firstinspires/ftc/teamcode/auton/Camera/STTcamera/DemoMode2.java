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
@Autonomous(name = "DemoMode2")
public class DemoMode2 extends LinearOpMode {

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
    double time;
    double chopLOn = 0.45;
    double chopROn = 0.32;
    double chopLOff = 0.61;
    double chopROff = 0.19;
    double iteration = 0;

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

        servoChopstickL.setPosition(chopLOff); // 0.45 = on, 0.61 = off
        servoChopstickR.setPosition(chopROff); // 0.32 = on, 0.19 = off
        servoIntakeR.setPosition(1);
        servoIntakeL.setPosition(0);
        servoMoveGripper.setPosition(0.243);

        telemetry.addLine("Paarse Pixel moet LINKS!");
        telemetry.update();
    }

    public void runOpMode() {
        methods.init(hardwareMap);

        methods.calibrateEncoders();
        camera.findScoringPosition();

        init(hardwareMap);
        arm.init(hardwareMap);
        drivetrain.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            time = System.currentTimeMillis();
            methods.driveDean(0 ,-12);
            methods.Stop();
            sleep(500);

            while (!isStopRequested()) {
                iteration += 1;
                while (arm.ArmPos() > -550 && !isStopRequested()) {
                    arm1.setPower(-.65);
                }
                methods.Stop();
                sleep(300);
                servoChopstickL.setPosition(chopLOn);
                servoChopstickR.setPosition(chopROn);
                sleep(300);
                servoIntakeL.setPosition(1);
                servoIntakeR.setPosition(0);
                sleep(800);
                while (arm.ArmPos() < 2050 && !isStopRequested()) {
                    arm1.setPower(.7);
                }
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos() * -1 + 1.2);
                sleep(800);
                servoChopstickL.setPosition(chopLOff);
                servoChopstickR.setPosition(chopROff);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                sleep(800);
                arm.moveGripper(0.00015 * arm.ArmPos() * -1 + 1.3);
                methods.Stop();
                while (arm.ArmPos() > 200 && !isStopRequested()) {
                    arm1.setPower(-.7);
                }
                methods.Stop();
                servoMoveGripper.setPosition(0.243);
                methods.driveDean(0,-60);
                methods.rotateToHeading(-40);
                methods.driveDean(0, -250);
                servoIntakeL.setPosition(1);
                servoIntakeR.setPosition(0);
                sleep(500);
                methods.driveDean(0,-30);
                sleep(1500);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                sleep(1500);
                methods.driveDean(0, 280);
                methods.rotateToHeading(0);
                methods.driveDean(0,60);
                if (iteration >= 7){
                    iteration = 0;
                }
            }
        }
    }
}

