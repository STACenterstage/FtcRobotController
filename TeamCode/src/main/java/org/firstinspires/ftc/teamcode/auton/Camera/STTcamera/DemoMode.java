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
@Autonomous(name = "DemoMode")
public class DemoMode extends LinearOpMode {

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
        servoMoveGripper.setPosition(0.2);

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
                methods.driveDean(285 ,0);
                methods.Stop();
                sleep(500);
                while (arm.ArmPos() > -200 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                methods.Stop();
                sleep(300);
                servoChopstickL.setPosition(chopLOn);
                servoChopstickR.setPosition(chopROn);
                sleep(300);
                servoIntakeL.setPosition(1);
                servoIntakeR.setPosition(0);
                sleep(500);
                while (arm.ArmPos() < 3200 && !isStopRequested()){
                    arm1.setPower(.7);
                }
                methods.Stop();
                sleep(500);
                arm.moveGripper(0.00015 * arm.ArmPos()*-1+1.2);
                sleep(800);
                servoChopstickL.setPosition(chopLOff);
                servoChopstickL.setPosition(chopLOff);
                servoIntakeL.setPosition(0);
                servoIntakeR.setPosition(1);
                sleep(800);
                methods.Stop();
                while (arm.ArmPos() > 300 && !isStopRequested()){
                    arm1.setPower(-.7);
                }
                methods.Stop();
                methods.driveDean(-250 ,0);
                servoIntakeL.setPosition(1);
                servoIntakeR.setPosition(0);





            servoMoveGripper.setPosition(0.245);
                servoChopstickL.setPosition(chopLOff);
                sleep(300);
                methods.driveDean(-45,15);
                methods.Stop();
                sleep(1000);

        }
    }
}

