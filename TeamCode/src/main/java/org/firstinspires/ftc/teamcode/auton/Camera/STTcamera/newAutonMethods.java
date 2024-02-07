package org.firstinspires.ftc.teamcode.auton.Camera.STTcamera;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class newAutonMethods {
    private LinearOpMode myOpMode;
//    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor FrontL;
    public DcMotor FrontR;
    public DcMotor BackL;
    public DcMotor BackR;

    final public int robotLength_cm = 39;
    final public int robotWidth_cm = 40;
    final public double gravityConstant = 1;

    double current_target_heading = 0;
    public IMU imu;
    double WHEEL_RADIUS = 37.5;//mm
    double ODO_RADIUS = 25.0;//mm
    double GEAR_RATIO = 1/20.0;
    double TICKS_PER_ROTATION = 8192;
    double OURTICKS_PER_CM;
    double threshold = 250;
    final double odoMultiplier = .706;

    private DcMotor encoderX, encoderY;

    public newAutonMethods(LinearOpMode opmode) {myOpMode = opmode;}

    public void init(HardwareMap map) {
        imu = map.get(IMU.class, "imu");

        FrontL = map.get(DcMotor.class, "left_front");
        FrontR = map.get(DcMotor.class, "right_front");
        BackL = map.get(DcMotor.class, "left_back");
        BackR = map.get(DcMotor.class, "right_back");

        BackR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);

        OURTICKS_PER_CM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
    }

    public void driveY (double position){
        driveY(position,0.3, myOpMode.telemetry);
    }
//    positive = forward
    public void driveY(double position, double speed, Telemetry telemetry) {
        calibrateEncoders();
        double Kp = 0.03;
        double turn = 0;
        double heading = current_target_heading;
        double OdoY_Pos = BackL.getCurrentPosition();
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoY_Pos;
        while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
                speed = -speed;
            }
            //TODO: see if turn correction works
//            turn = Kp*Math.abs(speed)*(heading-getCurrentHeading());

            telemetry.addData("tick", tick);
            telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.addData("CurrentHeading", getCurrentHeading());
            telemetry.addData("TargetHeading", heading);
            telemetry.update();

//            FrontL.setPower(speed + turn);
//            FrontR.setPower(-speed - turn);
//            BackL.setPower(-speed + turn);
//            BackR.setPower(speed - turn);

            FrontL.setPower(speed + turn);
            BackL.setPower(speed + turn);

            BackR.setPower(speed - turn);
            FrontR.setPower(speed - turn);

            OdoY_Pos = BackL.getCurrentPosition();
            dPos = tick - OdoY_Pos;
        }
        Stop();
        myOpMode.sleep(100);
    }
    public void driveX(double position){
        driveX(position,0.3, myOpMode.telemetry);
    }
    //positive = right
    public void driveX(double position, double speed, Telemetry telemetry) {
        speed = speed * -1;
        calibrateEncoders();
        double Kp = 0.03;
        double turn= 0;
        double heading = current_target_heading;
        double OdoX_Pos = FrontL.getCurrentPosition();
        double tick = (int) (position * OURTICKS_PER_CM);
        double dPos = tick - OdoX_Pos;
        while (!(dPos > -threshold && dPos < threshold) && myOpMode.opModeIsActive()) {
            if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                speed = -speed;
            }
            //TODO: see if turn correction works
//            turn = Kp*Math.abs(speed)*(heading-getCurrentHeading());

            telemetry.addData("tick", tick);
            telemetry.addData("PosX", OdoX_Pos / OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.addData("rawodomdata", position);
            telemetry.update();

            FrontL.setPower(-speed + turn);
            FrontR.setPower(speed - turn);
            BackL.setPower(gravityConstant * (speed + turn));
            BackR.setPower(gravityConstant * (-speed - turn));

            OdoX_Pos = FrontL.getCurrentPosition();
            dPos = tick - OdoX_Pos;

            myOpMode.sleep(10000);

        }
        Stop();
        myOpMode.sleep(100);
    }

    //positive = clockwise
    public void rotateToHeading(double target_heading, double speed, Telemetry telemetry) {
        double current_heading = -getCurrentHeading();
        double dHeading = target_heading - current_heading;
        double direction;
        telemetry.addData("curHeading", current_heading);
        telemetry.addData("dHeading",dHeading);
        telemetry.update();
        while (!(Math.abs(dHeading) < 1) && myOpMode.opModeIsActive()) {
            direction = -checkDirection(current_heading-target_heading*2);

            FrontL.setPower(-speed * direction);
            FrontR.setPower(speed * direction);
            BackL.setPower(gravityConstant * (-speed * direction));
            BackR.setPower(gravityConstant * speed * direction);

            current_heading = getCurrentHeading();
            dHeading = target_heading - current_heading;
            telemetry.addData("curHeading", current_heading);
            telemetry.addData("dHeading",dHeading);
            telemetry.update();
        }
        calibrateEncoders();
        Stop();
        current_target_heading = target_heading;
    }

    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
    }

    public void FieldCentric(double speed, HardwareMap map) {
        double theta = getCurrentHeading()*(Math.PI/180);
        double forward = (myOpMode.gamepad1.left_stick_x * Math.sin(theta) + myOpMode.gamepad1.left_stick_y * Math.cos(theta));
        double strafe = (myOpMode.gamepad1.left_stick_x * Math.cos(theta) - myOpMode.gamepad1.left_stick_y * Math.sin(theta));
        double rotate = myOpMode.gamepad1.right_stick_x;

        FrontL.setPower((-forward + strafe + rotate) * speed);
        FrontR.setPower((-forward - strafe - rotate) * speed);
        BackL.setPower((-forward - strafe + rotate) * speed);
        BackR.setPower((-forward + strafe - rotate) * speed);

        if (myOpMode.gamepad1.right_trigger > 0 && myOpMode.gamepad1.left_trigger > 0) {
            resetYaw();
        }
        myOpMode.telemetry.addData("wtf",-forward-strafe+rotate);
        myOpMode.telemetry.addData("Forward",forward);
        myOpMode.telemetry.addData("Strafe",strafe);
        myOpMode.telemetry.addData("Rotate",rotate);
        myOpMode.telemetry.addData("Currentheading",getCurrentHeading());
    }
    public void RobotCentric(double speed) {
        double FWD = myOpMode.gamepad1.left_stick_y;
        double STR = myOpMode.gamepad1.left_stick_x;
        double ROT = myOpMode.gamepad1.right_stick_x;
        speed = speed * -1;

        FrontL.setPower((FWD + STR + ROT) * (speed));
        FrontR.setPower((FWD - STR + ROT) * (speed));
        BackL.setPower((FWD - STR - ROT) * (speed));
        BackR.setPower((FWD + STR - ROT) * (speed));
    }

    public void Stop(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double getCurrentHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return (orientation.getYaw(AngleUnit.DEGREES));
    }

    public void resetYaw() {
        imu.resetYaw();
    }

    public void calibrateEncoders() {
        FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}