package org.firstinspires.ftc.teamcode.auton.Camera.STTcamera;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class EigenOdometry {
    private LinearOpMode myOpMode;

    public DcMotor FrontL;
    public DcMotor FrontR;
    public DcMotor BackL;
    public DcMotor BackR;

    final public double gravityConstant = 1;

    double current_target_heading = 0;
    public IMU imu;
    double WHEEL_RADIUS = 37.5;//mm
    double ODO_RADIUS = 25.0;//mm
    double GEAR_RATIO = 1/20.0;
    double TICKS_PER_ROTATION = 8192;
    double OURTICKS_PER_CM;
    double threshold = 250;
    final double odoMultiplier = .708;

    double power = 0.4;

    private DcMotor encoderX, encoderY;

    public EigenOdometry(LinearOpMode opmode) {myOpMode = opmode;}

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
        double OdoX_Pos = FrontL.getCurrentPosition();

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
            telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM);
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

            OdoX_Pos = FrontL.getCurrentPosition();
            OdoY_Pos = BackL.getCurrentPosition();
            dPos = tick - OdoY_Pos;

            while ((OdoX_Pos > 400 || OdoX_Pos < -400) && myOpMode.opModeIsActive()) {
                if(OdoX_Pos > 0){
                    // rijd naar rechts
                    FrontL.setPower(-power);
                    FrontR.setPower(power);
                    BackL.setPower(power);
                    BackR.setPower(-power);
                } else if(OdoX_Pos < 0){
                    // rijd naar links
                    FrontL.setPower(power);
                    FrontR.setPower(-power);
                    BackL.setPower(-power);
                    BackR.setPower(power);
                }
                OdoX_Pos = FrontL.getCurrentPosition();
                OdoY_Pos = BackL.getCurrentPosition();
                dPos = tick - OdoY_Pos;
            }
            while ((getCurrentHeading() > 5 || getCurrentHeading() < -5) && myOpMode.opModeIsActive()) {
                if(getCurrentHeading() > 0){
                    FrontL.setPower(-0.7*power);
                    FrontR.setPower(0.7*power);
                    BackL.setPower(-0.7*power);
                    BackR.setPower(0.7*power);
                } else if(getCurrentHeading() < 0){
                    FrontL.setPower(0.7*power);
                    FrontR.setPower(-0.7*power);
                    BackL.setPower(0.7*power);
                    BackR.setPower(-0.7*power);

                }
            }

            while ((getCurrentHeading() > 2 || getCurrentHeading() < -2) && myOpMode.opModeIsActive()) {
                if(getCurrentHeading() > 0){
                    FrontL.setPower(-0.4*power);
                    FrontR.setPower(0.4*power);
                    BackL.setPower(-0.4*power);
                    BackR.setPower(0.4*power);
                } else if(getCurrentHeading() < 0){
                    FrontL.setPower(0.4*power);
                    FrontR.setPower(-0.4*power);
                    BackL.setPower(0.4*power);
                    BackR.setPower(-0.4*power);

                }
            }

        }
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
        double OdoY_Pos = BackL.getCurrentPosition();
            double tick = (int) (position * OURTICKS_PER_CM);
            double dPos = tick - OdoX_Pos;
            while (!(dPos > -threshold && dPos < threshold) && myOpMode.opModeIsActive()) {
                if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                    speed = -speed;
                }


            telemetry.addData("tick", tick);
            telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM);
            telemetry.addData("dPos", dPos);
            telemetry.addData("speed", speed);
            telemetry.addData("CurrentHeading", getCurrentHeading());
            telemetry.addData("TargetHeading", heading);
            telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM);
            telemetry.update();

            FrontL.setPower(-speed + turn);
            FrontR.setPower(speed - turn);
            BackL.setPower(gravityConstant * (speed + turn));
            BackR.setPower(gravityConstant * (-speed - turn));

            OdoY_Pos = BackL.getCurrentPosition();
            OdoX_Pos = FrontL.getCurrentPosition();
            dPos = tick - OdoX_Pos;



            while ((OdoY_Pos > 500 || OdoY_Pos < -500) && myOpMode.opModeIsActive()) {
                if(OdoY_Pos > 0){
                    // rijd naar voren
                    FrontL.setPower(-0.7*power);
                    FrontR.setPower(-0.7*power);
                    BackL.setPower(-0.7*power);
                    BackR.setPower(-0.7*power);
                } else if(OdoY_Pos < 0){
                    // rijd naar achteren
                    FrontL.setPower(0.7*power);
                    FrontR.setPower(0.7*power);
                    BackL.setPower(0.7*power);
                    BackR.setPower(0.7*power);
                }
                OdoY_Pos = BackL.getCurrentPosition();
                OdoX_Pos = FrontL.getCurrentPosition();
                dPos = tick - OdoX_Pos;
            }
                while ((getCurrentHeading() > 5 || getCurrentHeading() < -5) && myOpMode.opModeIsActive()) {
                    if(getCurrentHeading() > 0){
                        FrontL.setPower(-0.7*power);
                        FrontR.setPower(0.7*power);
                        BackL.setPower(-0.7*power);
                        BackR.setPower(0.7*power);
                    } else if(getCurrentHeading() < 0){
                        FrontL.setPower(0.7*power);
                        FrontR.setPower(-0.7*power);
                        BackL.setPower(0.7*power);
                        BackR.setPower(-0.7*power);

                    }
                }

                while ((getCurrentHeading() > 1.5 || getCurrentHeading() < -1.5) && myOpMode.opModeIsActive()) {
                    if(getCurrentHeading() > 0){
                        FrontL.setPower(-0.4*power);
                        FrontR.setPower(0.4*power);
                        BackL.setPower(-0.4*power);
                        BackR.setPower(0.4*power);
                    } else if(getCurrentHeading() < 0){
                        FrontL.setPower(0.4*power);
                        FrontR.setPower(-0.4*power);
                        BackL.setPower(0.4*power);
                        BackR.setPower(-0.4*power);

                    }
                }

    }

        myOpMode.sleep(100);
    }

    int checkDirection(double val){
        if (val < 0)
            return -1;
        else return 1;
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