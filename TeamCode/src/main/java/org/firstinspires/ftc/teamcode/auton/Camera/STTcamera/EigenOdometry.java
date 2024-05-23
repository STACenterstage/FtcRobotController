package org.firstinspires.ftc.teamcode.auton.Camera.STTcamera;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    double WHEEL_RADIUS = 36.5;//mm
    double ODO_RADIUS = 25.0;//mm
    double GEAR_RATIO = 1/20.0;
    double TICKS_PER_ROTATION = 8192;
    double OURTICKS_PER_CM_Y;
    double OURTICKS_PER_CM_X;
    double threshold = 4000;
    final double odoMultiplierY = .58; //TODO: waardes aanpassen
    final double odoMultiplierX = .6;  //TODO: waardes aanpassen
    private DcMotorEx arm1;

    double beginTime;
    double TimeElapsed;
    double OdoY_Pos;
    double OdoX_Pos;
    double tickY;
    double tickX;
    double dPosY;
    double dPosX;
    double dPos;
    double Kp = 0.05;
    double FWD;
    double STR;
    double ROT;
    double speed;
    double min_speed = 0.3;
    double remweg;
    int remwegTicks = 24000;
    double a = 1;
    double b = 1;
    double heading;
    double dHeading;
    double direction;


    double power = 0.35;

    private DcMotor encoderX, encoderY;

    public EigenOdometry(LinearOpMode opmode) {myOpMode = opmode;}


    public void init(HardwareMap map) {
        imu = map.get(IMU.class, "imu");

        FrontL = map.get(DcMotor.class, "left_front");
        FrontR = map.get(DcMotor.class, "right_front");
        BackL = map.get(DcMotor.class, "left_back");
        BackR = map.get(DcMotor.class, "right_back");
        arm1 = map.get(DcMotorEx.class, "arm1");

        BackR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);

        OURTICKS_PER_CM_X = odoMultiplierX*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);
        OURTICKS_PER_CM_Y = odoMultiplierY*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        resetYaw();
    }


    public void updateDean(){
        dHeading = getCurrentHeading() + current_target_heading;
        TimeElapsed = System.currentTimeMillis() - beginTime;
        OdoX_Pos = FrontL.getCurrentPosition();
        OdoY_Pos = BackL.getCurrentPosition();
        dPosY = tickY - OdoY_Pos;
        dPosX = tickX - OdoX_Pos;
        dPos = Math.abs(dPosX) + Math.abs(dPosY);
    }

    public void driveDean(double x, double y) {driveDean(x, y, 0.35, myOpMode.telemetry, 30000);}
    public void driveDean(double x,double y, double max_speed, Telemetry telemetry, double stopTime) {
        calibrateEncoders();

        beginTime = System.currentTimeMillis();
        double turn;
        heading = current_target_heading;
        tickY = (int) (y * OURTICKS_PER_CM_Y);
        tickX = (int) (x * OURTICKS_PER_CM_X);
        remweg = max_speed * remwegTicks;
        updateDean();



        while ((Math.abs(dPosY) > threshold || Math.abs(dPosX) > threshold || Math.abs(dHeading) > 0.5) && myOpMode.opModeIsActive() && TimeElapsed < stopTime){

            if (Math.abs(dPosY) < remweg) {
                a = dPosY / remweg;
            } else {
                a = 1;
            }
            if (Math.abs(dPosX) < remweg) {
                b = dPosX / remweg;
            } else {
                b = 1;
            }

        FWD = a * max_speed;
        speed = min_speed + a * (max_speed - min_speed);
        STR = b * max_speed * 1.4;
        if ((dPosY < 0 && FWD > 0) || (dPosY > 0 && FWD < 0)) {
            FWD = -FWD;
        }
        if ((dPosX < 0 && STR > 0) || (dPosX > 0 && STR < 0)) {
            STR = -STR;
        }
        turn =  (-1 * Kp * Math.abs(speed) * dHeading);

        telemetry.addData("PosY", OdoY_Pos / OURTICKS_PER_CM_Y);
        telemetry.addData("PosX", OdoX_Pos / OURTICKS_PER_CM_X);
        telemetry.addData("turn", turn);
        telemetry.addData("FL power", FrontL.getPower());
        telemetry.addData("FR power", FrontR.getPower());
        telemetry.addData("BL power", BackL.getPower());
        telemetry.addData("BR power", BackR.getPower());
        telemetry.update();

        FrontL.setPower(FWD + STR + turn);
        FrontR.setPower(FWD - STR - turn);
        BackL.setPower(FWD - STR + turn);
        BackR.setPower(FWD + STR - turn);
        updateDean();
        }
        Stop();
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
         updateDean();

         double tick = (int) (position * OURTICKS_PER_CM_Y);
         double dPos = tick - OdoY_Pos;
         while (!(dPos > -threshold  && dPos < threshold) && myOpMode.opModeIsActive()) {
             if ((dPos < 0 && speed > 0) || (dPos > 0 && speed < 0)) {
                 speed = -speed;
             }
             //TODO: see if turn correction works
 //            turn = Kp*Math.abs(speed)*(heading-getCurrentHeading());

             telemetry.addData("tick", tick);
             telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM_Y);
             telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM_X);
             telemetry.addData("dPos", dPos);
             telemetry.addData("speed", speed);
             telemetry.addData("CurrentHeading", getCurrentHeading());
             telemetry.addData("TargetHeading", heading);
             telemetry.update();

             FrontL.setPower(speed + turn);
             BackL.setPower(speed + turn);
             BackR.setPower(speed - turn);
             FrontR.setPower(speed - turn);

             OdoX_Pos = FrontL.getCurrentPosition();
             OdoY_Pos = BackL.getCurrentPosition();
             dPos = tick - OdoY_Pos;

             while ((OdoX_Pos > 800 || OdoX_Pos < -800) && myOpMode.opModeIsActive()) {
                 if(OdoX_Pos > 0){
                     // rijd naar voren
                     FrontL.setPower(-0.7*power);
                     FrontR.setPower(0.7*power);
                     BackL.setPower(0.7*power);
                     BackR.setPower(-0.7*power);
                 } else if(OdoX_Pos < 0){
                     // rijd naar achteren
                     FrontL.setPower(0.7*power);
                     FrontR.setPower(-0.7*power);
                     BackL.setPower(-0.7*power);
                     BackR.setPower(0.7*power);
                 }}


 //            while ((OdoX_Pos > 500 || OdoX_Pos < -500) && myOpMode.opModeIsActive()) {
 //                if(OdoX_Pos > 0){
 //                    // rijd naar rechts
 //                    FrontL.setPower(-0.4*power);
 //                    FrontR.setPower(0.4*power);
 //                    BackL.setPower(0.4*power);
 //                    BackR.setPower(-0.4*power);
 //                } else if(OdoX_Pos < 0){
 //                    // rijd naar links
 //                    FrontL.setPower(0.4*power);
 //                    FrontR.setPower(-0.4*power);
 //                    BackL.setPower(-0.4*power);
 //                    BackR.setPower(0.4*power);
 //                }
 //                OdoX_Pos = FrontL.getCurrentPosition();
 //                OdoY_Pos = BackL.getCurrentPosition();
 //                dPos = tick - OdoY_Pos;
 //            }
             while (((getCurrentHeading()+current_target_heading)  > 5 || (getCurrentHeading()+current_target_heading) < -5) && myOpMode.opModeIsActive()) {
                 if((getCurrentHeading()+current_target_heading) > 0){
                     FrontL.setPower(-0.7*power);
                     FrontR.setPower(0.7*power);
                     BackL.setPower(-0.7*power);
                     BackR.setPower(0.7*power);
                 } else if((getCurrentHeading()+current_target_heading) < 0){
                     FrontL.setPower(0.7*power);
                     FrontR.setPower(-0.7*power);
                     BackL.setPower(0.7*power);
                     BackR.setPower(-0.7*power);

                 }
             }

             while (((getCurrentHeading()+current_target_heading) > 2 || (getCurrentHeading()+current_target_heading) < -2) && myOpMode.opModeIsActive()) {
                 if((getCurrentHeading()+current_target_heading) > 0){
                     FrontL.setPower(-0.4*power);
                     FrontR.setPower(0.4*power);
                     BackL.setPower(-0.4*power);
                     BackR.setPower(0.4*power);
                 } else if((getCurrentHeading()+current_target_heading) < 0){
                     FrontL.setPower(0.4*power);
                     FrontR.setPower(-0.4*power);
                     BackL.setPower(0.4*power);
                     BackR.setPower(-0.4*power);

                 }
             }
             updateDean();

         }
         myOpMode.sleep(100);
     }
     public void driveX(double position){
         driveX(position,0.3, myOpMode.telemetry);
     }
     //positive = right
     public void driveX(double  position, double speed, Telemetry telemetry) {
         speed = speed * -1;
         calibrateEncoders();
         double Kp = 0.03;
         double turn= 0;
         double heading = current_target_heading;
         double OdoX_Pos = FrontL.getCurrentPosition();
         double OdoY_Pos = BackL.getCurrentPosition();
         updateDean();

         double tick = (int) (position * OURTICKS_PER_CM_X);
             double dPos = tick - OdoX_Pos;
             while (!(dPos > -threshold && dPos < threshold) && myOpMode.opModeIsActive()) {
                 if ((dPos > 0 && speed > 0) || (dPos < 0 && speed < 0)) {
                     speed = -speed;
                 }

             telemetry.addData("tick", tick);
             telemetry.addData("PosY", OdoY_Pos/OURTICKS_PER_CM_Y);
             telemetry.addData("dPos", dPos);
             telemetry.addData("speed", speed);
             telemetry.addData("CurrentHeading", getCurrentHeading());
             telemetry.addData("TargetHeading", heading);
             telemetry.addData("PosX", OdoX_Pos/OURTICKS_PER_CM_X);
             telemetry.update();

             FrontL.setPower(-0.8*speed + turn);
             FrontR.setPower(0.8*speed - turn);
             BackL.setPower(gravityConstant * (0.8*speed + turn));
             BackR.setPower(gravityConstant * (-0.8*speed - turn));

             OdoY_Pos = BackL.getCurrentPosition();
             OdoX_Pos = FrontL.getCurrentPosition();
             dPos = tick - OdoX_Pos;

                 while ((OdoY_Pos > 800 || OdoY_Pos < -800) && myOpMode.opModeIsActive()) {
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
                     }}

 //            while ((OdoY_Pos > 600 || OdoY_Pos < -600) && myOpMode.opModeIsActive()) {
 //                if(OdoY_Pos > 0){
 //                    // rijd naar voren
 //                    FrontL.setPower(-0.3*power);
 //                    FrontR.setPower(-0.3*power);
 //                    BackL.setPower(-0.3*power);
 //                    BackR.setPower(-0.3*power);
 //                } else if(OdoY_Pos < 0){
 //                    // rijd naar achteren
 //                    FrontL.setPower(0.3*power);
 //                    FrontR.setPower(0.3*power);
 //                    BackL.setPower(0.3*power);
 //                    BackR.setPower(0.3*power);
 //                }
 //                OdoY_Pos = BackL.getCurrentPosition();
 //                OdoX_Pos = FrontL.getCurrentPosition();
 //                dPos = tick - OdoX_Pos;
 //            }

                 while (((getCurrentHeading()+current_target_heading) > 5 || (getCurrentHeading()+current_target_heading) < -5) && myOpMode.opModeIsActive()) {
                     if((getCurrentHeading()+current_target_heading) > 0){
                         FrontL.setPower(-0.7*power);
                         FrontR.setPower(0.7*power);
                         BackL.setPower(-0.7*power);
                         BackR.setPower(0.7*power);
                     } else if((getCurrentHeading()+current_target_heading) < 0){
                         FrontL.setPower(0.7*power);
                         FrontR.setPower(-0.7*power);
                         BackL.setPower(0.7*power);
                         BackR.setPower(-0.7*power);
                     }
                 }

                 while (((getCurrentHeading()+current_target_heading) > 1.5 || (getCurrentHeading()+current_target_heading) < -1.5) && myOpMode.opModeIsActive()) {
                     if((getCurrentHeading()+current_target_heading) > 0){
                         FrontL.setPower(-0.4*power);
                         FrontR.setPower(0.4*power);
                         BackL.setPower(-0.4*power);
                         BackR.setPower(0.4*power);
                     } else if((getCurrentHeading()+current_target_heading) < 0){
                         FrontL.setPower(0.4*power);
                         FrontR.setPower(-0.4*power);
                         BackL.setPower(0.4*power);
                         BackR.setPower(-0.4*power);
                     }
                 }
                 updateDean();

     }

         myOpMode.sleep(100);
     }

    public void rotateToHeading(double target_heading){rotateToHeading(target_heading,0.6, myOpMode.telemetry);}
    public void rotateToHeading(double target_heading, double speed, Telemetry telemetry) {
        calibrateEncoders();
        target_heading *= -1;
        double current_heading = -getCurrentHeading();
        double dHeading = target_heading - current_heading;
        double direction;
        double margin = 0.5;
        updateDean();

        telemetry.addData("curHeading", current_heading);
        telemetry.addData("dHeading",dHeading);
        telemetry.update();
        while (!(Math.abs(dHeading) < margin) && myOpMode.opModeIsActive()) {
            direction = checkDirection(dHeading);

            FrontL.setPower(-speed * direction);
            FrontR.setPower(speed * direction);
            BackL.setPower((-speed * direction));
            BackR.setPower(speed * direction);

            current_heading = -getCurrentHeading();
            dHeading = target_heading - current_heading;
            if(dHeading < 10 * margin) {
                speed = 0.2;
            }
            telemetry.addData("current_target_heading", current_target_heading);
            telemetry.addData("curHeading", current_heading);
            telemetry.addData("dHeading",dHeading);
            telemetry.update();
            updateDean();

        }
        calibrateEncoders();
        Stop();
        current_target_heading = target_heading;
    }
    public void Stop(){
        FrontL.setPower(0);
        FrontR.setPower(0);
        BackL.setPower(0);
        BackR.setPower(0);
        arm1.setPower(0);

        FrontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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