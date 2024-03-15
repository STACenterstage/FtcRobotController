//package org.firstinspires.ftc.teamcode.robotParts;
//
//import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.IMU;
//
//import java.util.Objects;
//
//public class Drivetrain_weirdo_FieldCentric {
//    private final LinearOpMode myOpMode;
//    public DcMotor FrontL;
//    public DcMotor FrontR;
//    public DcMotor BackL;
//    public DcMotor BackR;
//    double current_target_heading = 0;
//    public IMU imu;
//    double WHEEL_RADIUS = 48;//mm
//    double GEAR_RATIO = 1/13.7;
//    double TICKS_PER_ROTATION = 8192;
//    double OURTICKS_PER_CM;
//    double threshold = 250;
//    final double odoMultiplier = 1.76;
//    public Drivetrain_weirdo_FieldCentric(LinearOpMode opmode) {myOpMode = opmode;}
//
//
//    public void init(HardwareMap map) {
//        imu = map.get(IMU.class, "imu");
//
//        FrontL = map.get(DcMotor.class, "left_front");
//        FrontR = map.get(DcMotor.class, "right_front");
//        BackL = map.get(DcMotor.class, "left_back");
//        BackR = map.get(DcMotor.class, "right_back");
//
//        FrontL.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontR.setDirection(DcMotorSimple.Direction.FORWARD);
//        BackL.setDirection(DcMotorSimple.Direction.REVERSE);
//        BackR.setDirection(DcMotorSimple.Direction.FORWARD);
//
//        OURTICKS_PER_CM = odoMultiplier*(TICKS_PER_ROTATION)/(2*Math.PI * GEAR_RATIO * WHEEL_RADIUS);
//
//        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
//
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
//
//        imu.initialize(new IMU.Parameters(orientationOnRobot));
//        imu.resetYaw();
//    }
//
//    public static void FieldCentric() {
//        double theta = getCurrentHeadingRadians();
//        double FWD = (myOpMode.gamepad1.left_stick_x * Math.sin(theta) + myOpMode.gamepad1.left_stick_y * Math.cos(theta));
//        double STR = (myOpMode.gamepad1.left_stick_x * Math.cos(theta) - myOpMode.gamepad1.left_stick_y * Math.sin(theta));
//        double ROT = myOpMode.gamepad1.right_stick_x;
//        double maxPower = 1.0;
//
//        if (myOpMode.gamepad1.x) {
//            double marginYaw = 6.0;
//
//            double currentHeading = getCurrentHeadingDegrees();
//
//            double targetHeading;
//            if(Objects.equals(alliance, "Red")){targetHeading = 90.0;}
//            else {targetHeading = -90.0;}
//            double dHeading = Math.abs(current_target_heading - targetHeading);
//
//            ROT = 0.3;
//            if (dHeading < marginYaw) {ROT = 0.1;}
//            if (currentHeading < targetHeading) {
//                ROT *= -1;
//            }
//        }
//
//        double FrontLPower = ((-FWD + STR + ROT) * speed);
//        double FrontRPower = ((-FWD - STR - ROT) * speed);
//        double BackLPower = ((-FWD - STR + ROT) * speed);
//        double BackRPower = ((-FWD + STR - ROT) * speed);
//
//        maxPower = Math.max(maxPower, Math.abs(FrontLPower));
//        maxPower = Math.max(maxPower, Math.abs(FrontRPower));
//        maxPower = Math.max(maxPower, Math.abs(BackLPower));
//        maxPower = Math.max(maxPower, Math.abs(BackRPower));
//
//        FrontL.setPower(FrontLPower/maxPower);
//        FrontR.setPower(FrontRPower/maxPower);
//        BackL.setPower(BackLPower/maxPower);
//        BackR.setPower(BackRPower/maxPower);
//
//        if (myOpMode.gamepad1.right_bumper && myOpMode.gamepad1.left_bumper) {
//            resetYaw();
//        }
//        telemetry.addData("Current heading",getCurrentHeadingDegrees());
//        telemetry.update();
//    }
//}