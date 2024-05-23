package org.firstinspires.ftc.teamcode.drive_new;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.robotParts_new.Drivetrain_new;                //Importeert data uit andere mappen
import org.firstinspires.ftc.teamcode.robotParts_new.Onderdelen_new;                //Importeert data uit andere mappen

@TeleOp(name = "STAdrive_new",group = "TeleOp")                                     //Naam van project
public class STAdrive_new extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();                                //Slaat op hoe lang de robot is geinitialiseerd

    Drivetrain_new drivetrain = new Drivetrain_new();                               //Roept de drivetrain aan uit de geïmporteerde map
    Onderdelen_new onderdelen = new Onderdelen_new();                               //Roept de onderdelen aan uit de geïmporteerde map

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap);
        onderdelen.init(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {                                  //Loop van het rijden van de robot
            double y = gamepad1.left_stick_x;                       //Koppelt geactiveerde knop op controller aan variabele
            double x = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_x;

            drivetrain.drive(x, y, rotate);                         //Voert bij drivetrain aangemaakte opdracht uit

            //boolean servo1_on = gamepad1.a;                         //Koppelt servobeweging aan variabele
            //boolean servo1_off = gamepad1.b;                        //Koppelt servobeweging aan variabele

            /*if (servo1_on) {
                onderdelen.servo1(1);
            } else if (servo1_off) {
                onderdelen.servo1(0);
            }*/

            //telemetry.addLine();                                    //Zet zin op het scherm
            //telemetry.addData("Verstreken tijd", getRuntime());     //Zet data op het scherm
            //telemetry.update();                                     //Zorgt dat data geüpdated blijft
        }
    }
}