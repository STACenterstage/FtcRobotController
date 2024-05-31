package org.firstinspires.ftc.teamcode.robotParts_new;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Slides_new {

    private DcMotorEx left_slide;
    private DcMotorEx right_slide;
    public void init(HardwareMap map) {
        left_slide = map.get(DcMotorEx.class, "left_slide");
        right_slide = map.get(DcMotorEx.class, "left_slide");
    }
    public void slides_go_brr(double slides_movement){
        double slides_movement_m = slides_movement;

        left_slide.setPower(slides_movement_m);
        right_slide.setPower(slides_movement_m);
    }
}
