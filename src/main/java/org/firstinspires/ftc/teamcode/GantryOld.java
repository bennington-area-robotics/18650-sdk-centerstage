package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class GantryOld {

    private DcMotor gantryMotor;

    public GantryOld(HardwareMap hardwareMap){
        gantryMotor = hardwareMap.get(DcMotor.class, "gantryMotor");
    }


    public void reset(){
        gantryMotor.setPower(0);
        gantryMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gantryMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
