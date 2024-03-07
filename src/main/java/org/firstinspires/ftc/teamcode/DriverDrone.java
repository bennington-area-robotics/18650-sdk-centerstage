package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="Dronetester", group="Main Modes")
public class DriverDrone extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor LFront;
    private DcMotor RFront;
    private DcMotor LRear;
    private DcMotor RRear;

    private double turnDegreeConversionNum = 12.51577;
    private double sidewaysConversionNum = 50.6936508;

    private double forwardConversionNum = 47.232;

    private DcMotor gantry;
    private DcMotor carwashCollector;
    private Servo clawCollector;
    private Servo rampLifter;
    private Servo clawCollectorRotation;
    private Servo droneServo;

    private ElapsedTime
            carwashTimeout = new ElapsedTime(),
            clawTimeout = new ElapsedTime(),
            clawRotationTimeout = new ElapsedTime(),
            rampTimeout = new ElapsedTime(),
            runtime = new ElapsedTime(), //timer used to limit runtime of functions and misc
            rampTimer = new ElapsedTime(), //timer used to control ramp up speed in setMotorTargets
            droneLauncherArmingTimer = new ElapsedTime(),
            powerModTimer = new ElapsedTime()
                    ;
    private boolean carwashToggle, clawToggle, clawRotationToggle, rampToggle, powerModToggle;

    double DPAD_POWER_LVL = 0.7;
    double Current_Power_Lvl = 1;
    private void ResetMotors() {
        DcMotor[] motors = {
                LFront,
                LRear,
                RFront,
                RRear
        };

        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void Initialize_Motors_Servos() {
        LFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LRear.setDirection(DcMotorSimple.Direction.FORWARD);
        RFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RRear.setDirection(DcMotorSimple.Direction.REVERSE);

        LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gantry.setDirection(DcMotorSimple.Direction.FORWARD);
        carwashCollector.setDirection(DcMotorSimple.Direction.FORWARD);
        gantry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carwashCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gantry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carwashCollector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rampLifter.setPosition(0.4);
        clawCollector.setPosition(0);
        //clawCollectorRotation.setPosition(1);
        droneServo.setPosition(0);

        carwashToggle = false;
        clawToggle = true;
        clawRotationToggle = false;
        rampToggle = true;
        powerModToggle = true;
    }

    @Override
    public void runOpMode() {
        telemetry.addData("TESTING", "Initialized");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LFront = hardwareMap.get(DcMotor.class, "LFront");
        RFront = hardwareMap.get(DcMotor.class, "RFront");
        LRear = hardwareMap.get(DcMotor.class, "LRear");
        RRear = hardwareMap.get(DcMotor.class, "RRear");

        gantry = hardwareMap.get(DcMotor.class, "gantry");
        carwashCollector = hardwareMap.get(DcMotor.class, "carwashCollector");

        rampLifter = hardwareMap.get(Servo.class, "rampLifter");
        clawCollector = hardwareMap.get(Servo.class, "clawCollectorJaws");
        droneServo = hardwareMap.get(Servo.class, "droneServo");
        clawCollectorRotation = hardwareMap.get(Servo.class, "clawCollectorRotation");


        Initialize_Motors_Servos();

        waitForStart();
        runtime.reset();
        if (opModeIsActive()) {
            //setMotorTargets(6, 600, 10, 0.5, true);
            droneLauncherArmingTimer.reset();
            while (opModeIsActive()) {

                checkForUserInput();
                telemetry.addData("Gantry pos", gantry.getCurrentPosition());
                telemetry.addData("gantry target", gantry.getTargetPosition());
                telemetry.addData("drone arming timer", droneLauncherArmingTimer.seconds());
                telemetry.update();
            }
        }
    }
    private void GoStraight() {
        Set_Power_Values(
                0,
                -1 * DPAD_POWER_LVL,
                gamepad1.right_stick_x,
                Current_Power_Lvl
        );
    }
    private void GoBackwards() {
        Set_Power_Values(
                0,
                DPAD_POWER_LVL,
                gamepad1.right_stick_x,
                Current_Power_Lvl
        );
    }
    private void GoLeft() {
        Set_Power_Values(
                -1 * DPAD_POWER_LVL,
                0,
                gamepad1.right_stick_x,
                Current_Power_Lvl
        );
    }
    private void GoRight() {
        Set_Power_Values(
                DPAD_POWER_LVL,
                0,
                gamepad1.right_stick_x,
                Current_Power_Lvl
        );
    }

    /**
     * only Carl knows what this does. he's been gone since 2020.
     * since we only use the right joystick, this only ever returns 1.
     */
    private double Get_Denominator() {
        double sum = Math.abs(gamepad1.left_stick_y)
                + Math.abs(gamepad1.left_stick_x)
                + Math.abs(gamepad1.right_stick_x);
        if (sum > 1) {
            return sum;
        } else {
            return 1;
        }
    }

    /**
     * Sets the power values for the different motors.
     * To go straight/backwards, all LeftY values need to be equivalent.
     * To go sideways, the LeftX values of diagonally opposite motors need to be equivalent.
     * To rotate, the RightX values of motors on the same L/R side of the robot need to be equivalent.
     */
    private void Set_Power_Values(double LeftX, double LeftY, float RightX, double Power_Mod)
    {
        double lFront = (((-LeftY + LeftX) + RightX)/* / denominator*/) * Power_Mod;
        double lRear = (((-LeftY - LeftX) + RightX) /*/ denominator*/) * Power_Mod;
        double rFront = (((-LeftY - LeftX) - RightX) /*/ denominator*/) * Power_Mod;
        double rRear = (((-LeftY + LeftX) - RightX) /*/ denominator*/) * Power_Mod;

        LFront.setPower(lFront * 1.0007155);
        LRear.setPower(lRear * 1.0017707);
        RFront.setPower(rFront);
        RRear.setPower(rRear * 1.0032978);

    }
    private void Process_Movement() {
        telemetry.addData("Process_Movement", 0);
        if (gamepad1.dpad_up) {
            GoStraight();
        } else if (gamepad1.dpad_down) {
            GoBackwards();
        } else if (gamepad1.dpad_left) {
            GoLeft();
        } else if (gamepad1.dpad_right) {
            GoRight();
        } else {
            Set_Power_Values(0, 0, gamepad1.right_stick_x/2, Current_Power_Lvl);
        }

    }
    public void checkForUserInput(){
        checkCarwashInput(); //gamepad1.x
        checkClawInput(); //gamepad1.right_bumper
        checkClawRotationInput(); //gamepad1.left_bumper
        checkGantryInput(); //gamepad triggers
        //checkRampInput(); //gamepad1.left_joystick button
        Process_Movement();
        checkPowerModInput(); //gamepad1.y

        if (gamepad1.a && !gantry.isBusy()) {
            collectPixelFromRamp();
        }
        if (gamepad1.b && !gantry.isBusy()){
            movePixelToPlacingPosition();
        }
        if (gamepad1.left_stick_button && !gantry.isBusy()){
            setGantryTarget(-30, 2, 0.6);
            gantry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            gantry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        /*if (gamepad1.right_stick_button){
            setMotorTargets(1, (800), 10, 0.5, true);
            sleep(500);
        }*/
        if(gamepad1.right_stick_button){
            droneServo.setDirection(Servo.Direction.FORWARD);
            droneServo.setPosition(1);
        }
    }

    public void checkPowerModInput(){
        if (gamepad1.y && powerModTimer.milliseconds() > 700){
            if(powerModToggle){
                DPAD_POWER_LVL /= 2;
                Current_Power_Lvl /=2;
                powerModToggle = false;
            } else {
                DPAD_POWER_LVL *= 2;
                Current_Power_Lvl *=2;
                powerModToggle = true;
            }
            powerModTimer.reset();
        }

    }

    public void checkCarwashInput(){
        if (gamepad1.left_stick_y < -0.5) {
            rampLifter.setPosition(0);
            carwashCollector.setDirection(DcMotor.Direction.REVERSE);
            carwashCollector.setPower(0.8);
        } else if (gamepad1.left_stick_y > 0.5){
            rampLifter.setPosition(0);
            carwashCollector.setDirection(DcMotorSimple.Direction.FORWARD);
            carwashCollector.setPower(1);
        } else if (gamepad1.x) {
            carwashCollector.setPower(0);
            rampLifter.setPosition(0.4);
        }
    }
    public void checkRampInput(){
        if (gamepad1.left_stick_button && rampTimeout.milliseconds() > 700) {
            if(rampToggle){
                rampLifter.setPosition(0);
                rampToggle = false;
            }else{
                rampLifter.setPosition(0.4);
                rampToggle = true;
                carwashCollector.setPower(0);
            }
            rampTimeout.reset();
        }
    }

    public void checkClawInput(){
        if (gamepad1.right_bumper && clawTimeout.milliseconds() > 700) {
            if(clawToggle){
                clawCollector.setPosition(0.5); //close claw
                clawToggle = false;
            }else{
                clawCollector.setPosition(0.2);// open claw
                clawToggle = true;

            }
            clawTimeout.reset();
        }
    }
    public void checkClawRotationInput(){
        if (gamepad1.left_bumper && clawRotationTimeout.milliseconds() > 700 && gantry.getCurrentPosition() > 500) {
            if(clawRotationToggle){
                clawCollectorRotation.setPosition(1);//move claw to grabbing position
                clawRotationToggle = false;
            }else{
                clawCollectorRotation.setPosition(0);// move claw to placing position
                clawRotationToggle = true;
            }
            clawRotationTimeout.reset();
        }
    }

    public void collectPixelFromRamp() {
        LFront.setPower(0);
        RFront.setPower(0);
        LRear.setPower(0);
        RRear.setPower(0);
        carwashCollector.setPower(0);
        //moves gantry to position to rotate collector if the collector is not already in the position to grab pixel
        if (clawCollectorRotation.getPosition() != 1){
            setGantryTarget(620, 5, 1);
            clawCollectorRotation.setPosition(1);
        }
        //opens collector if it is not already
        if (clawCollector.getPosition() != 0.2){
            clawCollector.setPosition(0.2);
        }
        //moves gantry back and grabs pixel with collector
        setGantryTarget(0, 5, 1);
        clawCollector.setPosition(0.5);
        //setGantryTarget(200, 5, 0.7);

    }
    public void movePixelToPlacingPosition(){
        LFront.setPower(0);
        RFront.setPower(0);
        LRear.setPower(0);
        RRear.setPower(0);
        carwashCollector.setPower(0);
        //moves gantry forward and rotates collector to placing position
        setGantryTarget(620, 5, 1);

        clawCollectorRotation.setPosition(0);
        //setGantryTarget(300, 5, 0.7);

    }
    public void checkGantryInput() {
        int maxHeight = 665;
        int minHeight;
        if (clawCollectorRotation.getPosition() == 1) {
            minHeight = 0;
        } else {
            minHeight = 0;
        }
        if (gamepad1.left_trigger > 0 && gantry.getCurrentPosition() > minHeight) {
            gantry.setPower(gamepad1.left_trigger * -1);
        } else if (gamepad1.right_trigger > 0 && gantry.getCurrentPosition() < maxHeight) {
            gantry.setPower((gamepad1.right_trigger));
        } else {
            gantry.setPower(0);
        }
    }
    private void setGantryTarget(int target, int timeout, double targetSpeed) {

        gantry.setTargetPosition(target);
        gantry.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gantry.setPower(targetSpeed);

        runtime.reset();

        while(opModeIsActive() && gantry.isBusy() && (runtime.seconds() < timeout) ){
            gantry.setPower(targetSpeed);
            //Process_Movement();

            //telemetry
            telemetry.addData("*** Running setMotorTargets() ***", "\n");
            telemetry.addData("Elapsed time: ", runtime.seconds());
            telemetry.update();
        }
        gantry.setPower(0);
        gantry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void setMotorTargets(int direction, int target, int timeout, double targetSpeed, boolean ramp) {
        double
                powerMin = 0.2,      // motor power at beginning and end of ramp up and down
                speed = powerMin,   // initial motor power
                increment = 0.01,   // motor power increment
                distanceToReachTargetSpeed = 0
                        ;

        int incrementInterval = 250; // interval between incrementing power (in milliseconds)
        boolean set = false; // makes distanceToReachTargetSpeed run only once

        DcMotor[] motors = {
                LFront,
                LRear,
                RFront,
                RRear
        };

        switch(direction){
            case 1: // forward
                LFront.setTargetPosition(target);
                LRear.setTargetPosition(target);
                RFront.setTargetPosition(target);
                RRear.setTargetPosition(target);
                break;

            case 2: // back
                LFront.setTargetPosition(-target);
                LRear.setTargetPosition(-target);
                RFront.setTargetPosition(-target);
                RRear.setTargetPosition(-target);
                break;

            case 3: // left
                LFront.setTargetPosition(-target);
                LRear.setTargetPosition(target);
                RFront.setTargetPosition(target);
                RRear.setTargetPosition(-target);
                break;

            case 4: // right
                LFront.setTargetPosition(target);
                LRear.setTargetPosition(-target);
                RFront.setTargetPosition(-target);
                RRear.setTargetPosition(target);
                break;

            case 5: // rotate left
                LFront.setTargetPosition(-target);
                LRear.setTargetPosition(-target);
                RFront.setTargetPosition(target);
                RRear.setTargetPosition(target);
                break;

            case 6: // rotate right
                LFront.setTargetPosition(target);
                LRear.setTargetPosition(target);
                RFront.setTargetPosition(-target);
                RRear.setTargetPosition(-target);
                break;
        }

        for(DcMotor motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(!ramp)
                motor.setPower(targetSpeed);

            motor.setPower(speed);
        }

        runtime.reset();
        rampTimer.reset();
        while(
                opModeIsActive()
                        && (runtime.seconds() < timeout)
                        && (LFront.isBusy() && LRear.isBusy() && RFront.isBusy() && RRear.isBusy())
        ){
            if(ramp){ //only ramp up and down if ramp is true
                double averageMotorDistance = ( // creates a variable equal to the average distance between each motor's current position and its target position
                        Math.abs(LFront.getCurrentPosition() - LFront.getTargetPosition()) +
                                Math.abs(LRear.getCurrentPosition()  - LRear.getTargetPosition() ) +
                                Math.abs(RFront.getCurrentPosition() - RFront.getTargetPosition()) +
                                Math.abs(RRear.getCurrentPosition()  - RRear.getTargetPosition() )
                )/4.0;

                if(speed >= targetSpeed && !set){
                    distanceToReachTargetSpeed = target - averageMotorDistance;
                    set = true;
                }

                if(Math.abs(averageMotorDistance) > target/2.0 && speed < targetSpeed){ // if not half way through and not reached target speed then speed up
                    if (rampTimer.milliseconds() >= incrementInterval){
                        speed += increment;
                        rampTimer.reset();
                    }
                }else if( // if half way through or at the distance required to ramp down, and speed is greater than powerMin, then slow down
                        (
                                averageMotorDistance <= distanceToReachTargetSpeed
                                        || averageMotorDistance > target/2.0
                        ) && speed >= powerMin
                ){
                    if (rampTimer.milliseconds() >= incrementInterval){
                        speed -= increment;
                        rampTimer.reset();
                    }else{ // in case speed goes lower than powerMin then set it to powerMin
                        speed = powerMin;
                    }
                }
                LFront.setPower(speed);
                LRear.setPower(speed);
                RFront.setPower(speed);
                RRear.setPower(speed);
            } else {
                LFront.setPower(targetSpeed);
                LRear.setPower(targetSpeed);
                RFront.setPower(targetSpeed);
                RRear.setPower(targetSpeed);
            }




            //telemetry
            telemetry.addData("*** Running setMotorTargets() ***", "\n");
            telemetry.addData("Speed: ", speed);
            telemetry.addData("Elapsed time: ", runtime.seconds());

            telemetry.addData("Running to",  " %7d :%7d :%7d :%7d",
                    LFront.getTargetPosition(),  LRear.getTargetPosition(), RFront.getTargetPosition(), RRear.getTargetPosition());

            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    LFront.getCurrentPosition(), RFront.getCurrentPosition(), LRear.getCurrentPosition(), RRear.getCurrentPosition());

            telemetry.update();

        }

        ResetMotors();
    }
}
