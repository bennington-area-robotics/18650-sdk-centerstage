/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


@Autonomous(name="AutonomousModeBlueClose", group="Robot")

public class AutonomousBlueClose extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/red-blue-training-2.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "blue", "red"
    };

    private DcMotor 
        LFront,
        RFront,
        LRear,
        RRear
    ;
    private DcMotor gantryMotor;
    private DcMotor carwashCollector;
    private Servo clawCollector;
    private Servo rampLifter;
    private Servo clawCollectorRotation;

    private ElapsedTime 
        runtime = new ElapsedTime(), //timer used to limit runtime of functions and misc
        PIDTimer = new ElapsedTime(), //timer used for PID
        rampTimer = new ElapsedTime() //timer used to control ramp up speed in setMotorTargets
    ;

    private double
        zoom = 5.0,
        integralSum = 0,
        lastError = 0
    ;

    private final double[] K1 = {0.5, 0.01, 0.01, 0.1};
    private final double turnDegreeConversionNum = 12.51577;
    private final double sidewaysConversionNum = 50.6936508;
    private final double forwardConversionNum = 47.232;
    private int randomizationPosition = 1; //randomization position, defaults to 1 (left)

    //arrays of movement directions for deliverToBackboard()
    private int[]
        blueTurns = {5, 3, 4},
        redTurns = {6, 4, 3}
    ;
    private String alliance = "blue"; //alliance, defaults to blue
    private int wait = 0;

    //The variable to store our instance of the TensorFlow Object Detection processor.
    private TfodProcessor tfod;
    //The variable to store our instance of the vision portal.
    private VisionPortal visionPortal;
    //The variable to store our instance of the AprilTag processor.
    private AprilTagProcessor aprilTag;

    /**
     * sets the motor power of all DC motors to 0 and resets the encoders
     */
    private void ResetMotors() {
        DcMotor[] motors = {
            LFront, 
            LRear, 
            RFront, 
            RRear,
            gantryMotor,
            carwashCollector,
        };

        for(DcMotor motor : motors){
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * initializes all the motors
     */
    private void InitializeMotors() {

        LFront = hardwareMap.get(DcMotor.class, "LFront");
        LRear = hardwareMap.get(DcMotor.class, "LRear");
        RFront = hardwareMap.get(DcMotor.class, "RFront");
        RRear = hardwareMap.get(DcMotor.class, "RRear");
        gantryMotor = hardwareMap.get(DcMotor.class, "gantryMotor");
        carwashCollector = hardwareMap.get(DcMotor.class, "carwashCollector");

        rampLifter = hardwareMap.get(Servo.class, "rampLifter");
        clawCollector = hardwareMap.get(Servo.class, "clawCollectorJaws");
        clawCollectorRotation = hardwareMap.get(Servo.class, "clawCollectorRotation");

        LFront.setDirection(DcMotor.Direction.REVERSE);
        LRear.setDirection(DcMotorSimple.Direction.FORWARD);
        RFront.setDirection(DcMotorSimple.Direction.FORWARD);
        RRear.setDirection(DcMotorSimple.Direction.REVERSE);
        gantryMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        carwashCollector.setDirection(DcMotorSimple.Direction.FORWARD);

        //rampLifter.setPosition(1);
        clawCollector.setPosition(0.5);
        clawCollectorRotation.setPosition(0.2);

        ResetMotors();
    }

    /**
     * Moves the robot-base in a specified direction to a target position.
     * @param direction: direction that the drive base will move (forward, back, left, right, rotate left, rotate right)
     * @param target: the amount of rotation ticks the motors will move
     * @param timeout: ignore this ngl, set it to 4 or something it's unimportant
     * @param targetSpeed: the speed the robot will ramp up to
     * @param ramp: the rate at which the robot ramps, set to 0 if you don't want to ramp
     */
    private void setMotorTargets(int direction, int target, int timeout, double targetSpeed, double ramp) {
        double 
            powerMin = 0.2,      // motor power at beginning and end of ramp up and down
            speed = powerMin,   // initial motor power
            increment = ramp,   // motor power increment
            distanceToReachTargetSpeed = 0
        ;

        double incrementInterval = 0.25; // interval between incrementing power (in seconds)
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

            if(ramp < 0.01) {
                speed = targetSpeed;
                motor.setPower(speed);
            }
            motor.setPower(speed);
        }

        runtime.reset();
        rampTimer.reset();
        while(
            opModeIsActive()
            && (runtime.seconds() < timeout)
            && (LFront.isBusy() && LRear.isBusy() && RFront.isBusy() && RRear.isBusy())
        ){
            if (ramp > 0.00) {
                double averageMotorDistance = ( // creates a variable equal to the average distance between each motor's current position and its target position
                        Math.abs(LFront.getCurrentPosition() - LFront.getTargetPosition()) +
                                Math.abs(LRear.getCurrentPosition() - LRear.getTargetPosition()) +
                                Math.abs(RFront.getCurrentPosition() - RFront.getTargetPosition()) +
                                Math.abs(RRear.getCurrentPosition() - RRear.getTargetPosition())
                ) / 4.0;

                if (speed >= targetSpeed && !set) {
                    distanceToReachTargetSpeed = target - averageMotorDistance;
                    set = true;
                }

                if (Math.abs(averageMotorDistance) > target / 2.0 && speed < targetSpeed) { // if not half way through and not reached target speed then speed up
                    if (rampTimer.seconds() >= incrementInterval) {
                        speed += increment;
                        rampTimer.reset();
                    }
                } else if ( // if half way through or at the distance required to ramp down, and speed is greater than powerMin, then slow down
                        averageMotorDistance <= distanceToReachTargetSpeed
                                || averageMotorDistance > target / 2.0
                ) {
                    if (rampTimer.seconds() >= incrementInterval) {
                        if (speed >= powerMin) {
                            speed -= increment;
                            rampTimer.reset();
                        } else { // in case speed goes lower than powerMin then set it to powerMin
                            speed = powerMin;
                        }
                    }
                }

            }

            LFront.setPower(speed);
            LRear.setPower(speed);
            RFront.setPower(speed);
            RRear.setPower(speed);

            //telemetry
            telemetry.addData("*** Running setMotorTargets() ***", "\n");

            telemetry.addData("Randomization position", randomizationPosition);
            telemetry.addData("Speed: ", LFront.getPower());
            telemetry.addData("Elapsed time: ", runtime.seconds());
            telemetry.addData("ramptimer: ", rampTimer.seconds());

            telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", 
                LFront.getTargetPosition(),  LRear.getTargetPosition(), RFront.getTargetPosition(), RRear.getTargetPosition());

            telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d",
                    LFront.getCurrentPosition(), RFront.getCurrentPosition(), LRear.getCurrentPosition(), RRear.getCurrentPosition());

            telemetry.update();

        }
        ResetMotors();
    }

    /**
     * Moves the gantryMotor to a specfied position
     * @param target: the target position for the gantryMotor, 0 is the minimum extension and 650 is the maximum extensoin
     * @param timeout: just set this to 4 or something it's useless
     * @param targetSpeed: the speed the gantryMotor will move at
     */
    private void setGantryTarget(int target, int timeout, double targetSpeed) {

        gantryMotor.setTargetPosition(target);
        gantryMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gantryMotor.setPower(targetSpeed);

        runtime.reset();

        while(opModeIsActive() && gantryMotor.isBusy() && (runtime.seconds() < timeout) ){
            gantryMotor.setPower(targetSpeed);

            //telemetry
            telemetry.addData("*** Running setMotorTargets() ***", "\n");
            telemetry.addData("Elapsed time: ", runtime.seconds());
            telemetry.update();
        }
        gantryMotor.setPower(0);
        gantryMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void runAutonomous(){
        findPropPosition(); // move forward, scan for team prop, return to original start position, and return the randomization position.
        moveToBackboardShort(blueTurns);
        moveArmToPosition();

        alignWithAprilTag(getTargetAprilTag());
        placeYellowPixel();
        resetArm();
        //park();
        sleep(2000);
        //reset();
    }

    @Override
    public void runOpMode() {

        InitializeMotors();
        initTfodAndAprilTags();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()){
            runAutonomous();
            /*moveArmToPosition();
            randomizationPosition = 2;
            alignWithAprilTag(getTargetAprilTag());
            placeYellowPixel();*/

            while (opModeIsActive())
            {
                telemetryAprilTag();
                telemetry.update();
                sleep(20);
            }
        }
        visionPortal.close();

    }

    /**
     * Initialize the TensorFlow Object Detection processor and April Tag Detection
     */
    private void initTfodAndAprilTags() {
        // Create the TensorFlow processor by using a builder.i
        tfod = new TfodProcessor.Builder()
            .setModelFileName(TFOD_MODEL_FILE)

            // The following default settings are available to un-comment and edit as needed to
            // set parameters for custom models.
            .setModelLabels(LABELS)
            .setModelAspectRatio(9.0 / 9.0)
            .build()
            //tfod.setZoom(zoom);
        ;
        
        aprilTag = new AprilTagProcessor.Builder()
            .build()
        ;

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    /**
     * @param reference
     * @param state
     * @param params
     * @return
     */
    private double PIDControl(double reference, double state, double[] params){
        double error = reference - state;
        integralSum += (error * PIDTimer.seconds());
        double derivative = (error-lastError)/PIDTimer.seconds();

        lastError = error;
        PIDTimer.reset();

        return(
            (params[0]*error) + 
            (params[1]*derivative) + 
            (params[2]*integralSum) + 
            (params[3]*reference)
        );
    }

    /**
     * runs until a team prop of our specified team is detected
     * @return: returning true kills the method that this one is called from; returning false causes it to continue scanning
     */
    public boolean detectProp() {
        runtime.reset();
        while (runtime.seconds() < 2.5 && opModeIsActive()){
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for(Recognition detection : currentRecognitions){
                if (detection.getLabel().equals(alliance)){
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * robot moves forward, before checking each spikemark for the teamprop, after which a purple pixel is placed
     * @return: returns the spike mark position the prop was placed on, which will be used later for aligning with the april tag
     */
    public void findPropPosition() {
        setMotorTargets(1,(int) (17* forwardConversionNum), 10, 0.7, 0.04); // move forward to the spike marks
        if(detectProp()){ // if prop is detected while facing forwards return position 2 (middle)
            randomizationPosition = 2;
            placePurplePixel();

            return;

        }

        setMotorTargets(5, (int) (40 * turnDegreeConversionNum), 10, 0.7, 0.03); // move from position 2 (middle) to position 1 (left)

        if(detectProp()){
            randomizationPosition = 1;
            placePurplePixel();
            return;
        }
        randomizationPosition = 3; //assume if not seen in position 1 or 2, that it is in position 3
        setMotorTargets(6, (int) (75 * turnDegreeConversionNum), 10, 0.7, 0.03); // rotate left, back to center
        placePurplePixel();
        setMotorTargets(5, (int) (35 * turnDegreeConversionNum), 10, 0.7, 0.03); // rotate left, back to center
    }

    /**
     * moves the robot to the backboard to place a pixel
     * @param turns: designates the order and types of turns the robot takes along its path, which are reversed between the different alliances
     */

    public void moveToBackboardShort(int[] turns){
        int turnDeg = 90;
        if (randomizationPosition == 1){
            turnDeg -= 50;
        }
        setMotorTargets(turns[0], (int) (turnDeg * turnDegreeConversionNum), 10, 0.7, 0.02); //turn left/right depending on team
        setMotorTargets(1, (int) (24 * forwardConversionNum), 10, 0.7, 0.02); //turn left/right depending on team
        setMotorTargets(turns[2], (int) ((1 + (6 * randomizationPosition) * sidewaysConversionNum)), 10, 0.55, 0.01);

    }

    /**
     * uses the alliance and randomization position to determine which april tag the robot will be aligning with
     * @return: returns the name of the april tag that the robot will align with
     */
    //find the string that identifies the target april tag and return it
    public String getTargetAprilTag(){
        String targetAprilTag = "";
        if(alliance.equals("blue")){
            switch(randomizationPosition){
                case 2:
                    targetAprilTag = "BlueAllianceCenter";
                    break;
                case 1:
                    targetAprilTag = "BlueAllianceLeft";
                    break;
                case 3:
                    targetAprilTag = "BlueAllianceRight";
                    break;
            }
        }else{
            switch(randomizationPosition){
                case 2:
                    targetAprilTag = "RedAllianceCenter";
                    break;
                case 1:
                    targetAprilTag = "RedAllianceLeft";
                    break;
                case 3:
                    targetAprilTag = "RedAllianceRight";
                    break;
            }
        }

        return targetAprilTag;
    }

    /**
     * aligns the robot with the april tag, first with the angle, then with the x, and lastly with the y
     */
    public void alignWithAprilTag(String target){

        double absolute;
        AprilTagDetection aprilTagDetection;

        aprilTagDetection = getAprilTagTargetDetection(target);

        while((aprilTagDetection == null || aprilTagDetection.metadata == null) && opModeIsActive()){
            aprilTagDetection = getAprilTagTargetDetection(target);
            telemetryAprilTag();
            telemetry.update();
        } //wait for april tag to be detected

        //align angle based on scale

        setMotorTargets(5, (int)(turnDegreeConversionNum * aprilTagDetection.ftcPose.yaw), 10, 0.35, 0.01);


        //align horizontally with the target april tag for a set time limit

        //runtime.reset();
        //set the inital relative position to act as a reference to PID
        /*absolute = aprilTagDetection.ftcPose.x;
        while(runtime.seconds() < 2.5 && opModeIsActive()) {
            aprilTagDetection = getAprilTagTargetDetection(target);
            telemetry.addData("Aligning with x", 1);
            telemetry.update();
            if(aprilTagDetection != null && aprilTagDetection.metadata != null ) {

                double motorPower = PIDControl(aprilTagDetection.ftcPose.x, absolute - aprilTagDetection.ftcPose.x, K1) / 10;
                LRear.setPower(-motorPower);
                RRear.setPower(motorPower);
                LFront.setPower(motorPower);
                RFront.setPower(-motorPower);

                if(Math.abs(aprilTagDetection.ftcPose.x) > 0.75) {
                    break;
                }
            }
        }
        LRear.setPower(0);
        RRear.setPower(0);
        LFront.setPower(0);
        RFront.setPower(0);
        */
        //moves forward/back to 10 inches from the target april tag for a set time limit

        do{
            aprilTagDetection = getAprilTagTargetDetection(target);
            telemetryAprilTag();
            telemetry.update();
        } while((aprilTagDetection == null || aprilTagDetection.metadata == null) && opModeIsActive());
        //wait for april tag to be detected again
        telemetry.addData("x", aprilTagDetection.ftcPose.x);
        telemetry.update();
        //sleep(10000);
        setMotorTargets(3, (int)(sidewaysConversionNum * -aprilTagDetection.ftcPose.x), 10, 0.35, 0.00);
        telemetry.addData("y", aprilTagDetection.ftcPose.y);
        telemetry.addData("range", aprilTagDetection.ftcPose.range);
        telemetry.addData("running to ", (aprilTagDetection.ftcPose.y - 6));
        telemetry.update();
        //sleep(10000);
        setMotorTargets(1, (int)((aprilTagDetection.ftcPose.y - 8) * forwardConversionNum), 2, 0.45, 0.01);

    }
    public void moveArmToPosition(){
        //setGantryTarget(630, 5, 0.7);
        clawCollectorRotation.setPosition(0.4);
        ///setGantryTarget(0, 5, 0.7);
        sleep(300);
    }
    public void placeYellowPixel(){
        setMotorTargets(3, (int)(1.5 * sidewaysConversionNum), 5, 0.5, 0.0);
        setMotorTargets(1, (int)(3 * forwardConversionNum), 5, 0.5, 0.1);
        sleep(500);
        clawCollector.setPosition(0);
        sleep(500);
    }
    public void resetArm(){
        setMotorTargets(2, (int)(4 * forwardConversionNum), 5, 0.5, 0.02);
        setGantryTarget(0, 4, 1);
        //setMotorTargets(1, (int)(4 * forwardConversionNum), 5, 0.5, 0.02);
    }

    public void placePurplePixel(){
        int inchesForward = 5;
        double rampPower = 0.15;
        if(randomizationPosition == 2) {
            inchesForward = 14;
            rampPower = 0.18;
        }
        if(randomizationPosition == 1){
            inchesForward = 6;
            setMotorTargets(5, (int)(10 * turnDegreeConversionNum), 10, 0.6, 0.015);

        }
        setMotorTargets(1, (int)(inchesForward * forwardConversionNum), 10, 0.5, 0.015);
        rampLifter.setPosition(0);
        sleep(50);
        carwashCollector.setDirection(DcMotorSimple.Direction.REVERSE);
        carwashCollector.setPower(rampPower);
        sleep(700);
        carwashCollector.setPower(0);
        rampLifter.setPosition(0.4);
        setMotorTargets(2, (int)(inchesForward * forwardConversionNum), 10, 0.5, 0.015);
    }

    public void park(){
        if(alliance.equals("blue")){ // alliance is blue
            setMotorTargets(4, (int)((33-6*randomizationPosition) * sidewaysConversionNum), 4, 0.4, 0);
        }else{ // alliance is red
            setMotorTargets(3, (int)((7+6*randomizationPosition) * sidewaysConversionNum), 4, 0.4, 0);
        }
    }

    public void reset(){
        setGantryTarget(620, 5, 0.7);
        //sleep(1000);
        clawCollectorRotation.setPosition(1);
        setGantryTarget(0, 5, 0.7);
    }

    public AprilTagDetection getAprilTagTargetDetection(String target){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // Step through the list of detections until found target
        for(AprilTagDetection detection : currentDetections){
            if(detection.metadata.name.equals(target)) {
                return detection;
            }
        }

        return null;
    }
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {

            if (detection.metadata != null) {
                //double angle = Math.PI/2;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }

        }   // end for() loop


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");


        // Pitch is the measure of rotation about the X axis
        // Roll is the measure of rotation about the Y axis
        // Heading, or Yaw, is the measure of rotation about the Z axis

    }
}


