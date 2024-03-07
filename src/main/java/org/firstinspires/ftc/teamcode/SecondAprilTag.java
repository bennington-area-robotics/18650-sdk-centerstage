package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import java.util.List;


@TeleOp(name = "2nd April Tag (Java)")

public class SecondAprilTag extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private DcMotor LFront;
    private DcMotor RFront;
    private DcMotor LRear;
    private DcMotor RRear;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;


    ElapsedTime PIDTimer = new ElapsedTime();

    double integralSum = 0;
    double lastError = 0;
    double Kp = 0.5, Ki = 0.01, Kd = 0.01, Kf = 0.1;


    @Override
    public void runOpMode() {
        LFront = hardwareMap.get(DcMotor.class, "LFront");
        RFront = hardwareMap.get(DcMotor.class, "RFront");
        LRear = hardwareMap.get(DcMotor.class, "LRear");
        RRear = hardwareMap.get(DcMotor.class, "RRear");
        LFront.setDirection(DcMotorSimple.Direction.REVERSE);
        LFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                telemetryAprilTag();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()



    //returns a power level based on PIDF variables and reference vs state
    private double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += (error * PIDTimer.seconds());
        double derivative = (error-lastError)/PIDTimer.seconds();
        lastError = error;
        PIDTimer.reset();
        return ((Kp*error) + (Ki*derivative) + (Kd*integralSum) + (Kf*reference));
    }


    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()


                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }


        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();



    }   // end method initAprilTag()



    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());


        // Step through the list of detections and display info for each one.
        boolean foundTag = false;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                String alliance = "BlueAllianceCenter";


                double motorPower;
                double absoluteState = 0;
                absoluteState = detection.ftcPose.x;


                if(detection.metadata.name == alliance){
                    foundTag = true;
                    if(true==true && detection.ftcPose.x > 0.1 || detection.ftcPose.x < 0.1){
                        motorPower = PIDControl(detection.ftcPose.x, absoluteState - detection.ftcPose.x)/10;
                        LFront.setPower(motorPower);
                        RFront.setPower(motorPower*-1);
                        LRear.setPower(motorPower*-1);
                        RRear.setPower(motorPower);
                    }else{
                        telemetry.addLine("TEST HI HELLO THERE HOW ARE U DOING");
                        LFront.setPower(0);
                        RFront.setPower(0);
                        LRear.setPower(0);
                        RRear.setPower(0);
                    }

                }

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

        if(!foundTag){
            LFront.setPower(0);
            RFront.setPower(0);
            LRear.setPower(0);
            RRear.setPower(0);
        }


        // Pitch is the measure of rotation about the X axis
        // Roll is the measure of rotation about the Y axis
        // Heading, or Yaw, is the measure of rotation about the Z axis



    }   // end method telemetryAprilTag()

}   // end class

// double xMin = 0;
// double xMax = 0.1;
// double yMin = 3.5;
// double yMax = 5.0;
// double zMin = 0;
// double zMax = 0.2;

// Y axis points straight outward from the camera lens center
// X axis points to the right, perpendicular to the Y axis
// Z axis points upward, perpendicular to Y and X