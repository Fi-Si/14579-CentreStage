package org.firstinspires.ftc.teamcode;


import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Basic: Linear OpMode V3", group="Linear OpMode")
//@Disabled
public class BasicOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private AprilTagProcessor aprilTag;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagProcessor myAprilTagProcessor;

        //Create a new AprilTag Proccessor Builder Project and assigning it to a variable
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        //Optional: Specify a custom library of AprilTags
        //myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);     //The OpMode must have already created a Library

        //Set the default tag library
        //"CurrentGameTagLibrary should show the current game library
        myAprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());


        //Don't think Library Builder is necessary nor Custom April Tags. Custom April Tag and Library Builder setup below:


        //AprilTagMetadata myAprilTagMetadata;
        //AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        //AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        //AprilTagLibrary myAprilTagLibrary;
        //AprilTagProcessor myAprilTagProcessor;

// Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        //myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

// Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
// Get the AprilTagLibrary for the current season.
        //myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

// Create a new AprilTagMetdata object and assign it to a variable.
        //myAprilTagMetadata = new AprilTagMetdata(55, "Our Awesome Team Tag", 3.5, DistanceUnit.INCH);

// Add a tag to the AprilTagLibrary.Builder.
        //myAprilTagLibraryBuilder.addTag(myAprilTagMetadata);

// Build the AprilTag library and assign it to a variable.
        //myAprilTagLibrary = myAprilTagLibraryBuilder.build();

// Create a new AprilTagProcessor.Builder object and assign it to a variable.
        //myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

// Set the tag library.
        //myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

// Build the AprilTag processor and assign it to a variable.
        //myAprilTagProcessor = myAprilTagProcessorBuilder.build();


        //Optional: set other custom features of the AprilTag Processor (4 are shown here)
        myAprilTagProcessorBuilder.setDrawTagID(true);                  //Default: ture, for all detections
        myAprilTagProcessorBuilder.setDrawTagOutline(true);            //Default: true, when tag size was provided (thus eligible for pose estimation)
        myAprilTagProcessorBuilder.setDrawAxes(true);                  //Default: false
        myAprilTagProcessorBuilder.setDrawCubeProjection(true);        //Default: false

        // Create an AprilTagProcessor by calling build()
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();


        //AprilTag Java Builder Chain Example - - - - - - - - -


        //AprilTagProcessor myAprilTagProcessor;

        //myAprilTagProcessor = new AprilTgaProcessor.Builder()
        //.setTagLibrary(myAprilTagLibrary)
        //.setDrawTagID(true)
        //.setDrawTagOutline(true)
        //.setDrawAxes(true)
        //.setDrawCubeProjection(true)
        //.build();


        //TfodProcessor myTfodProcessor;
        // Create the TensorFLow Obeject Detection Processor and assing it to a variable
        //myTfodProcessor = TfodProcessor.easyCreateWithDefaults();


        //TensorFlow Initialization - Builder  - - - - - - -

        TfodProcessor.Builder myTfodProcessorBuilder;
        TfodProcessor myTfodProcessor;
// Create a new TFOD Processor Builder object.
        myTfodProcessorBuilder = new TfodProcessor.Builder();

// Optional: set other custom features of the TFOD Processor (4 are shown here).
        myTfodProcessorBuilder.setMaxNumRecognitions(10);  // Max. number of recognitions the network will return
        myTfodProcessorBuilder.setUseObjectTracker(true);  // Whether to use the object tracker
        myTfodProcessorBuilder.setTrackerMaxOverlap((float) 0.2);  // Max. % of box overlapped by another box at recognition time
        myTfodProcessorBuilder.setTrackerMinSize(16);  // Min. size of object that the object tracker will track

// Create a TFOD Processor by calling build()
        myTfodProcessor = myTfodProcessorBuilder.build();

        //TensorFlow Java Builder Chain - - - - - - - - -


        //TfodProcessor myTfodProcessor;

        //myTfodProcessor = new TfodProcessor.Builder()
        //      .setMaxNumRecognitions(10)
        //    .setUseObjectTracker(true)
        //  .setTrackerMaxOverlap((float) 0.2)
        //.setTrackerMinSize(16)
        //.build();

        // Enable or disable the AprilTag processor.
        //myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);


        //VisionPortal Initialization - Easy - - - - - -

        //VisionPortal myVisionPortal;

// Create a VisionPortal, with the specified camera and AprilTag processor, and assign it to a variable.
        //myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), myAprilTagProcessor);


        //VisionPortal Initialization - Builder - - - - - - - - -

        VisionPortal.Builder myVisionPortalBuilder;
        VisionPortal myVisionPortal;

// Create a new VisionPortal Builder object.
        myVisionPortalBuilder = new VisionPortal.Builder();

// Specify the camera to be used for this VisionPortal.
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));      // Other choices are: RC phone camera and "switchable camera name".

// Add the AprilTag Processor to the VisionPortal Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);       // An added Processor is enabled by default.
        myVisionPortalBuilder.addProcessor(myTfodProcessor);

// Optional: set other custom features of the VisionPortal (4 are shown here).
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableLiveView(true);      // Enable LiveView (RC preview).
        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

// Create a VisionPortal by calling build()
        myVisionPortal = myVisionPortalBuilder.build();

        // Enable or disable the AprilTag processor.
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

// Enable or disable the TensorFlow Object Detection processor.
        myVisionPortal.setProcessorEnabled(myTfodProcessor, true);


        //Java Builder Chain - - - - - - -

        //VisionPortal myVisionPortal;

        //myVisionPortal = new VisionPortal.Builder()
        //      .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        //    .addProcessor(myAprilTagProcessor)
        //  .setCameraResolution(new Size(640, 480))
        //.setStreamFormat(VisionPortal.StreamFormat.YUY2)
        //     .enableCameraMonitoring(true)
        // .setAutoStopLiveView(true)
        //   .build();


// Enable or disable the AprilTag processor.
        // myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            //AprilTag FOR Loop:



            List<AprilTagDetection> myAprilTagDetections;  // list of all detections
            int myAprilTagIdCode;                           // ID code of current detection, in for() loop

// Get a list of AprilTag detections.
            myAprilTagDetections = myAprilTagProcessor.getDetections();

// Cycle through through the list and process each AprilTag.
            for ( AprilTagDetection tempTag : myAprilTagDetections) {

                if (tempTag.metadata != null) {  // This check for non-null Metadata is not needed for reading only ID code.
                    myAprilTagIdCode = tempTag.id;

                    // Now take action based on this tag's ID code, or store info for later action.m

                    //Retrive AprilTag Name:
//                    AprilTagDetection myAprilTagDetection;
                    String myAprilTagName;
                    myAprilTagName = tempTag.metadata.name;

                    //Asign variables for AprilTagDetection:

                    AprilTagDetection myAprilTagDetection;
                    myAprilTagDetection = tempTag;
                    double myTagPoseX = myAprilTagDetection.ftcPose.x;
                    double myTagPoseY = myAprilTagDetection.ftcPose.y;
                    double myTagPoseZ = myAprilTagDetection.ftcPose.z;
                    double myTagPosePitch = myAprilTagDetection.ftcPose.pitch;
                    double myTagPoseRoll = myAprilTagDetection.ftcPose.roll;
                    double myTagPoseYaw = myAprilTagDetection.ftcPose.yaw;


                    //Caluclated Extension of the Basic Pose
                    // Range, direct (point-to-point) distance to the tag center

                    //Bearing, the angle the camera must turn (left/right) to point directly at the tag center

                    //Elevation, the angle the camera must tilt (up/down) to point directly at the tag center

                    double myTagPoseRange = myAprilTagDetection.ftcPose.range;
                    double myTagPoseBearing = myAprilTagDetection.ftcPose.bearing;
                    double myTagPoseElevation = myAprilTagDetection.ftcPose.elevation;


                    //April Tag Detection (To remain after variable definitions:



                    int myAprilTagIDCode = myAprilTagDetection.id;

                    telemetryAprilTag();
                }
                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

                // Tank Mode uses one stick to control each wheel.
                // - This requires no math, but it is hard to drive forward slowly and keep straight.
                // leftPower  = -gamepad1.left_stick_y ;
                // rightPower = -gamepad1.right_stick_y ;

                // Send calculated power to wheels


                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
                telemetry.update();
            }
        }
    }






    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
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
        telemetry.addData("Range", AprilTagDetection.ftcPose.range);



    }
}

