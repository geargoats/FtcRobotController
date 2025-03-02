/* Copyright (c) 2022 FIRST. All rights reserved.
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
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;


/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Auto Drive", group="TFOD")
//@Disabled
public class AutoDriveByGyro_Linear_Apriltag_TFOD_Mechanam extends LinearOpMode {
    private static final boolean DEBUG = true;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;
    private TfodProcessor tfod;
    private static final String TFOD_MODEL_ASSET = "team_element2023.tflite";
    private static final String[] LABELS = {
            "blue_te","red_te"
    };
    private final double LEFT_TE_SIDE = 200;  //Where is the boarder for the left team element
    private final double RIGHT_TE_SIDE = 450; //Where is the boarder for the right team element
    private final int LEVEL_ARM = 2500;
    private final double DIST_MAX = 20; //if the Distance is greater than 20" then you are not left of a post


    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;

    /* Declare OpMode members. */
    private DistanceSensor sensorDistance;
    private DcMotor         leftBackDrive   = null;
    private DcMotor         rightBackDrive  = null;
    private DcMotor         leftFrontDrive   = null;
    private DcMotor         rightFrontDrive  = null;
    private DcMotor         angleMotor = null;
    private IMU             imu         = null;      // Control/Expansion Hub IMU
    private CRServo pixel_servo;// Right Front Pixel
    private CRServo pixel_servo_left;//Left Front Pixel
    private TouchSensor linear_slide_back;

    private double          robotHeading  = 0;
    private double          headingOffset = 0;
    private double          headingError  = 0;


    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  driveSpeed    = 0;
    private double  turnSpeed     = 0;
    private double  leftBackSpeed     = 0;
    private double  leftFrontSpeed = 0;
    private double  rightBackSpeed    = 0;
    private double  rightFrontSpeed    = 0;
    private int     leftBackTarget    = 0;
    private int     rightBackTarget   = 0;
    private int     leftFrontTarget    = 0;
    private int     rightFrontTarget   = 0;
    private int     angleTarget = 0;
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    // Store Color and Location info
    private boolean TE_color = false; //Blue = false, Red = true;
    private int TE_location = 0; //0 = Left, 1= Center, 2= Right;
    private String Te_position = "None"; //Temporary TE String
    private float TE_confidence = 0; // Default to 0 confidence
    private boolean pole_left;// If there is a pole left, and Red detected you are in backfield if there is a poll left and blue then you are in the frontfield.

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 28 ;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double     DRIVE_GEAR_REDUCTION    = 19.2 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = 1/0.0225300931183284; //Measured
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
    //                                                  (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     ROBOT_WIDTH_INCH        =  12.5; //From Center of wheels to Center of Wheels

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    static final double     TURN_SPEED              = 0.4;     // Max Turn speed to limit turn rate
    static final double     STRAFE_SPEED            = 0.5;     // Max Turn speed to limit turn rate
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
                                                               // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.03;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error for AprilTag
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected

   // @Override
    public void runOpMode() {
        pixel_servo = hardwareMap.get(CRServo.class, "pixel_holder");
        pixel_servo_left = hardwareMap.get(CRServo.class, "pixel_holder_2");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "range");


        initDoubleVision();
        visionPortal.setProcessorEnabled(tfod, true);
        visionPortal.setProcessorEnabled(aprilTag, false);

        //initAprilTag();

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        // Initialize the drive system variables.
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        angleMotor = hardwareMap.get(DcMotor.class,"lift_Motor");
        linear_slide_back = hardwareMap.get(TouchSensor.class, "linear_stop_back");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        angleMotor.setDirection(DcMotorSimple.Direction.FORWARD);//Moves Claw up

        // define initialization values for IMU, and then initialize it.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();


        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetryTfod();
            // Push telemetry to the Driver Station.
            telemetry.addData("Hub orientation", "Logo=%s   USB=%s\n ", logoDirection, usbDirection);
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
            telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
            telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
            telemetry.addData("Left Distance", String.format("%.01f in", sensorDistance.getDistance(DistanceUnit.INCH)));
            pole_left = sensorDistance.getDistance(DistanceUnit.INCH)<DIST_MAX; //True Poll to left
            telemetry.addData("Pole Detected:"," %s", (pole_left)?"True":"False");
            telemetry.update();

        }
        pixel_servo.setPower(-1);
        pixel_servo_left.setPower(-1);
        sleep(300);
        pixel_servo.setPower(1); //Hold Pixel Right Front
        pixel_servo_left.setPower(1);// Hold Pixel Left Front
        angleMotor.setPower(0);


        // Set the encoders for closed loop speed control, and reset the heading.
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetHeading();

        for (int i = 0; i<10; i++) {
            te_detector();
            sleep(20);
        }
        pixel_servo_left.setPower(1);
        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        switch (TE_location) {
            case 0:
                telemetry.addLine("Starting Left TE Location");
                left_pixel_place();
                break;
            case 1:
                telemetry.addLine("Starting Center TE Location");
                center_pixel_place();
                break;
            default:
                telemetry.addLine("Starting Right TE Location");
                right_pixel_place();
                break;

        }

        if (DEBUG) {
            ElapsedTime holdTimer = new ElapsedTime();
            holdTimer.reset();
            double holdTime = 2;

            // keep looping while we have time remaining.
            while (opModeIsActive() && (holdTimer.time() < holdTime)) {

                telemetryAprilTag();
                telemetry.update();
            }
        }

        while(opModeIsActive()) {

        }
        // Step through each leg of the path,
        // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        //telemetryAprilTag(); //First Read April tag



        telemetry.addData("Path", "Complete");
        telemetry.update();


        sleep(1000);  // Pause to display last telemetry message.
    }

    private void driveSpeed(double strafeSpeed, double v, double v1) {
    }

    /*
     * ====================================================================================================
     * Driving "Helper" functions are below this line.
     * These provide the high and low level methods that handle driving straight and turning.
     * ====================================================================================================
     */

    // **********  HIGH Level driving functions.  ********************


    /**
    *  Method to drive in a straight line, on a fixed compass heading (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backward.
    * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from the current robotHeading.
    */
    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftBackTarget = leftBackDrive.getCurrentPosition() + moveCounts;
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftBackDrive.setTargetPosition(leftBackTarget);
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);

            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, maxDriveSpeed, 0);


            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy()) && leftFrontDrive.isBusy() && rightFrontDrive.isBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(maxDriveSpeed,maxDriveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(maxDriveSpeed,0, 0);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



    /**
     *  Method to drive in a straight line straffing, on a fixed compass heading (angle), based on encoder counts.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the opmode running.
     *
     * @param maxDriveSpeed MAX Speed for forward/rev motion (range 0 to +1.0) .
     * @param distance   Distance (in inches) to move from current position to the right.  Negative distance means move left.
     * @param heading      Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from the current robotHeading.
     */
    public void driveStrafe(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = leftFrontDrive.getCurrentPosition() + moveCounts;
            rightFrontTarget = rightFrontDrive.getCurrentPosition() + moveCounts;
            leftBackTarget = leftBackDrive.getCurrentPosition() - moveCounts;
            rightBackTarget = rightBackDrive.getCurrentPosition() - moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            leftBackDrive.setTargetPosition(leftBackTarget);
            leftFrontDrive.setTargetPosition(leftFrontTarget);
            rightBackDrive.setTargetPosition(rightBackTarget);
            rightFrontDrive.setTargetPosition(rightFrontTarget);

            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot_Strafe(maxDriveSpeed, 0);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (leftBackDrive.isBusy() && rightBackDrive.isBusy()) && leftFrontDrive.isBusy() && rightFrontDrive.isBusy()) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot_Strafe(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(maxDriveSpeed,0, 0);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(maxTurnSpeed,0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(maxTurnSpeed,0, 0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(maxTurnSpeed,0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(maxTurnSpeed,0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }


    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot_april(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }


    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double max_speed, double drive, double turn) {
        max_speed =  (max_speed>1.0) ? 1.0 : max_speed; // if max speed is greater than 1.0 return 1.0
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftBackSpeed  = drive - turn;
        leftFrontSpeed  = drive - turn;
        rightBackSpeed = drive + turn;
        rightFrontSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed)),Math.max( Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed)));
        if (max > max_speed)
        {
            leftBackSpeed /= max;
            leftFrontSpeed /= max;
            rightBackSpeed /= max;
            rightFrontSpeed /= max;
        }

        leftBackDrive.setPower(leftBackSpeed*max_speed);
        leftFrontDrive.setPower(leftFrontSpeed*max_speed);
        rightBackDrive.setPower(rightBackSpeed*max_speed);
        rightFrontDrive.setPower(rightFrontSpeed*max_speed);
    }

    public void moveRobot_Strafe(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftBackSpeed  = drive + turn;
        leftFrontSpeed  = drive - turn;
        rightBackSpeed = drive + turn;
        rightFrontSpeed = drive - turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed)),Math.max( Math.abs(leftFrontSpeed), Math.abs(rightFrontSpeed)));
        if (max > 1.0)
        {
            leftBackSpeed /= max;
            leftFrontSpeed /= max;
            rightBackSpeed /= max;
            rightFrontSpeed /= max;
        }

        leftBackDrive.setPower(leftBackSpeed);
        leftFrontDrive.setPower(leftFrontSpeed);
        rightBackDrive.setPower(rightBackSpeed);
        rightFrontDrive.setPower(rightFrontSpeed);
    }

    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
            telemetry.addData("Target Pos BL:BR",  "%7d:%7d",      leftBackTarget,  rightBackTarget);
            telemetry.addData("Actual Pos BL:BR",  "%7d:%7d",      leftBackDrive.getCurrentPosition(),
                    rightBackDrive.getCurrentPosition());
            telemetry.addData("Target Pos FL:FR",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            telemetry.addData("Actual Pos FL:FR",  "%7d:%7d",      leftFrontDrive.getCurrentPosition(),
                    rightFrontDrive.getCurrentPosition());
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftBackSpeed, rightBackSpeed);
        telemetry.addData("Wheel Speeds FL:FR.", "%5.2f : %5.2f", leftFrontSpeed, rightFrontSpeed);
        telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        //       Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //        return angles.firstAngle;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    private void initDoubleVision() {
        // -----------------------------------------------------------------------------------------
        // AprilTag Configuration
        // -----------------------------------------------------------------------------------------

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // -----------------------------------------------------------------------------------------
        // TFOD Configuration
        // -----------------------------------------------------------------------------------------

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();

        // -----------------------------------------------------------------------------------------
        // Camera Configuration
        // -----------------------------------------------------------------------------------------

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessors(tfod, aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessors(tfod, aprilTag)
                    .build();
        }
    }   // end initDoubleVision()

    private void telemetryTfod() {
        float confidence = 80;
        String temp_label = "";
        String red_label = "red_te";



        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            if (recognition.getConfidence()*100 > confidence  ) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                temp_label = recognition.getLabel();
                telemetry.addData("Test:", "%s", temp_label);
                TE_color = (temp_label.equals(red_label));
                if (x < LEFT_TE_SIDE) {
                    TE_location = 0;  //Left
                    Te_position = "Left";
                } else if (x > RIGHT_TE_SIDE) {
                    TE_location = 2;  //Right
                    Te_position = "Right";
                } else {
                    TE_location = 1; //Center
                    Te_position = "Center";
                }
            }

        }
        telemetry.addData(""," ");
        telemetry.addData("Image", "%s", (TE_color) ? "Red": "Blue");
        telemetry.addData("Position", "%s", Te_position);

    }

    private void te_detector(){
        float confidence = 80;
        String red_label = "red_te";
        String temp_label = "";

//        sleep(500);
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());
        for (Recognition recognition : currentRecognitions) {
            telemetry.addData("Confidence", "%.0f %% gt %.0f %%", recognition.getConfidence()*100,TE_confidence);
            if (recognition.getConfidence()*100 > confidence  && TE_confidence < recognition.getConfidence()) {
                TE_confidence = recognition.getConfidence();
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                temp_label = recognition.getLabel();
                telemetry.addData("Test:", "%s", temp_label);
                TE_color = (temp_label.equals(red_label));
                if (x < LEFT_TE_SIDE) {
                    TE_location = 0;  //Left
                    Te_position = "Left";
                } else if (x > RIGHT_TE_SIDE) {
                    TE_location = 2;  //Right
                    Te_position = "Right";
                } else {
                    TE_location = 1; //Center
                    Te_position = "Center";
                }
            }

        }
        telemetry.addData(""," ");
        telemetry.addData("Image", "%s", (TE_color) ? "Red": "Blue");
        telemetry.addData("Position", "%s", Te_position);
        telemetry.update();

    }

    /**
     * Add telemetry about AprilTag detections.
     */
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

    }   // end method telemetryAprilTag()

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
    }

    private void left_pixel_place (){
        //Place code here
        pixel_servo_left.setPower(1);
        driveStraight(0.4,16,0);
//        driveStraight(0.5,24,90);
        turnToHeading(TURN_SPEED,30);
        pixel_servo_left.setPower(-1.0);
        if (TE_color)   driveStraight(DRIVE_SPEED,10,30);
        else     driveStraight(DRIVE_SPEED,8.5,30);
        pixel_servo_left.setPower(-0.5);
        holdHeading(TURN_SPEED,30,0.4);
        driveStraight(DRIVE_SPEED,-9,30);
//        holdHeading(DRIVE_SPEED,90,0.2);
        if (pole_left) {
            turnToHeading(TURN_SPEED, -90);
            holdHeading(DRIVE_SPEED, -90, 0.2);
            if(!TE_color) { ///If blue center straffe
                driveStrafe(STRAFE_SPEED, -48, -90);
            } else {
                red_back_stage();
            }
        } else {
            turnToHeading(TURN_SPEED, 90);
            holdHeading(DRIVE_SPEED, 90, 0.2);
            if(TE_color) { //IF red center Straffe
                driveStrafe(STRAFE_SPEED, 48, 90);
            } else {
                blue_back_stage();
            }
        }
    }
    private void  center_pixel_place (){
    /*    //Place code here
        pixel_servo_left.setPower(1);
        driveStraight(0.4,14,0);
        if (pole_left){
            turnToHeading(TURN_SPEED,45);
            driveStrafe(STRAFE_SPEED,22,45);
            turnToHeading(TURN_SPEED,90);
            driveStrafe(0.4,16,90);
            pixel_servo_left.setPower(-1);
            driveStraight(DRIVE_SPEED,-4,90);
            driveStrafe(STRAFE_SPEED,16,90);
        } else {
            turnToHeading(TURN_SPEED,-45);
            driveStrafe(STRAFE_SPEED,-22,-45);
            turnToHeading(TURN_SPEED,-90);
            driveStrafe(0.4,-16,-90);
            pixel_servo_left.setPower(-1);
            driveStraight(DRIVE_SPEED,-4,-90);
            driveStrafe(STRAFE_SPEED,-16,-90);

        }
*/
        pixel_servo_left.setPower(0);
        driveStraight(DRIVE_SPEED,28.5,0);
        pixel_servo_left.setPower(-1);
        driveStraight(DRIVE_SPEED,-10,0);
        turnToHeading(TURN_SPEED,-90);
        if (pole_left) { //Red Turn Right
            if (TE_color) {//Red Back Stage
                red_back_stage();
            } else {
                turnToHeading(turnSpeed, -90);
            }
        } else {
            if (TE_color) { //Red Front Stage
                turnToHeading(turnSpeed, 90);
            }else { // Blue Back Stage
                blue_back_stage();
            }
        }
    }
    private void right_pixel_place(){
        //Place code here
        pixel_servo_left.setPower(1);
        driveStraight(0.4,14,0);
//        driveStraight(0.5,24,90);
        turnToHeading(TURN_SPEED,-40);
        driveStrafe(STRAFE_SPEED,-1,-40);
        pixel_servo_left.setPower(-1);
        driveStraight(DRIVE_SPEED,12.5,-40);
        holdHeading(TURN_SPEED,-40,0.4);
        driveStraight(DRIVE_SPEED,-12,-40);
//        holdHeading(DRIVE_SPEED,90,0.2);
        if (pole_left) {
            turnToHeading(TURN_SPEED, -90);
            holdHeading(DRIVE_SPEED, -90, 0.2);
            if(!TE_color) { ///If blue front stage
                driveStrafe(STRAFE_SPEED, -48, -90);
            } else { //Red Back Stage
                red_back_stage();
            }
        } else {
            turnToHeading(TURN_SPEED, 90);
            holdHeading(DRIVE_SPEED, 90, 0.2);
            if(TE_color) { //IF red front stage
                driveStrafe(STRAFE_SPEED, 24, 90);
            } else { //Blue back Stage
                blue_back_stage();
            }
        }
    }
    private void red_back_stage ( ) {
        //Do Something
        driveStrafe(STRAFE_SPEED,14,-90);
        driveStraight(DRIVE_SPEED,43,-90);
    }

    private void blue_back_stage () {
        //Do Something



        //doing something

    }

   /* private void orient_on_red_apriltag(){
        targetFound = false;
        int find_tag = 7;
        desiredTag  = null;
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if ((detection.id == find_tag) ) {
                // Yes, we want to use this tag.
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                // This tag is in the library, but we do not want to track it right now.
                telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
            }
        }
        if (targetFound) {
            telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData("\n>","Red 7 Apriltag not foundt\n");
        }
        double  rangeError      = (desiredTag.ftcPose.range - 26);
        double  headingError    = (desiredTag.ftcPose.bearing - 17);
        double  yawError        = (desiredTag.ftcPose.yaw - -17.0);

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        drive  = Range.clip(rangeError * SPEED_GAIN, -DRIVE_SPEED, DRIVE_SPEED);
        turn   = Range.clip(headingError * TURN_GAIN, -TURN_SPEED, TURN_SPEED) ;
        strafe = Range.clip(-yawError * STRAFE_GAIN, -STRAFE_SPEED, STRAFE_SPEED);

        telemetry.addData("Error", "rangeErr %5.1fin, headErr %3.0f deg, yawErr %3.0f deg", rangeError,headingError,yawError);
        telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        telemetry.update();

        // Apply desired axes motions to the drivetrain.
        if ((Math.abs(drive)+Math.abs(strafe)+Math.abs(turn))>0.1) moveRobot_april(drive, strafe, turn);
        sleep(50);



    }*/

}
