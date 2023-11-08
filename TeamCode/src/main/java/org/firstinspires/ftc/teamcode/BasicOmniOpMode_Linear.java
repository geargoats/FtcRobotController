package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import java.time.Clock;

/**
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear Opmode")
//@Disabled
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private CRServo drone_servo;
    private CRServo pixel_servo;
    private CRServo pixel_servo_2;
    private DcMotor linear_motor;
    private DcMotor angleMotor;
    private TouchSensor linear_stop_btm;
    private TouchSensor linear_slide_back;
    private TouchSensor linear_slide_front;
    private final double     LIFT_SPEED             = 1;
    float Speed_multiplier;
    float min_speed = (float)0.3;
    float robot_speed;
    boolean drone_delay = false;
    double delay_mili=getRuntime();
    boolean pixel_toggle = true;
    boolean changed = false; //Outside of loop()
    boolean changed2 = false; //Outside of loop()




    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        drone_servo = hardwareMap.get(CRServo.class,"drone_servo");
        pixel_servo = hardwareMap.get(CRServo.class, "pixel_holder");
        pixel_servo_2 = hardwareMap.get(CRServo.class, "pixel_holder_2");
        linear_motor = hardwareMap.get(DcMotor.class,"linear_motor");
        angleMotor = hardwareMap.get(DcMotor.class,"lift_Motor");
        //linear_stop_btm = hardwareMap.get(TouchSensor.class, "linear_stop");
        linear_slide_back = hardwareMap.get(TouchSensor.class, "linear_stop_back"); //Tells you when you hit the back angle
        linear_slide_front = hardwareMap.get(TouchSensor.class, "linear_stop_front");//Tells you when you hit the front angle
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        linear_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        linear_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        double linear_speed_stick;

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        pixel_servo.setPower(-1);
        linear_motor.setPower(0);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            //Increase Speed based on trigger;
            Speed_multiplier = gamepad1.right_trigger;

            linear_motor.setPower(0);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max; // equal to leftFrontPower = leftFrontPower / max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            robot_speed = min_speed + ((1-min_speed) * Speed_multiplier);

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontPower *=robot_speed;
            rightBackPower *=robot_speed;
            leftBackPower  *= robot_speed;
            rightFrontPower *= robot_speed;
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Angle Encoder Value", "%d",angleMotor.getCurrentPosition());
            telemetry.update();
            if(gamepad1.y && !changed) {
                if(pixel_servo.getPower() == -1) pixel_servo.setPower(1);
                else pixel_servo.setPower(-1);
                changed = true;
            } else if(!gamepad1.y) changed = false;
            if(gamepad1.x && !changed2) {
                if(pixel_servo_2.getPower() == -1) pixel_servo_2.setPower(1);
                else pixel_servo_2.setPower(-1);
                changed2 = true;
            } else if(!gamepad1.x) changed2 = false;
            if (gamepad1.a) {
                if (delay_mili+1 < getRuntime()){
                    drone_servo.setPower(-1);
                    sleep(500);
                } else {
                    drone_servo.setPower(1);
                }
            } else {
                drone_servo.setPower(1);
                delay_mili = getRuntime();
            }
            linear_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad1.dpad_up) {
                linear_motor.setPower(-1);
            } else if(gamepad1.dpad_down) {
                linear_motor.setPower(1);
            } else {
                linear_motor.setPower(0);
            }
            angleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (!linear_slide_back.isPressed() && gamepad1.left_bumper){
                    angleMotor.setPower(0.2);
            } else if(gamepad1.right_bumper){
                angleMotor.setPower(-0.2);
            } else {
                angleMotor.setPower(0);
            }
            /*if ((gamepad1.dpad_down || gamepad1.dpad_up )) {
                double speed_dpad = 0.1;
                double dpad_up = (gamepad1.dpad_up) ? speed_dpad : 0.0;
                double dpad_down = (gamepad1.dpad_down) ? speed_dpad*-1.0 : 0.0;
                linear_speed_stick = (gamepad1.dpad_down) ? dpad_down : dpad_up;
                if (linear_stop_btm.isPressed()) {
                    linear_motor.setPower(0);
                } else {
                    linear_motor.setPower(linear_speed_stick);
                }
            } else {
                //linear_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                telemetry.update();

            }*/
        }
    }
}
