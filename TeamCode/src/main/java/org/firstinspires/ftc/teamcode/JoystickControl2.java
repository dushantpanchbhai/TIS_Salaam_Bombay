package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Config
@TeleOp(name="Joystick Control 2", group="Linear Opmode")
//@Disabled
public class JoystickControl2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    // Arm and Extension
    private DcMotorEx arm, extension;

    public static PIDFCoefficients extpidcoeffs = new PIDFCoefficients(10, 0, 0, 0);

    // Claw
    private Servo leftClaw, rightClaw;

    // Carousel
    DcMotorEx carousel;

    // Sensors
    Rev2mDistanceSensor distance1;
    Rev2mDistanceSensor distance2;

    // Dashboard
    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry = dashboard.getTelemetry();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Chassis
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);

        // Arm config
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        // Extension config
        extension = hardwareMap.get(DcMotorEx.class, "extension");
        extension.setDirection(DcMotorSimple.Direction.REVERSE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, extpidcoeffs);
        extension.setPositionPIDFCoefficients(Constants.EXT_COEFF);

        // Claw config
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");
        leftClaw.setDirection(Servo.Direction.REVERSE);

        // Carousel
        carousel = hardwareMap.get(DcMotorEx.class, "carousel");

        distance1 = hardwareMap.get(Rev2mDistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(Rev2mDistanceSensor.class, "distance2");

        // Get arm and extension ready
        arm.setTargetPosition(0);
        arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm.setPower(Constants.ARM_MOVE);

        extension.setTargetPosition(0);
        extension.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extension.setPower(Constants.EXT_MOVE);


        // Start pose
        Pose2d start = new Pose2d(0,0,Math.toRadians(0));

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        while(opModeIsActive())
        {
            drive.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y/2,0,-gamepad1.right_stick_x));
            //arm
            if(gamepad1.a)
            {
                arm.setTargetPosition(arm.getCurrentPosition() - 40);
            }
            else if(gamepad1.x)
            {
                arm.setTargetPosition(arm.getCurrentPosition() + 40);
            }
            else if(gamepad1.y)
            {
                extension.setTargetPosition(extension.getCurrentPosition() + 5);
            }
            else if(gamepad1.b)
            {
                extension.setTargetPosition(extension.getCurrentPosition() - 5);
            }
            else if(gamepad1.left_bumper)
            {
                leftClaw.setPosition(0.2);
                rightClaw.setPosition(0.2);
            }
            else if(gamepad1.right_bumper)
            {
                leftClaw.setPosition(0.85);
                rightClaw.setPosition(0.85);
            }
            else if(gamepad1.dpad_up)
            {
                arm.setTargetPosition(390);
                extension.setTargetPosition(50);
            }
            else if(gamepad1.dpad_right)
            {
                arm.setTargetPosition(170);
                extension.setTargetPosition(50);
            }
            else if(gamepad1.dpad_down)
            {
                arm.setTargetPosition(15);
                extension.setTargetPosition(50);
            }
            else if(gamepad1.dpad_left)
            {
                arm.setTargetPosition(-120);
                extension.setTargetPosition(25);
            }

            if(gamepad1.right_trigger > 0)
            {
                carousel.setPower(1);
            }
            else if(gamepad1.left_trigger > 0)
            {
                carousel.setPower(-1);
            }
            else
            {
                carousel.setPower(0);
            }

            telemetry.addData("arm postition",arm.getCurrentPosition());
            telemetry.addData("extension position",extension.getCurrentPosition());
            telemetry.addData("left claw position",leftClaw.getPosition());
            telemetry.addData("rightClaw position",rightClaw.getPosition());
            telemetry.update();
        }
        if (isStopRequested())  return;
        PoseStorage.currentPose = drive.getPoseEstimate();

    }
    public double getAvgDis(Rev2mDistanceSensor distanceSensor){
        double sumOfReads = 0;
        double[] disReads = new double[5];
        for (int i = 0; i < 5; i++){
            disReads[i] = distanceSensor.getDistance(DistanceUnit.INCH);
            sumOfReads += disReads[i];
        }
        return sumOfReads / disReads.length;
    }
    public void moveArm(int armTarget){
        arm.setTargetPosition(armTarget);
        arm.setPower(Constants.ARM_MOVE);
    }
}
