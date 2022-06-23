package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Constants.GRIP;
import static org.firstinspires.ftc.teamcode.Constants.HIGHEST_POS;
import static org.firstinspires.ftc.teamcode.Constants.MIDDLE_POS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "trajectory_seq_1",group = "Linear Opmode")
public class TrajectorySequence1 extends LinearOpMode {

    public ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx arm,extension;

    public static PIDFCoefficients extpidcoffs = new PIDFCoefficients(10,0,0,0);

    private Servo leftClaw,rightClaw;

    // bot path parameters
    public double x = 7;
    public double y = 8.66;
    public double t = 4;//50-46.82;
    double backDist = Math.sqrt(Math.pow((-57.5+34.5-y),2) + Math.pow(57.5-23,2));
    double temp = Math.atan((double) Math.abs(-57.5+34.5-y)/(double) Math.abs(57.5-23));
    double backAngle = 90 - Math.toDegrees(temp);
    double error = 10;
    double lastTurn = -(90-backAngle) + 10;
    boolean toMove = false;

    DcMotorEx carousel;

    Rev2mDistanceSensor distance1;
    Rev2mDistanceSensor distance2;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("status","Initialized");
        telemetry.update();

        telemetry = dashboard.getTelemetry();
        telemetry.addData("status","Initialized");
        telemetry.update();

        SampleTankDrive tankDrive = new SampleTankDrive(hardwareMap);

        arm = hardwareMap.get(DcMotorEx.class,"arm");
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);

        extension = hardwareMap.get(DcMotorEx.class,"extension");
        extension.setDirection(DcMotorSimple.Direction.REVERSE);
        extension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extension.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, extpidcoffs);
        extension.setPositionPIDFCoefficients(Constants.EXT_COEFF);

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

        Pose2d start = new Pose2d(-34.5+x,57.5+y,Math.toRadians(-90));
        tankDrive.setPoseEstimate(start);
        // Trajectories
        TrajectorySequence move = tankDrive.trajectorySequenceBuilder(start)
                //arm intial position sequence
                .addTemporalMarker(()->{
                    arm.setTargetPosition(40);
                })
                .addTemporalMarker(()->{
                    leftClaw.setPosition(GRIP);
                    rightClaw.setPosition(GRIP);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    arm.setTargetPosition(200); //210
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    extension.setTargetPosition(50);
                }).waitSeconds(1)

                //detecting distance for arm position
                .addTemporalMarker(()->{
                    if (getAvgDis(distance1) < 10){
                        moveArm(15);
                    }
                    else if (getAvgDis(distance2) < 10){
                        moveArm(170);
                    }
                    else {
                        toMove = true;
                        moveArm(390);
                    }
                }).waitSeconds(1)

                //moving forward toward the carousel and turning right
//                .turn(Math.toRadians(2)) //extra
                .forward(x+57.5-23)
                .turn(Math.toRadians(90))
                //dropping the claw cube
                .addTemporalMarker(()->{
                    leftClaw.setPosition(Constants.DROP);
                    rightClaw.setPosition(Constants.DROP);
                }).waitSeconds(0.5)


                .addTemporalMarker(()->{
                    leftClaw.setPosition(0.2);
                    rightClaw.setPosition(0.2);
                }).waitSeconds(1)

                //turning back to intital angle and moving to left most position
                .turn(Math.toRadians(-backAngle-error))
                .back(backDist + t)

                //rotating carousel for 3 seconds
                .addTemporalMarker(()->{
                    carousel.setPower(1);
                }).waitSeconds(3)
                .addTemporalMarker(()->{
                    carousel.setPower(0);
                }).waitSeconds(0.5)

                //getting back to mid position and face towards the center and go there
                .forward(t)
                .turn(Math.toRadians(lastTurn))
                .forward(19)

                //moving arm to initial position
                .addTemporalMarker(()->{
                    moveArm(270);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    extension.setTargetPosition(0);
                    leftClaw.setPosition(0.85);
                    rightClaw.setPosition(0.85);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    moveArm(0);
                })
                .build();

        telemetry.addData("arm position",arm.getCurrentPosition());
        telemetry.addData("extension position",extension.getCurrentPosition());
        telemetry.addData("left claw",leftClaw.getPosition());
        telemetry.addData("right claw",rightClaw.getPosition());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested())  return;

        tankDrive.followTrajectorySequence(move);
        PoseStorage.currentPose = tankDrive.getPoseEstimate();

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
