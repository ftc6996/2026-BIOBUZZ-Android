package org.firstinspires.ftc.teamcode;

//https://github.com/ftc6996/2025-Decode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Robot.java Completed transfer
//https://github.com/ftc6996/2025-Decode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/MecanumDrive.java
//https://github.com/ftc6996/2025-Decode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/Launcher.java Completed up to line 178
//https://github.com/ftc6996/2025-Decode/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/mechanisms/LimeLight.java Completed transfer

import static org.firstinspires.ftc.teamcode.Pedro.Constants.*;
//import static org.firstinspires.ftc.teamcode.Constants.Game.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Pedro.Constants;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class Robot{
    /*****************************************************************
     *                                                                *
     *                         PRIVATE SECTION                        *
     *                                                                *
     *****************************************************************/
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor intake_motor;
    private DcMotor left_front_drive, left_rear_drive, right_front_drive, right_rear_drive;
    private CRServo windmillServo = null;
    private DcMotorEx launcher = null;

    public Robot() {
        initDrive();
        initIMU();
        initPinPoint();
    }

    private void initDrive()
    {
        left_front_drive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        left_rear_drive = hardwareMap.get(DcMotor.class, "left_rear_drive");
        right_front_drive  = hardwareMap.get(DcMotor.class, "right_front_drive");
        right_rear_drive = hardwareMap.get(DcMotor.class, "right_rear_drive");

        left_front_drive.setDirection(DcMotor.Direction.REVERSE);
        left_rear_drive.setDirection(com.qualcomm.robotcore.hardware.DcMotor.Direction.REVERSE);
        right_front_drive.setDirection(DcMotor.Direction.FORWARD);
        right_rear_drive.setDirection(DcMotor.Direction.FORWARD);

        left_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rear_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        left_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void initIMU()
    {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(orientation);
        imu.initialize(parameters);
    }
    private void initPinPoint()
    {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setPosition(new Pose2D(DistanceUnit.MM,0,0, AngleUnit.DEGREES,0));
        pinpoint.setOffsets(90,0,DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        isPinpointInitialized = true;
    }
    private GoBildaPinpointDriver pinpoint;
    private IMU imu;
    private RevHubOrientationOnRobot orientation;
    private HardwareMap hardwareMap;
    private boolean isPinpointInitialized = false;
    private boolean shotRequested = false;
    private LLResult result;
    private int tagID;
    private double tagLocationX, tagLocationY, tagDistance;
    private double tagLocationArea, tagLocationWidth, tagLocationHeight;
    private double angleToGoal; //in radians
    private double speedAdjustment = 1;
    public void setPower(double left_front, double right_front, double left_back, double right_back)
    {
        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double currentMaxSpeed = speedAdjustment;
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(left_front));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(right_front));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(left_back));
        currentMaxSpeed = Math.max(currentMaxSpeed, Math.abs(right_back));

        // Because we are adding vectors and motors only take values between
        // [-1,1] we may need to normalize them.

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        left_front /= currentMaxSpeed;
        right_front /= currentMaxSpeed;
        left_back /= currentMaxSpeed;
        right_back /= currentMaxSpeed;

        //flat adjustment, easy way to lower power without changing lots of code
        left_front *= speedAdjustment;
        right_front *= speedAdjustment;
        left_back *= speedAdjustment;
        right_back *= speedAdjustment;

        left_front_drive.setPower(left_front);
        left_rear_drive.setPower(left_back);
        right_front_drive.setPower(right_front);
        right_rear_drive.setPower(right_back);
    }
    /*****************************************************************
     *                                                                *
     *                         PUBLIC SECTION                         *
     *                                                                *
     *****************************************************************/
    public Limelight3A vision;
    public int target_velocity;
    public int target_tag = -1;
    public boolean isTargetFound = false;
    public boolean isAccurite = false;
    public boolean rapidFire = false;
    public Pose2D aprilTagPose =  new Pose2D(DistanceUnit.INCH, -66, 66, AngleUnit.DEGREES, 0);
    public double dX, dY;
    public int numShotsRequested = 0;
    public int numShotsFufiled = 0;
    public Pose3D tagPose = new Pose3D(
            new Position(DistanceUnit.INCH,0,0,0,0),
            new YawPitchRollAngles(AngleUnit.DEGREES,0,0,0,0));
    public double getTagDistance()
    {
        return tagDistance;
    }
    /*public LimeLight(Limelight3A vision)
    {
        this.vision = vision;

        setPipeline(0);
        vision.start();
        clearFoundAprilTag();
    }
    public LimeLight(Limelight3A vision, int pipeline)
    {
        this.vision = vision;

        setPipeline(pipeline);
        vision.start();
        clearFoundAprilTag();
    }*/
    public void setPipeline(int pipeline) {vision.pipelineSwitch(pipeline);}
    public int getPipeline()
    {
        return result.getPipelineIndex();
    }
    public void clearFoundAprilTag()
    {
        tagID = 0;
        tagLocationX = 0;
        tagLocationY = 0;
        tagLocationArea = 0;
        tagLocationWidth = 0;
        tagLocationHeight = 0;
        tagDistance = 0;
    }
    public void getAprilTags()
    {
        result = vision.getLatestResult();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            tagID = fiducial.getFiducialId();
            tagLocationX = fiducial.getTargetXDegrees();
            tagLocationY = fiducial.getTargetYDegrees();
            tagLocationWidth = fiducial.getTargetXPixels();
            tagLocationHeight = fiducial.getTargetYPixels();
            tagLocationArea = tagLocationWidth * tagLocationHeight;
            angleToGoal = Math.toRadians(0 + tagLocationY);
            tagPose = fiducial.getRobotPoseTargetSpace();
            tagDistance = tagPose.getPosition().toUnit(DistanceUnit.INCH).z;
        }

    }
    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        STOPPED
    }
    public LaunchState launchState;
    public enum TurningState {
        IDLE,
        START_LEFT,
        START_RIGHT,
        READING,
        STOP
    }
    public TurningState turningState;
    public void init(HardwareMap hardwareMap) {

        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
        this.hardwareMap = hardwareMap;
        initDrive();
        initIMU();
        initPinPoint();

        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchState = LaunchState.IDLE;
        intake_motor = hardwareMap.get(DcMotor.class, "intake_motor");
        intake_motor.setDirection(DcMotor.Direction.REVERSE);
        intake_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        windmillServo = hardwareMap.get(CRServo.class, "windmillServo");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setDirection(DcMotorSimple.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(94, 0,0, 12);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        vision = hardwareMap.get(Limelight3A.class, "limelight");
    }
    public void updatePinPoint(){
        if (!isPinpointInitialized)
            return;
        pinpoint.update();
    }
    public void move(double drive, double strafe, double twist, double time) {
        double[] speeds = {
                (drive + strafe + twist),
                (drive - strafe - twist),
                (drive - strafe + twist),
                (drive + strafe - twist)
        };
        // apply the calculated values to the motors.
        setPower(speeds[0], speeds[1], speeds[2], speeds[3]);
        runtime.reset();
        if (runtime.seconds() >= time){
            setPower(0, 0, 0, 0);
        }
    }
    public void stop() {setPower(0,0,0,0);}
    public void intake(double power) {
        intake_motor.setPower(power);
    }
    public void setFlyWheelVelocity(double velocity) {
        launcher.setVelocity(velocity);
    }
    public void shoot(boolean shotRequested, int velocity, int numShots) {
        numShotsRequested = numShots;
        target_velocity = Math.abs(velocity);
    }
    public Pose2D getPinpointPosition(){
        if (!isPinpointInitialized)
            return new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 0);
        return pinpoint.getPosition();
    }
    public void getAprilTag()
    {
        getAprilTags();
    }
    public int getTagID() {return tagID;}
    public double getTagLocationX()
    {
        return tagLocationX;
    }
    public double getTagLocationY()
    {
        return tagLocationY;
    }
    public void processTelemetry(Telemetry telemetry) {
        telemetry.addData("Launch State: ", launchState);

        telemetry.addLine("-----------------------------------");
        telemetry.addData("Pipeline",getPipeline());

        telemetry.addData("Target Found",isTargetFound);
        if (isTargetFound) {
            telemetry.addData("ID seen", getTagID());
            telemetry.addData("Distance", getTagDistance());
            telemetry.addData("Pose", tagPose.getPosition().toUnit(DistanceUnit.INCH).toString());
        }
    }
}