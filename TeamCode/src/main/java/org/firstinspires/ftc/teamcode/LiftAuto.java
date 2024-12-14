package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.opencv.core.Mat;

@Autonomous(name = "LiftAuto")
@Config
public class LiftAuto extends LinearOpMode {

    public static int DRIVE1 = -13;
    public static int DRIVE2 = 3;
    public static int STRAFE1 = -5;
    public static int ARM1 = 50;
    public static int ARM2 = -55;
    public static int LIFT1 = 1610;
    public static int LIFT2 = -1610;
    public static double MIN_DUMP = 0;
    public static double MAX_DUMP = 1.0;

    public Autodrive driver;
    DcMotor arm;
    DcMotor lift;
    Servo dump;
    Servo clamp;
    IMU imu;


    @Override
    public void runOpMode() {
        runauto();
    }

    public void runauto() {

        waitForStart();

        driver = new Autodrive(hardwareMap, this::opModeIsActive);
        DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "backleft");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "backright");
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");
        Servo clamp = hardwareMap.get(Servo.class, "claw");
        Servo dump = hardwareMap.get(Servo.class, "dump");
        IMU imu = hardwareMap.get(IMU.class,"imu");

        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        );

        dump.setPosition(MAX_DUMP);
        driver.arm(ARM1);
        driver.drive(DRIVE1,false);
        driver.lift(LIFT1);
        dump.setPosition(MIN_DUMP);
        sleep(1000);
        dump.setPosition(MAX_DUMP);
        sleep(1000);
        driver.drive(DRIVE2,false);
        driver.lift(LIFT2);
        driver.arm(ARM2);

        TelemetryPacket stats = new TelemetryPacket();
        stats.put("Dump", dump.getPosition());
        FtcDashboard.getInstance().sendTelemetryPacket(stats);

        }
    }


