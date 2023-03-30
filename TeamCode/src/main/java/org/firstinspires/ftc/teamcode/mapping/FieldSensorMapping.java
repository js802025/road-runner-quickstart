package org.firstinspires.ftc.teamcode.mapping;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.arcrobotics.ftclib.controller.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;


@Config
public class FieldSensorMapping {

    SampleMecanumDrive drive;
    HardwareMap hardwareMap;
    OpenCvWebcam webcam;
    Telemetry telemetry;
    DetectionPipeline pipeline;
    public boolean isBusy;
    public static double speed = .25;
    public static double p = 0.002;
    public static double i = 0;
    public static double d = 0;
    public static double tolerance = 25;
    Drivetrain drivetrain;

    public static PIDController controller = new PIDController(p, i, d);

    public FieldSensorMapping(LinearOpMode opMode, SampleMecanumDrive drive, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = drive;
        drivetrain =  new Drivetrain(opMode, hardwareMap, telemetry, drive);
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        controller.setTolerance(tolerance);
        initCV();
    }

    public void turnToColor() {
//        webcam.setPipeline(new DetectionPipeline(telemetry));
        isBusy = true;


    }

    public void update() {
        if (isBusy) {
            double dist = pipeline.getDistance();
            isBusy = !pipeline.isFacing();
            if (!isBusy) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                return;
            }
            drivetrain.JoystickMovement(0, 0, controller.calculate(dist, 0), 0, false, false, false, false);
        }
    }

    public boolean isBusy(){ return isBusy;}

    private void initCV() {
        // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Combines the above to create a webcam that we will use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Sets our pipeline to view images through as the one we want
        this.pipeline = new DetectionPipeline(telemetry);
        webcam.setPipeline(pipeline);

        // Turns on the webcam
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(960, 720, OpenCvCameraRotation.UPRIGHT);
            }
            //320 240

            //This is needed so it knows what to do if something goes wrong
            public void onError(int thing) {
                telemetry.addData("error", thing);
            }

        });
    }



    }
