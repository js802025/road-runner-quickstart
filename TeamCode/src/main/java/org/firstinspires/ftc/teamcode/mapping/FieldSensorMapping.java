package org.firstinspires.ftc.teamcode.mapping;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.pipelines.DetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class FieldSensorMapping {

    SampleMecanumDrive drive;
    HardwareMap hardwareMap;
    OpenCvWebcam webcam;
    Telemetry telemetry;
    DetectionPipeline pipeline;
    Boolean isBusy;
    public FieldSensorMapping(SampleMecanumDrive drive, HardwareMap hardwareMap, Telemetry telemetry) {
        this.drive = drive;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        initCV();
    }

    public void turnToColor() {
        webcam.setPipeline(new DetectionPipeline(telemetry));
        isBusy = true;


    }

    public void update() {
        if (isBusy) {
            double mult = pipeline.getMultiplier();
            isBusy = !pipeline.isFacing();
            if (!isBusy) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
                return;
            }
            drive.setWeightedDrivePower(new Pose2d(0, 0, 1 * mult));
        }
    }

    private void initCV() {
        // Sets variable for the camera id
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // Gives a name to the webcam
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Combines the above to create a webcam that we will use
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Sets our pipeline to view images through as the one we want
        DetectionPipeline pipeline = new DetectionPipeline(telemetry);
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
