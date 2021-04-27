package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.vision;

public class Vuforia {

//    private static final String VUFORIA_KEY =
//            "AXmG74D/////AAABmcbI+OuRcECkskgZxnebQ4aCvV3y4ZxNNMqL/ii0UTZ4LvJzrZXQ/Tmpjus37PyY6Qgy2esiZm1gCbpD08BG3bplvN1aDfRWlrXuhnwbsXfRT1WoJlg41K1j3jEY3+JMn3nQ0dFslzFDomXDRe9PUpuEPyZpR2uCkmWT26JOIfImG0kkdgTmYnxiuVCwE5k4qfYGZq0qxx6q5OowqkB/WLcMB9lGD5b88oGOMDXoil0JI4pZcVam4fdERnd490N9pX7mzdXYfDPntu+uYKZu9kNHbU6rqrnJJfzX3C0WXa1qF2e3zJCyR5ckciG/I4fSZgISyPwmMPO3+ss0NcboYEnPQvsG8Onwu30/Qq42B5/O";
//
//
//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
//    private static VuforiaLocalizer vuforia;
//
//    /**
//     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
//     * Detection engine.
//     */
//    private static TFObjectDetector tfod;
//
//    /**
//     *
//     */
//    private static void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
//    }
//
//    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
//    private static final String LABEL_FIRST_ELEMENT = "Quad";
//    private static final String LABEL_SECOND_ELEMENT = "Single";
//
//    /**
//     * Initialize the TensorFlow Object Detection engine.
//     */
//    private static void initTfod() {
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.8f;
//        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//    }
//
//    /**
//     * this initialize every motor, servo, and sensor
//     */
//
//    public static void initVision() {
//        initVuforia();
//        initTfod();
//        if (tfod != null) {
//            tfod.activate();
//        }
//    }
//
//
//    public static String getObjectAmount() {
//        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//        String a = "";
//        for (int i = 0; i < 100; i++) {
//            for (Recognition recognition : updatedRecognitions) {
//                a =
//                        recognition.getLabel();
//                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
////                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
////                        recognition.getLeft(), recognition.getTop());
////                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
////                        recognition.getRight(), recognition.getBottom());
//            }
//            telemetry.update();
//        }
////        telemetry.addData("# Object Detected", updatedRecognitions.size());
//        telemetry.update();
//        return a;
//    }
}
