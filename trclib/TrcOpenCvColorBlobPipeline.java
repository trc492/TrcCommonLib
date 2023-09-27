/*
 * Copyright (c) 2022 Titan Robotics Club (http://www.titanrobotics.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package TrcCommonLib.trclib;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;

/**
 * This class implements a generic OpenCV color blob detection pipeline.
 */
public class TrcOpenCvColorBlobPipeline implements TrcOpenCvPipeline<TrcOpenCvDetector.DetectedObject<?>>
{
    /**
     * This class encapsulates info of the detected object. It extends TrcOpenCvDetector.DetectedObject that requires
     * it to provide a method to return the detected object rect and area.
     */
    public static class DetectedObject extends TrcOpenCvDetector.DetectedObject<MatOfPoint>
    {
        /**
         * Constructor: Creates an instance of the object.
         *
         * @param contour specifies the contour of the detected object.
         */
        public DetectedObject(MatOfPoint contour)
        {
            super(contour);
        }   //DetectedObject

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            // Get detected object bounding box.
            return Imgproc.boundingRect(object);
        }   //getRect

        /**
         * This method returns the area of the detected object.
         *
         * @return area of the detected object.
         */
        @Override
        public double getArea()
        {
            // OpenCv returns the actual area of the object, not just the bounding box.
            return Imgproc.contourArea(object);
        }   //getArea

        /**
         * This method returns the pose of the detected object relative to the camera.
         *
         * @return pose of the detected object relative to camera.
         */
        @Override
        public TrcPose3D getObjectPose()
        {
            // ColorBlob detection does not provide detected object pose, let caller use homography to calculate it.
            return null;
        }   //getObjectPose

        /**
         * This method returns the real world width of the detected object.
         *
         * @return real world width of the detected object.
         */
        @Override
        public Double getObjectWidth()
        {
            // ColorBlob detection does not provide detected object width, let caller use homography to calculate it.
            return null;
        }   //getObjectWidth

        /**
         * This method returns the real world depth of the detected object.
         *
         * @return real world depth of the detected object.
         */
        @Override
        public Double getObjectDepth()
        {
            // ColorBlob detection does not provide detected object depth, let caller use homography to calculate it.
            return null;
        }   //getObjectDepth

    }   //class DetectedObject

    /**
     * This class encapsulates all the filter contour parameters.
     */
    public static class FilterContourParams
    {
        double minArea = 0.0;
        double minPerimeter = 0.0;
        double[] widthRange = {0.0, 1000.0};
        double[] heightRange = {0.0, 1000.0};
        double[] solidityRange = {0.0, 100.0};
        double[] verticesRange = {0.0, 1000000.0};
        double[] aspectRatioRange = {0.0, 1000.0};

        public FilterContourParams setMinArea(double minArea)
        {
            this.minArea = minArea;
            return this;
        }   //setMinArea

        public FilterContourParams setMinPerimeter(double minPerimeter)
        {
            this.minPerimeter = minPerimeter;
            return this;
        }   //setMinPerimeter

        public FilterContourParams setWidthRange(double min, double max)
        {
            this.widthRange[0] = min;
            this.widthRange[1] = max;
            return this;
        }   //setWidthRange

        public FilterContourParams setHeightRange(double min, double max)
        {
            this.heightRange[0] = min;
            this.heightRange[1] = max;
            return this;
        }   //setHeightRange

        public FilterContourParams setSolidityRange(double min, double max)
        {
            this.solidityRange[0] = min;
            this.solidityRange[1] = max;
            return this;
        }   //setSolidityRange

        public FilterContourParams setVerticesRange(double min, double max)
        {
            this.verticesRange[0] = min;
            this.verticesRange[1] = max;
            return this;
        }   //setVerticesRange

        public FilterContourParams setAspectRatioRange(double min, double max)
        {
            this.aspectRatioRange[0] = min;
            this.aspectRatioRange[1] = max;
            return this;
        }   //setAspectRatioRange

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "minArea=%.1f, minPerimeter=%.1f, width=(%.1f,%.1f), height=(%.1f,%.1f), solidity=(%.1f,%.1f), " +
                "vertices=(%.0f,%.0f), aspectRatio=(%.1f,%.1f)",
                minArea, minPerimeter, widthRange[0], widthRange[1], heightRange[0], heightRange[1], solidityRange[0],
                solidityRange[1], verticesRange[0], verticesRange[1], aspectRatioRange[0], aspectRatioRange[1]);
        }   //toString

    }   //class FilterContourParams

    private static final Scalar ANNOTATE_RECT_COLOR = new Scalar(0, 255, 0, 255);
    private static final Scalar ANNOTATE_RECT_WHITE = new Scalar(255, 255, 255, 255);
    private static final int ANNOTATE_RECT_THICKNESS = 3;
    private static final double ANNOTATE_FONT_SCALE = 0.3;

    private final String instanceName;
    private final int colorConversion;
    private double[] colorThresholds;
    private final FilterContourParams filterContourParams;
    private final TrcDbgTrace tracer;
    private final Mat colorConversionOutput = new Mat();
    private final Mat colorThresholdOutput = new Mat();
    private final Mat hierarchy = new Mat();
    private final Mat[] intermediateMats;

    private final TrcVisionPerformanceMetrics performanceMetrics = new TrcVisionPerformanceMetrics();
    private final AtomicReference<DetectedObject[]> detectedObjectsUpdate = new AtomicReference<>();
    private int intermediateStep = 0;
    private boolean annotateEnabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param colorConversion specifies color space conversion (Imgproc.COLOR_*).
     * @param colorThresholds specifies an array of color thresholds. If useHsv is false, the array contains RGB
     *        thresholds (minRed, maxRed, minGreen, maxGreen, minBlue, maxBlue). If useHsv is true, the array contains
     *        HSV thresholds (minHue, maxHue, minSat, maxSat, minValue, maxValue).
     * @param filterContourParams specifies the parameters for filtering contours.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public TrcOpenCvColorBlobPipeline(
        String instanceName, int colorConversion, double[] colorThresholds, FilterContourParams filterContourParams,
        TrcDbgTrace tracer)
    {
        if (colorThresholds == null || colorThresholds.length != 6)
        {
            throw new RuntimeException("colorThresholds must be an array of 6 doubles.");
        }

        this.instanceName = instanceName;
        this.colorConversion = colorConversion;
        this.colorThresholds = colorThresholds;
        this.filterContourParams = filterContourParams;
        this.tracer = tracer;
        intermediateMats = new Mat[3];
        intermediateMats[0] = null;
        intermediateMats[1] = colorConversionOutput;
        intermediateMats[2] = colorThresholdOutput;
    }   //TrcOpenCvColorBlobPipeline

    /**
     * This method returns the instance name.
     *
     * @return instance name.
     */
    @Override
    public String toString()
    {
        return instanceName;
    }   //toString

    /**
     * This method returns the color threshold values.
     *
     * @return array of color threshold values.
     */
    public double[] getColorThresholds()
    {
        return colorThresholds;
    }   //getColorThresholds

    /**
     * This method sets the color threshold values.
     *
     * @param colorThresholds specifies an array of color threshold values.
     */
    public void setColorThresholds(double... colorThresholds)
    {
        this.colorThresholds = colorThresholds;
    }   //setColorThresholds

    //
    // Implements TrcOpenCvPipeline interface.
    //

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    @Override
    public void reset()
    {
        performanceMetrics.reset();
        intermediateStep = 0;
    }   //reset

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] process(Mat input)
    {
        DetectedObject[] detectedObjects = null;
        ArrayList<MatOfPoint> contoursOutput = new ArrayList<>();
        ArrayList<MatOfPoint> filterContoursOutput = new ArrayList<>();
        double startTime = TrcTimer.getCurrentTime();

        intermediateMats[0] = input;
        filterByColor(input, colorConversion, colorThresholds, colorThresholdOutput);
        findContours(colorThresholdOutput, true, contoursOutput);
        if (filterContourParams != null)
        {
            filterContours(contoursOutput, filterContourParams, filterContoursOutput);
            contoursOutput = filterContoursOutput;
        }
        performanceMetrics.logProcessingTime(startTime);
        performanceMetrics.printMetrics(tracer);

        if (contoursOutput.size() > 0)
        {
            detectedObjects = new DetectedObject[contoursOutput.size()];
            for (int i = 0; i < detectedObjects.length; i++)
            {
                detectedObjects[i] = new DetectedObject(contoursOutput.get(i));
            }

            if (annotateEnabled)
            {
                Mat output = getIntermediateOutput(intermediateStep);
                Scalar color = intermediateStep == 0? ANNOTATE_RECT_COLOR: ANNOTATE_RECT_WHITE;
                annotateFrame(
                    output, instanceName, detectedObjects, color, ANNOTATE_RECT_THICKNESS, ANNOTATE_FONT_SCALE);
//                // This line is for tuning Homography.
//                Imgproc.line(output, new Point(0, 120), new Point(639, 120), new Scalar(255, 255, 255), 2);
            }

            detectedObjectsUpdate.set(detectedObjects);
        }

        return detectedObjects;
    }   //process

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    @Override
    public DetectedObject[] getDetectedObjects()
    {
        return detectedObjectsUpdate.getAndSet(null);
    }   //getDetectedObjects

    /**
     * This method enables/disables image annotation of the detected object.
     *
     * @param enabled specifies true to enable annotation, false to disable.
     */
    @Override
    public void setAnnotateEnabled(boolean enabled)
    {
        annotateEnabled = enabled;
    }   //setAnnotateEnabled

    /**
     * This method checks if image annotation is enabled.
     *
     * @return true if annotation is enabled, false otherwise.
     */
    @Override
    public boolean isAnnotateEnabled()
    {
        return annotateEnabled;
    }   //isAnnotateEnabled

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (0 is the original input frame).
     */
    @Override
    public void setVideoOutput(int intermediateStep)
    {
        if (intermediateStep >= 0 && intermediateStep < intermediateMats.length)
        {
            this.intermediateStep = intermediateStep;
        }
    }   //setVideoOutput

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     */
    @Override
    public void setNextVideoOutput()
    {
        intermediateStep = (intermediateStep + 1) % intermediateMats.length;
    }   //setNextVideoOutput

    /**
     * This method returns an intermediate processed frame. Typically, a pipeline processes a frame in a number of
     * steps. It may be useful to see an intermediate frame for a step in the pipeline for tuning or debugging
     * purposes.
     *
     * @param step specifies the intermediate step (0 is the original input frame).
     * @return processed frame of the specified step.
     */
    @Override
    public Mat getIntermediateOutput(int step)
    {
        Mat mat = null;

        if (step >= 0 && step < intermediateMats.length)
        {
            mat = intermediateMats[step];
        }

        return mat;
    }   //getIntermediateOutput

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    @Override
    public Mat getSelectedOutput()
    {
        return getIntermediateOutput(intermediateStep);
    }   //getSelectedOutput

    /**
     * This method process the image by filtering with the specified color ranges.
     *
     * @param input specifies the input frame.
     * @param colorConversion specifies color space conversion (Imgproc.COLOR_*).
     * @param colorThresholds specifies the color ranges (redMin, redMax, greenMin, greenMax, blueMin, blueMax) or
     *        (hueMin, hueMax, satMin, satMax, valueMin, valueMax) if useHsv is true.
     * @param out specifies the output frame for the result.
     */
    private void filterByColor(Mat input, int colorConversion, double[] colorThresholds, Mat out)
    {
        Imgproc.cvtColor(input, colorConversionOutput, colorConversion);
        Core.inRange(
            colorConversionOutput, new Scalar(colorThresholds[0], colorThresholds[2], colorThresholds[4]),
            new Scalar(colorThresholds[1], colorThresholds[3], colorThresholds[5]), out);
    }   //filterByColor

    /**
     * Sets the values of pixels in a binary image to their distance to the nearest black pixel.
     *
     * @param input specifies the image from which to find object contours.
     * @param externalOnly specifies true to use EXTERNAL mode, false to use LIST mode.
     * @param contours specifies the list to hold the contours found.
     */
    private void findContours(Mat input, boolean externalOnly, List<MatOfPoint> contours)
    {
        contours.clear();
        Imgproc.findContours(
            input, contours, hierarchy, externalOnly? Imgproc.RETR_EXTERNAL: Imgproc.RETR_LIST,
            Imgproc.CHAIN_APPROX_SIMPLE);
    }   //findContours

    /**
     * This method filters out contours that do not meet certain criteria.
     *
     * @param inputContours specifies the input list of contours.
     * @param filterContourParams specifies the filter contour parameters.
     * @param output specifies the the output list of contours.
     */
    private void filterContours(
        List<MatOfPoint> inputContours, FilterContourParams filterContourParams, List<MatOfPoint> output)
    {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //
        // Perform the filtering.
        //
        for (int i = 0; i < inputContours.size(); i++)
        {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            // Check width.
            if (bb.width < filterContourParams.widthRange[0] || bb.width > filterContourParams.widthRange[1])
            {
                continue;
            }
            // Check height.
            if (bb.height < filterContourParams.heightRange[0] || bb.height > filterContourParams.heightRange[1])
            {
                continue;
            }
            // Check area.
            final double area = Imgproc.contourArea(contour);
            if (area < filterContourParams.minArea)
            {
                continue;
            }
            // Check perimeter.
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < filterContourParams.minPerimeter)
            {
                continue;
            }
            // Check solidity.
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++)
            {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < filterContourParams.solidityRange[0] || solid > filterContourParams.solidityRange[1])
            {
                continue;
            }
            // Check vertex count.
            if (contour.rows() < filterContourParams.verticesRange[0] ||
                contour.rows() > filterContourParams.verticesRange[1])
            {
                continue;
            }
            // Check aspect ratio.
            final double ratio = bb.width / (double)bb.height;
            if (ratio < filterContourParams.aspectRatioRange[0] || ratio > filterContourParams.aspectRatioRange[1])
            {
                continue;
            }

            output.add(contour);
        }
    }   //filterContours

}  //class TrcOpenCvColorBlobPipeline
