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

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/**
 * This interface implements the standard methods for an OpenCV pipeline. It also provides default methods to log
 * pipeline performance metrics.
 *
 * @param <O> specifies the detected object type the pipeline will produce.
 */
public interface TrcOpenCvPipeline<O>
{
    /**
     * This class encapsulates the pipeline performance metrics.
     */
    class PerformanceMetrics
    {
        double startTime = 0.0;
        double totalProcessedTime = 0.0;
        long totalProcessedFrames = 0;

        /**
         * This method resets the pipeline performance metrics. It is typically called before enabling the pipeline.
         */
        public void reset()
        {
            startTime = TrcTimer.getCurrentTime();
            totalProcessedTime = 0.0;
            totalProcessedFrames = 0;
        }   //reset

        /**
         * This method is called to log the processing time of the pipeline.
         *
         * @param startTime specifies the timestamp when the processing starts.
         */
        public void logProcessingTime(double startTime)
        {
            totalProcessedTime += TrcTimer.getCurrentTime() - startTime;
            totalProcessedFrames++;
        }   //logProcessingTime

        /**
         * This method prints the pipeline performance metrics using the given tracer.
         */
        public void printMetrics(TrcDbgTrace tracer)
        {
            final String funcName = "printMetrics";

            if (tracer != null)
            {
                tracer.traceInfo(
                    funcName, "AvgProcessTime=%.3f sec, FrameRate=%.1f",
                    totalProcessedTime/totalProcessedFrames,
                    totalProcessedFrames/(TrcTimer.getCurrentTime() - startTime));
            }
        }   //printMetrics

    }   //class PerformanceMetrics

    PerformanceMetrics performanceMetrics = new PerformanceMetrics();

    /**
     * This method is called to reset the state of the pipeline if any.
     */
    void reset();

    /**
     * This method is called to process the input image through the pipeline.
     *
     * @param input specifies the input image to be processed.
     * @return array of detected objects.
     */
    O[] process(Mat input);

    /**
     * This method returns the array of detected objects.
     *
     * @return array of detected objects.
     */
    O[] getDetectedObjects();

    /**
     * This method sets the intermediate mat of the pipeline as the video output mat and optionally annotate the
     * detected rectangle on it.
     *
     * @param intermediateStep specifies the intermediate mat used as video output (0 is the original input frame).
     * @param annotate specifies true to annotate detected rectangles on the output mat, false otherwise.
     */
    void setVideoOutput(int intermediateStep, boolean annotate);

    /**
     * This method cycles to the next intermediate mat of the pipeline as the video output mat.
     *
     * @param annotate specifies true to annotate detected rectangles on the output mat, false otherwise.
     */
    void setNextVideoOutput(boolean annotate);

    /**
     * This method returns an intermediate processed frame. Typically, a pipeline processes a frame in a number of
     * steps. It may be useful to see an intermediate frame for a step in the pipeline for tuning or debugging
     * purposes.
     *
     * @param step specifies the intermediate step (0 is the original input frame).
     * @return processed frame of the specified step.
     */
    Mat getIntermediateOutput(int step);

    /**
     * This method returns the selected intermediate output Mat.
     *
     * @return selected output mat.
     */
    Mat getSelectedOutput();

    /**
     * This method is called to overlay rectangles of the detected objects on an image.
     *
     * @param image specifies the frame to be rendered to the video output.
     * @param detectedObjects specifies the detected objects.
     * @param color specifies the color of the annotated rectangle.
     * @param thickness specifies the thickness of the annotated rectangle.
     */
    default void annotateFrame(
        Mat image, TrcOpenCvDetector.DetectedObject<?>[] detectedObjects, Scalar color, int thickness)
    {
        for (TrcOpenCvDetector.DetectedObject<?> object : detectedObjects)
        {
            Imgproc.rectangle(image, object.getRect(), color, thickness);
        }
    }   //annotatedFrame

}   //interface TrcOpenCvPipeline
