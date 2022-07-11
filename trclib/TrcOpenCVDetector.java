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
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;

/**
 * This class implements a generic OpenCV detector. Typically, it is extended by a specific detector that provides
 * the algorithm to process an image for detecting objects using OpenCV APIs.
 */
public abstract class TrcOpenCVDetector implements TrcVisionProcessor<Mat, TrcOpenCVDetector.DetectedObject>
{
    protected static final String moduleName = "TrcOpenCVDetector";
    protected static final boolean debugEnabled = false;
    protected static final boolean tracingEnabled = false;
    protected static final boolean useGlobalTracer = false;
    protected static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    protected static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    protected TrcDbgTrace dbgTrace = null;

    /**
     * This class encapsulates info of the detected object. It extends TrcVisionTargetInfo.ObjectInfo that requires
     * it to provide a method to return the detected object rect.
     */
    public static class DetectedObject extends TrcVisionTargetInfo.ObjectInfo
    {
        public final Rect rect;
        public final float angle;
        public final float response;
        public final int octave;
        public final int classId;

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param rect specifies the rect of the object.
         * @param angle specifies specifies computed orientation of the keypoint (-1 if not applicable).
         * @param response specifies the response, by which the strongest keypoints have been selected. Can be used
         *        for further sorting or subsampling.
         * @param octave specifies octave (pyramid layer), from which the keypoint has been extracted.
         * @param classId specifies object ID, that can be used to cluster keypoints by an object they belong to.
         */
        public DetectedObject(Rect rect, float angle, float response, int octave, int classId)
        {
            this.rect = rect;
            this.angle = angle;
            this.response = response;
            this.octave = octave;
            this.classId = classId;
        }   //DetectedObject

        /**
         * Constructor: Creates an instance of the object.
         *
         * @param rect specifies the rect of the object.
         */
        public DetectedObject(Rect rect)
        {
            this(rect, -1.0f, 0.0f, 0, 0);
        }   //DetectedObject

        /**
         * This method returns the rect of the detected object.
         *
         * @return rect of the detected object.
         */
        @Override
        public Rect getRect()
        {
            return rect;
        }   //getRect

        /**
         * This method returns the string form of the target info.
         *
         * @return string form of the target info.
         */
        @Override
        public String toString()
        {
            return "Rect=" + rect.toString();
        }   //toString

    }   //class DetectedObject

    /**
     * This interface provides a method for filtering false positive objects in the detected target list.
     */
    public interface FilterTarget
    {
        boolean validateTarget(DetectedObject object);
    }   //interface FilterTarget

    private final String instanceName;
    private final int imageWidth, imageHeight;
    private final TrcDbgTrace tracer;
    private final TrcHomographyMapper homographyMapper;
    private final TrcVisionTask<Mat, DetectedObject> visionTask;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param numImageBuffers specifies the number of image buffers to allocate.
     * @param imageWidth specifies the width of the camera image.
     * @param imageHeight specifies the height of the camera image.
     * @param cameraRect specifies the camera rectangle for Homography Mapper, can be null if not provided.
     * @param worldRect specifies the world rectangle for Homography Mapper, can be null if not provided.
     * @param tracer specifies the tracer for trace info, null if none provided.
     */
    public TrcOpenCVDetector(
        String instanceName, int numImageBuffers, int imageWidth, int imageHeight,
        TrcHomographyMapper.Rectangle cameraRect, TrcHomographyMapper.Rectangle worldRect,
        TrcDbgTrace tracer)
    {
        if (debugEnabled)
        {
            dbgTrace = useGlobalTracer?
                TrcDbgTrace.getGlobalTracer():
                new TrcDbgTrace(moduleName + "." + instanceName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.imageWidth = imageWidth;
        this.imageHeight = imageHeight;
        this.tracer = tracer;

        if (cameraRect != null && worldRect != null)
        {
            homographyMapper = new TrcHomographyMapper(cameraRect, worldRect);
        }
        else
        {
            homographyMapper = null;
        }
        //
        // Pre-allocate the image buffers.
        //
        Mat[] imageBuffers = new Mat[numImageBuffers];
        for (int i = 0; i < imageBuffers.length; i++)
        {
            imageBuffers[i] = new Mat();
        }

        visionTask = new TrcVisionTask<>(instanceName, this, imageBuffers);
    }   //TrcOpenCVDetector

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
     * This method returns the state of the vision task.
     *
     * @return true if the vision task is enabled, false if disabled.
     */
    public boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";
        boolean enabled = visionTask != null && visionTask.isTaskEnabled();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", Boolean.toString(enabled));
        }

        return enabled;
    }   //isTaskEnabled

    /**
     * This method enables/disables the vision processing task.
     *
     * @param enabled specifies true to enable vision task, false to disable.
     */
    public void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", Boolean.toString(enabled));
        }

        if (visionTask != null)
        {
            visionTask.setTaskEnabled(enabled);
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method enables/disables image annotation of the detected object rects.
     *
     * @param enabled specifies true to enable video out, false to disable.
     */
    public void setVideoOutEnabled(boolean enabled)
    {
        final String funcName = "setVideoOutEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        visionTask.setVideoOutEnabled(enabled);
    }   //setVideoOutEnabled

    /**
     * This method returns an array of detected targets from Grip vision.
     *
     * @param filter specifies the filter to call to filter out false positive targets.
     * @param comparator specifies the comparator to sort the array if provided, can be null if not provided.
     * @return array of detected target info.
     */
    @SuppressWarnings("unchecked")
    public synchronized TrcVisionTargetInfo<DetectedObject>[] getDetectedTargetsInfo(
        FilterTarget filter, Comparator<? super TrcVisionTargetInfo<DetectedObject>> comparator)
    {
        final String funcName = "getDetectedTargetsInfo";
        TrcVisionTargetInfo<DetectedObject>[] targets = null;
        DetectedObject[] detectedObjs = visionTask.getDetectedObjects();

        if (detectedObjs != null)
        {
            ArrayList<TrcVisionTargetInfo<DetectedObject>> targetList = new ArrayList<>();

            for (DetectedObject detectedObj : detectedObjs)
            {
                if (filter == null || filter.validateTarget(detectedObj))
                {
                    TrcVisionTargetInfo<DetectedObject> targetInfo =
                        new TrcVisionTargetInfo<>(detectedObj, imageWidth, imageHeight, homographyMapper);
                    targetList.add(targetInfo);
                }
            }

            if (targetList.size() > 0)
            {
                targets = targetList.toArray(new TrcVisionTargetInfo[0]);
                if (comparator != null && targets.length > 1)
                {
                    Arrays.sort(targets, comparator);
                }
            }

            if (targets != null && tracer != null)
            {
                for (int i = 0; i < targets.length; i++)
                {
                    tracer.traceInfo(funcName, "[%d] Target=%s", i, targets[i]);
                }
            }
        }

        return targets;
    }   //getDetectedTargetsInfo

    /**
     * This method is called to overlay rectangles of the detected objects on an image.
     *
     * @param image specifies the frame to be rendered to the video output.
     * @param detectedObjects specifies the detected objects.
     * @param color specifies the color of the rectangle outline.
     * @param thickness specifies the thickness of the rectangle outline.
     */
    public void drawRectangles(Mat image, DetectedObject[] detectedObjects, Scalar color, int thickness)
    {
        //
        // Overlay a rectangle on each detected object.
        //
        if (detectedObjects != null)
        {
            for (DetectedObject detectedObject : detectedObjects)
            {
                Rect rect = detectedObject.getRect();
                //
                // Draw a rectangle around the detected object.
                //
                Imgproc.rectangle(
                    image, new Point(rect.x, rect.y), new Point(rect.x + rect.width, rect.y + rect.height),
                    color, thickness);
            }
        }
    }   //drawRectangles

}   //class TrcOpenCVDetector
