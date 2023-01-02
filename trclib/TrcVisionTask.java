/*
 * Copyright (c) 2017 Titan Robotics Club (http://www.titanrobotics.com)
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

/**
 * This class implements a platform independent vision task. When enabled, it grabs a frame from the video source,
 * calls the provided vision processor to process the frame and overlays rectangles on the detected objects in the
 * image. This class is to be extended by a platform dependent vision processor.
 *
 * @param <I> specifies the type of the input image.
 * @param <O> specifies the type of the detected objects.
 */
public class TrcVisionTask<I, O>
{
    private static final String moduleName = "TrcVisionTask";
    private static final boolean debugEnabled = false;
    private static final boolean tracingEnabled = false;
    private static final TrcDbgTrace.TraceLevel traceLevel = TrcDbgTrace.TraceLevel.API;
    private static final TrcDbgTrace.MsgLevel msgLevel = TrcDbgTrace.MsgLevel.INFO;
    private TrcDbgTrace dbgTrace = null;

    private final String instanceName;
    private final TrcVisionProcessor<I, O> visionProcessor;
    private final I[] imageBuffers;
    private final TrcTaskMgr.TaskObject visionTaskObj;
    private boolean taskEnabled = false;
    private int imageIndex = 0;
    private volatile O[] detectedObjects = null;
    private boolean videoOutEnabled = false;

    private TrcDbgTrace tracer = null;
    private double totalTime = 0.0;
    private long totalFrames = 0;
    private double taskStartTime = 0.0;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param visionProcessor specifies the vision processor object.
     * @param imageBuffers specifies an array of image buffers.
     */
    public TrcVisionTask(
        String instanceName, TrcVisionProcessor<I, O> visionProcessor, I[] imageBuffers)
    {
        if (debugEnabled)
        {
            dbgTrace = new TrcDbgTrace(moduleName, tracingEnabled, traceLevel, msgLevel);
        }

        this.instanceName = instanceName;
        this.visionProcessor = visionProcessor;
        this.imageBuffers = imageBuffers;
        visionTaskObj = TrcTaskMgr.createTask(instanceName, this::visionTask);
    }   //TrcVisionTask

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
     * This method enables/disables vision processing performance report.
     *
     * @param tracer specifies a tracer to enable performance report, null to disable.
     */
    public void setPerfReportEnabled(TrcDbgTrace tracer)
    {
        this.tracer = tracer;
    }   //setPerfReportEnabled

    /**
     * This method enables/disables the vision task. As long as the task is enabled, it will continue to
     * acquire/process images.
     *
     * @param enabled specifies true to enable vision task, false to disable.
     */
    public synchronized void setTaskEnabled(boolean enabled)
    {
        final String funcName = "setTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
        }

        if (enabled && !taskEnabled)
        {
            detectedObjects = null;
            totalTime = 0.0;
            totalFrames = 0;
            taskStartTime = TrcTimer.getCurrentTime();
            visionTaskObj.registerTask(TrcTaskMgr.TaskType.STANDALONE_TASK);
        }
        else if (!enabled && taskEnabled)
        {
            visionTaskObj.unregisterTask();
            detectedObjects = null;
        }
        taskEnabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setTaskEnabled

    /**
     * This method returns the state of the vision task.
     *
     * @return true if the vision task is enabled, false otherwise.
     */
    public synchronized boolean isTaskEnabled()
    {
        final String funcName = "isTaskEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%s", taskEnabled);
        }

        return taskEnabled;
    }   //isTaskEnabled

    /**
     * This method enables/disables image annotation of the detected object rects.
     *
     * @param enabled specifies true to enable video out, false to disable.
     */
    public synchronized void setVideoOutEnabled(boolean enabled)
    {
        final String funcName = "setVideoOutEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%s", enabled);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        videoOutEnabled = enabled;
    }   //setVideoOutEnabled

    /**
     * This method sets the vision task processing interval.
     *
     * @param interval specifies the processing interval in msec. If 0, process as fast as the CPU can run.
     */
    public synchronized void setProcessingInterval(long interval)
    {
        final String funcName = "setProcessingInterval";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "interval=%dms", interval);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }

        visionTaskObj.setTaskInterval(interval);
    }   //setProcessingInterval

    /**
     * This method returns the vision task processing interval.
     *
     * @return vision task processing interval in msec.
     */
    public synchronized long getProcessingInterval()
    {
        final String funcName = "getProcessingInterval";
        long interval = visionTaskObj.getTaskInterval();

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API);
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API, "=%d", interval);
        }

        return interval;
    }   //getProcessingInterval

    /**
     * This method returns the last detected objects. Note that this call consumes the objects, meaning if this method
     * is called again before the next frame is finished processing, it will return a null.
     *
     * @return the last detected objects.
     */
    public synchronized O[] getDetectedObjects()
    {
        O[] objects = detectedObjects;
        detectedObjects = null;

        return objects;
    }   //getDetectedObjects

    /**
     * This method runs periodically to do vision processing.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the current robot run mode.
     */
    private synchronized void visionTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "visionTask";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (visionProcessor.getFrame(imageBuffers[imageIndex]))
        {
            double startTime = TrcTimer.getCurrentTime();
            //
            // Capture an image and subject it for object detection. The object detector produces an array of
            // rectangles representing objects detected.
            //
            detectedObjects = visionProcessor.processFrame(imageBuffers[imageIndex]);

            if (videoOutEnabled)
            {
                visionProcessor.annotateFrame(imageBuffers[imageIndex], detectedObjects);
            }

            double elapsedTime = TrcTimer.getCurrentTime() - startTime;
            totalTime += elapsedTime;
            totalFrames++;
            if (tracer != null)
            {
                tracer.traceInfo(
                    funcName, "AvgProcessTime=%.3f sec, FrameRate=%.1f",
                    totalTime/totalFrames, totalFrames/(TrcTimer.getCurrentTime() - taskStartTime));
            }
            //
            // Switch to the next buffer so that we won't clobber the info while the client is accessing it.
            //
            imageIndex = (imageIndex + 1) % imageBuffers.length;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //visionTask

}   //class TrcVisionTask
