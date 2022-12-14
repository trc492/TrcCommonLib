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

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * This class implements a ThresholdTrigger. It monitors the value source against the lower and upper threshold
 * values. If the value stays within the lower and upper thresholds for at least the given settling period, the
 * the trigger state is set to active and the trigger callback is called. If the value exits the threshold range,
 * the trigger state is set to inactive and the trigger callback is also called.
 */
public class TrcThresholdTrigger implements TrcTrigger
{
    private static final String moduleName = "TrcThresholdTrigger";
    private static final TrcDbgTrace globalTracer = TrcDbgTrace.getGlobalTracer();
    private static final boolean debugEnabled = false;

    /**
     * This interface implements the method to read the sensor value.
     */
    public interface ValueSource
    {
        /**
         * This method reads the sensor value.
         *
         * @return sensor value.
         */
        double getValue();

    }   //interface ValueSource

    /**
     * This class encapsulates the trigger state. Access to this object must be thread safe (i.e. needs to be
     * synchronized).
     */
    private static class TriggerState
    {
        Double lowerThreshold = null;
        Double upperThreshold = null;
        Double settlingPeriod = null;
        volatile boolean triggerActive = false;
        volatile double startTime = 0.0;
        volatile boolean triggerEnabled = false;
        final ArrayList<Double> recordedData = new ArrayList<>();

        @Override
        public String toString()
        {
            return String.format(
                Locale.US,
                "(lowerThreshold=%.3f,upperThreshold=%.3f,settling=%.3f,active=%s,startTime=%.3f,enabled=%s,data=%s)",
                lowerThreshold, upperThreshold, settlingPeriod, triggerActive, startTime, triggerEnabled,
                Arrays.toString(recordedData.toArray()));
        }   //toString

    }   //class TriggerState

    private final String instanceName;
    private final ValueSource valueSource;
    private final TrcEvent.Callback triggerCallback;
    private final TriggerState triggerState;
    private final TrcEvent callbackEvent;
    private final AtomicBoolean callbackContext;
    private final TrcTaskMgr.TaskObject triggerTaskObj;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param valueSource specifies the interface that implements the value source.
     * @param triggerCallback specifies the callback handler to notify when the trigger state changed.
     */
    public TrcThresholdTrigger(String instanceName, ValueSource valueSource, TrcEvent.Callback triggerCallback)
    {
        if (valueSource == null || triggerCallback == null)
        {
            throw new IllegalArgumentException("ValueSource/TriggerCallback cannot be null.");
        }

        this.instanceName = instanceName;
        this.valueSource = valueSource;
        this.triggerCallback = triggerCallback;
        triggerState = new TriggerState();
        callbackEvent = new TrcEvent(instanceName + ".callbackEvent");
        callbackContext = new AtomicBoolean();
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);
    }   //TrcThresholdTrigger

    /**
     * This method returns the instance name and its state.
     *
     * @return instance name and state.
     */
    @Override
    public String toString()
    {
        String str;

        synchronized (triggerState)
        {
            str = String.format(Locale.US, "%s.%s=%s", moduleName, instanceName, triggerState);
        }

        return str;
    }   //toString

    //
    // Implements TrcSensorTrigger abstract methods.
    //

    /**
     * This method enables/disables the task that monitors the sensor value.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    @Override
    public void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        synchronized (triggerState)
        {
            if (enabled && !triggerState.triggerEnabled)
            {
                if (triggerState.lowerThreshold == null || triggerState.upperThreshold == null ||
                    triggerState.settlingPeriod == null)
                {
                    throw new RuntimeException("Must call setTrigger first before enabling the trigger.");
                }

                triggerState.startTime = TrcUtil.getCurrentTime();
                triggerState.recordedData.clear();
                triggerTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
            }
            else if (!enabled && triggerState.triggerEnabled)
            {
                triggerTaskObj.unregisterTask();
            }
            triggerState.triggerActive = false;
            triggerState.triggerEnabled = enabled;

            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.%s: enabled=%s (state=%s)", moduleName, instanceName, enabled, triggerState);
            }
        }
    }   //setEnabled

    /**
     * This method checks if the trigger task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    @Override
    public boolean isEnabled()
    {
        return triggerState.triggerEnabled;
    }   //isEnabled

    /**
     * This method reads the current analog sensor value. It may return null if it failed to read the sensor.
     *
     * @return current sensor value, null if it failed to read the sensor.
     */
    @Override
    public double getSensorValue()
    {
        return valueSource.getValue();
    }   //getValue

    /**
     * This method reads the current digital sensor state (not supported).
     *
     * @return current sensor state.
     */
    @Override
    public boolean getSensorState()
    {
        return triggerState.triggerActive;
    }   //getState

    /**
     * This method sets the lower/upper threshold values within which the sensor reading must stay for at least the
     * settling period for it to trigger the notification.
     *
     * @param lowerThreshold specifies the lower threshold value for the trigger.
     * @param upperThreshold specifies the upper threshold value for the trigger.
     * @param settlingPeriod specifies the period in seconds the sensor value must stay within threshold range for it
     *                       to trigger.
     */
    public void setTrigger(double lowerThreshold, double upperThreshold, double settlingPeriod)
    {
        final String funcName = "setTrigger";

        if (debugEnabled)
        {
            globalTracer.traceInfo(
                funcName, "%s.%s: lowerThreshold=%f, upperThreshold=%f, settingPeriod=%.3f",
                moduleName, instanceName, lowerThreshold, upperThreshold, settlingPeriod);
        }

        synchronized (triggerState)
        {
            triggerState.lowerThreshold = lowerThreshold;
            triggerState.upperThreshold = upperThreshold;
            triggerState.settlingPeriod = settlingPeriod;
        }
    }   //setTrigger

    /**
     * This method returns an array of the data recorded during the trigger settling period.
     *
     * @return array of trigger settling data, null if no data recorded.
     */
    public Double[] getTriggerSettlingData()
    {
        Double[] data = null;

        synchronized (triggerState)
        {
            int arraySize = triggerState.recordedData.size();

            if (arraySize > 0)
            {
                data = new Double[arraySize];
                triggerState.recordedData.toArray(data);
            }
        }

        return data;
    }   //getTriggerSettlingData

    /**
     * This method is called periodically to check if the sensor value is within the lower and upper threshold range.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "triggerTask";
        double currTime = TrcUtil.getCurrentTime();
        double currValue = valueSource.getValue();
        boolean triggered = false;
        boolean active;

        synchronized (triggerState)
        {
            if (currValue < triggerState.lowerThreshold || currValue > triggerState.upperThreshold)
            {
                // Outside of threshold range.
                if (triggerState.triggerActive)
                {
                    // Only fires a trigger event when the threshold range is first exited (i.e. edge event).
                    triggerState.triggerActive = false;
                    triggered = true;
                }
                triggerState.startTime = currTime;
                triggerState.recordedData.clear();
            }
            else if (currTime >= triggerState.startTime + triggerState.settlingPeriod)
            {
                // Inside of threshold range and past settling period.
                if (!triggerState.triggerActive)
                {
                    // Only fires a trigger event when the threshold range is first entered and stayed for settling
                    // period (i.e. edge event).
                    triggerState.recordedData.add(currValue);
                    triggerState.triggerActive = true;
                    triggered = true;
                }
            }
            else
            {
                triggerState.recordedData.add(currValue);
            }
            active = triggerState.triggerActive;
        }

        if (triggered)
        {
            if (debugEnabled)
            {
                globalTracer.traceInfo(
                    funcName, "%s.%s: Triggered (state=%s)", moduleName, instanceName, active);
            }

            callbackContext.set(active);
            callbackEvent.setCallback(triggerCallback, callbackContext);
            callbackEvent.signal();
        }
    }   //triggerTask

}   //class TrcThresholdTrigger
