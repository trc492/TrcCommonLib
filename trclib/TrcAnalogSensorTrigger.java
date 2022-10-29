/*
 * Copyright (c) 2015 Titan Robotics Club (http://www.titanrobotics.com)
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

import java.util.Arrays;

/**
 * This class implements an AnalogSensorTrigger. It monitors the value of the analog sensor against an array of
 * threshold values. If the sensor reading crosses any of the thresholds in the array, it will call a notification
 * handler so that an action could be performed.
 */
public class TrcAnalogSensorTrigger<D> extends TrcTrigger
{
    private final TrcSensor<D> sensor;
    private final int index;
    private final D dataType;
    private final AnalogTriggerHandler triggerHandler;
    private final TrcTaskMgr.TaskObject triggerTaskObj;
    private double[] thresholds;
    private int sensorZone;
    private double sensorValue;
    private boolean enabled = false;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor that is used to detect the trigger.
     * @param index specifies the data index of the sensor to read the sensor value.
     * @param dataType specifies the data type of the sensor to read the sensor value.
     * @param dataPoints specifies an array of trigger points or an array of thresholds if dataIsTrigger is true.
     * @param triggerHandler specifies the object to handle the trigger event.
     * @param dataIsTrigger specifies true if dataPoints specifies an array of trigger points, false if it is an
     *                      array of thresholds. Trigger points will be converted to threshold points.
     */
    public TrcAnalogSensorTrigger(
        String instanceName, TrcSensor<D> sensor, int index, D dataType, double[] dataPoints,
        AnalogTriggerHandler triggerHandler, boolean dataIsTrigger)
    {
        super(instanceName);

        if (sensor == null || triggerHandler == null)
        {
            throw new IllegalArgumentException("Sensor/TriggerHandler cannot be null.");
        }

        this.sensor = sensor;
        this.index = index;
        this.dataType = dataType;
        this.triggerHandler = triggerHandler;
        triggerTaskObj = TrcTaskMgr.createTask(instanceName + ".triggerTask", this::triggerTask);

        if (dataIsTrigger)
        {
            setTriggerPoints(dataPoints);
        }
        else
        {
            setThresholds(dataPoints);
        }

        sensorValue = getValue();
        sensorZone = getValueZone(sensorValue);
    }   //TrcAnalogSensorTrigger

    /**
     * This method creates and threshold array and calculates all the threshold values. A threshold value is the
     * average of two adjacent trigger points.
     *
     * @param triggerPoints specifies the array of trigger points.
     */
    private synchronized void setTriggerPoints(double[] triggerPoints)
    {
        final String funcName = "setTriggerPoints";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.FUNC, "triggerPts=%s", Arrays.toString(triggerPoints));
        }

        if (triggerPoints == null)
        {
            throw new IllegalArgumentException("TriggerPoints cannot be null");
        }

        if (triggerPoints.length < 2)
        {
            throw new IllegalArgumentException("zoneValues must have at least two elements.");
        }

        thresholds = new double[triggerPoints.length - 1];
        for (int i = 0; i < thresholds.length; i++)
        {
            thresholds[i] = (triggerPoints[i] + triggerPoints[i + 1])/2.0;
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%s", Arrays.toString(thresholds));
        }
    }   //setTriggerPoints

    /**
     * This method stores the threshold array.
     *
     * @param thresholds specifies the array of thresholds.
     */
    private synchronized void setThresholds(double[] thresholds)
    {
        final String funcName = "setThresholds";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(
                    funcName, TrcDbgTrace.TraceLevel.FUNC, "thresholds=%s", Arrays.toString(thresholds));
        }

        if (thresholds == null)
        {
            throw new IllegalArgumentException("thresholds cannot be null");
        }

        if (thresholds.length == 0)
        {
            throw new IllegalArgumentException("thresholds cannot be empty");
        }

        this.thresholds = Arrays.copyOf(thresholds, thresholds.length);

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC);
        }
    }   //setThresholds

    /**
     * This method returns the current zone it is in.
     *
     * @return current zone index.
     */
    public synchronized int getCurrentZone()
    {
        return sensorZone;
    }   //getCurrentZone

    /**
     * This method returns the last sensor value.
     *
     * @return last sensor value.
     */
    public synchronized double getCurrentValue()
    {
        return sensorValue;
    }   //getCurrentValue

    //
    // Implements TrcSensorTrigger abstract methods.
    //

    /**
     * This method enables/disables the task that monitors the sensor value.
     *
     * @param enabled specifies true to enable, false to disable.
     */
    @Override
    public synchronized void setEnabled(boolean enabled)
    {
        final String funcName = "setEnabled";

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.API, "enabled=%b", enabled);
        }

        if (enabled)
        {
            sensorValue = getValue();
            sensorZone = getValueZone(sensorValue);
            triggerTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            triggerTaskObj.unregisterTask();
        }
        this.enabled = enabled;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.API);
        }
    }   //setEnabled

    /**
     * This method checks if the trigger task is enabled.
     *
     * @return true if enabled, false otherwise.
     */
    public boolean isEnabled()
    {
        return enabled;
    }   //isEnabled

    /**
     * This method reads the current analog sensor value. It may return null if it failed to read the sensor.
     *
     * @return current sensor value, null if it failed to read the sensor.
     */
    @Override
    public double getValue()
    {
        TrcSensor.SensorData<Double> data = sensor.getProcessedData(index, dataType);
        return data != null && data.value != null? data.value: 0.0;
    }   //getValue

    /**
     * This method reads the current digital sensor state (not supported).
     *
     * @return current sensor state.
     */
    @Override
    public boolean getState()
    {
        throw new UnsupportedOperationException("Analog sensor does not support digital state.");
    }   //getSensorState

    /**
     * This method determines the sensor zone with the given sensor value.
     *
     * @param value specifies the sensor value.
     * @return sensor zone the value is in.
     */
    private int getValueZone(double value)
    {
        final String funcName = "getValueZone";
        int zone = -1;

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.FUNC, "value=%f", value);
        }

        if (value < thresholds[0])
        {
            zone = 0;
        }
        else
        {
            for (int i = 0; i < thresholds.length - 1; i++)
            {
                if (value >= thresholds[i] && value < thresholds[i + 1])
                {
                    zone = i + 1;
                    break;
                }
            }

            if (zone == -1)
            {
                zone = thresholds.length;
            }
        }

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.FUNC, "=%d", zone);
        }

        return zone;
    }   //getValueZone

    /**
     * This method is called periodically to check the current sensor value against the threshold array to see it
     * crosses any thresholds and the triggerHandler will be notified.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running. (e.g. Autonomous, TeleOp, Test).
     */
    private synchronized void triggerTask(TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode)
    {
        final String funcName = "triggerTask";
        double currValue = getValue();
        int currZone = getValueZone(currValue);

        if (debugEnabled)
        {
            dbgTrace.traceEnter(funcName, TrcDbgTrace.TraceLevel.TASK, "taskType=%s,runMode=%s", taskType, runMode);
        }

        if (currZone != sensorZone)
        {
            //
            // We have crossed to another zone, let's notify somebody.
            //
            if (triggerHandler != null)
            {
                triggerHandler.analogTriggerEvent(sensorZone, currZone, currValue);
            }

            if (debugEnabled)
            {
                dbgTrace.traceInfo(funcName, "%s crossing zones %d->%d (value=%f)",
                                   instanceName, sensorZone, currZone, currValue);
            }
        }
        sensorValue = currValue;
        sensorZone = currZone;

        if (debugEnabled)
        {
            dbgTrace.traceExit(funcName, TrcDbgTrace.TraceLevel.TASK);
        }
    }   //triggerTask

}   //class TrcAnalogSensorTrigger
