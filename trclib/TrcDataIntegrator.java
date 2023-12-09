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

/**
 * This class does data integration for sensors that have one or more axes. Some value sensors such as gyros and
 * accelerometers may need to integrate their data to provide heading from gyro rotation rate, and velocity or
 * distance from accelerometer acceleration data. This class uses a periodic task to do integration and optionally
 * double integration.
 */
public class TrcDataIntegrator<D>
{
    private final String instanceName;
    private final TrcSensor<D> sensor;
    private final D dataType;
    private final TrcTaskMgr.TaskObject integratorTaskObj;
    private final TrcSensor.SensorData<Double>[] inputData;
    private final TrcSensor.SensorData<Double>[] integratedData;
    private final TrcSensor.SensorData<Double>[] doubleIntegratedData;
    private final double[] prevTimes;

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor object that needs integration.
     * @param dataType specifies the data type to be integrated.
     * @param doubleIntegration specifies true to do double integration, false otherwise.
     */
    public TrcDataIntegrator(String instanceName, TrcSensor<D> sensor, D dataType, boolean doubleIntegration)
    {
        if (sensor == null)
        {
            throw new NullPointerException("sensor cannot be null.");
        }

        int numAxes = sensor.getNumAxes();

        this.instanceName = instanceName;
        this.sensor = sensor;
        this.dataType = dataType;
        integratorTaskObj = TrcTaskMgr.createTask(instanceName + ".integratorTask", this::integratorTask);

        inputData = new TrcSensor.SensorData[numAxes];
        integratedData = new TrcSensor.SensorData[numAxes];
        doubleIntegratedData = doubleIntegration? new TrcSensor.SensorData[numAxes]: null;
        prevTimes = new double[numAxes];

        for (int i = 0; i < numAxes; i++)
        {
            integratedData[i] = new TrcSensor.SensorData<>(0.0, 0.0);
            if (doubleIntegratedData != null)
            {
                doubleIntegratedData[i] = new TrcSensor.SensorData<>(0.0, 0.0);
            }
            prevTimes[i] = 0.0;
        }
    }   //TrcDataIntegrator

    /**
     * Constructor: Creates an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param sensor specifies the sensor object that needs integration.
     * @param dataType specifies the data type to be integrated.
     */
    public TrcDataIntegrator(String instanceName, TrcSensor<D> sensor, D dataType)
    {
        this(instanceName, sensor, dataType, false);
    }   //TrcDataProcessor

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
     * This method enables the data integrator. The data integrator is not automatically enabled when created. You
     * must explicitly call this method to enable the data integrator.
     *
     * @param enabled specifies true for enabling the data processor, disabling it otherwise.
     */
    public synchronized void setEnabled(boolean enabled)
    {
        if (enabled)
        {
            reset();
            integratorTaskObj.registerTask(TrcTaskMgr.TaskType.INPUT_TASK);
        }
        else
        {
            integratorTaskObj.unregisterTask();
        }
    }   //setEnabled

    /**
     * This method resets the indexed integratedData and doubleIntegratedData.
     *
     * @param index specifies the index.
     */
    public synchronized void reset(int index)
    {
        prevTimes[index] = TrcTimer.getCurrentTime();
        integratedData[index].value = 0.0;
        if (doubleIntegratedData != null)
        {
            doubleIntegratedData[index].value = 0.0;
        }
    }   //reset

    /**
     * This method resets all integratorData and doubleIntegratedData.
     */
    public synchronized void reset()
    {
        for (int i = 0; i < integratedData.length; i++)
        {
            reset(i);
        }
    }   //reset

    /**
     * This method returns the last indexed input data.
     *
     * @param index specifies the index.
     * @return the last indexed input data.
     */
    public synchronized TrcSensor.SensorData<Double> getInputData(int index)
    {
        return new TrcSensor.SensorData<>(inputData[index].timestamp, inputData[index].value);
    }   //getInputData

    /**
     * This method returns the last indexed integrated data.
     *
     * @param index specifies the index.
     * @return last indexed integrated data.
     */
    public synchronized TrcSensor.SensorData<Double> getIntegratedData(int index)
    {
        return new TrcSensor.SensorData<>(integratedData[index].timestamp, integratedData[index].value);
    }   //getIntegratedData

    /**
     * This method returns the last indexed double integrated data.
     *
     * @param index specifies the index.
     * @return last indexed double integrated data.
     */
    public synchronized TrcSensor.SensorData<Double> getDoubleIntegratedData(int index)
    {
        return new TrcSensor.SensorData<>(doubleIntegratedData[index].timestamp, doubleIntegratedData[index].value);
    }   //getDoubleIntegratedData

    /**
     * This method is called periodically to do data integration.
     *
     * @param taskType specifies the type of task being run.
     * @param runMode specifies the competition mode that is running.
     * @param slowPeriodicLoop specifies true if it is running the slow periodic loop on the main robot thread,
     *        false otherwise.
     */
    private synchronized void integratorTask(
        TrcTaskMgr.TaskType taskType, TrcRobot.RunMode runMode, boolean slowPeriodicLoop)
    {
        boolean allZeroAxis = true;
        double[] deltaTime = new double[inputData.length];
        for (int i = 0; i < inputData.length; i++)
        {
            //
            // Get sensor data.
            //
            inputData[i] = sensor.getProcessedData(i, dataType);
            deltaTime[i] = inputData[i].timestamp - prevTimes[i];
            if (inputData[i].value != 0.0)
            {
                allZeroAxis = false;
            }
            //
            // Do integration.
            //
            integratedData[i].timestamp = inputData[i].timestamp;
            integratedData[i].value = integratedData[i].value + inputData[i].value*deltaTime[i];
            prevTimes[i] = inputData[i].timestamp;
        }

        //
        // Do double integration if necessary.
        //
        if (doubleIntegratedData != null)
        {
            for (int i = 0; i < inputData.length; i++)
            {
                doubleIntegratedData[i].timestamp = inputData[i].timestamp;
                if (allZeroAxis)
                {
                    integratedData[i].value = 0.0;
                }
                else
                {
                    doubleIntegratedData[i].value =
                            doubleIntegratedData[i].value + integratedData[i].value*deltaTime[i];
                }
            }
        }
    }   //integratorTask

}   //class TrcDataIntegrator
