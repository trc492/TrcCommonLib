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

import java.io.File;
import java.util.Locale;

/**
 * This class implements the Debug Tracer.
 */
public class TrcDbgTrace
{
    private static final String moduleName = TrcDbgTrace.class.getSimpleName();

    //
    // Deprecated: starts here
    //
    /**
     * This enum specifies the different debug tracing levels. They are used in the traceEnter and traceExit methods.
     */
    public enum TraceLevel
    {
        QUIET(0),
        INIT(1),
        API(2),
        CALLBK(3),
        EVENT(4),
        FUNC(5),
        TASK(6),
        UTIL(7),
        HIFREQ(8);

        private final int value;

        TraceLevel(int value)
        {
            this.value = value;
        }   //TraceLevel

        public int getValue()
        {
            return this.value;
        }   //getValue

    }   //enum TraceLevel

    private static int indentLevel = 0;
    private boolean traceEnabled = false;
    private TraceLevel traceLevel = TraceLevel.FUNC;

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     * @param traceEnabled specifies true to enable debug tracing, false to disable.
     * @param traceLevel specifies the trace level.
     * @param msgLevel specifies the message level.
     */
    // Deprecated
    public TrcDbgTrace(String instanceName, boolean traceEnabled, TraceLevel traceLevel, MsgLevel msgLevel)
    {
        this(instanceName, null);
        setDbgTraceConfig(traceEnabled, msgLevel);
    }   //TrcDbgTrace

    /**
     * This method sets the global tracer configuration. The OpMode trace object was created with default
     * configuration of disabled method tracing, method tracing level is set to API and message trace level
     * set to INFO. Call this method if you want to change the configuration.
     *
     * @param traceEnabled specifies true if enabling method tracing.
     * @param msgLevel specifies the message level.
     */
    // Deprecated
    public static void setGlobalTracerConfig(boolean traceEnabled, TrcDbgTrace.MsgLevel msgLevel)
    {
        globalTracer.setDbgTraceConfig(traceEnabled, msgLevel);
    }   //setGlobalTracerConfig

    /**
     * This method sets the trace level, message level of the debug tracer. It can also enables/disables function
     * tracing.
     *
     * @param traceEnabled specifies true to enable function tracing, false to disable.
     * @param msgLevel specifies the message level.
     */
    public void setDbgTraceConfig(boolean traceEnabled, MsgLevel msgLevel)
    {
        this.traceEnabled = traceEnabled;
        this.msgLevel = msgLevel;
    }   //setDbgTraceConfig

    /**
     * This method is typically called at the beginning of a method to trace the entry parameters of the method.
     *
     * @param callerName specifies the name to identify the caller.
     * @param funcLevel specifies the trace level.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceEnter(String callerName, TraceLevel funcLevel, String format, Object... args)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            dbgLog.traceMsg(tracePrefix(callerName, true, false) + String.format(format, args) + ")\n");
        }
    }   //traceEnter

    /**
     * This method is typically called at the beginning of a method.
     *
     * @param callerName specifies the name to identify the caller.
     * @param funcLevel specifies the trace level.
     */
    public void traceEnter(String callerName, TraceLevel funcLevel)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            dbgLog.traceMsg(tracePrefix(callerName, true, true));
        }
    }   //traceEnter

    /**
     * This method is typically called at the end of a method to trace the return value of the method.
     *
     * @param callerName specifies the name to identify the caller.
     * @param funcLevel specifies the trace level.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceExit(String callerName, TraceLevel funcLevel, String format, Object... args)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            dbgLog.traceMsg(tracePrefix(callerName, false, false) + String.format(format, args) + "\n");
        }
    }   //traceExitMsg

    /**
     * This method is typically called at the end of a method.
     *
     * @param callerName specifies the name to identify the caller.
     * @param funcLevel specifies the trace level.
     */
    public void traceExit(String callerName, TraceLevel funcLevel)
    {
        if (traceEnabled && funcLevel.getValue() <= traceLevel.getValue())
        {
            dbgLog.traceMsg(tracePrefix(callerName, false, true));
        }
    }   //traceExit

    /**
     * This method returns a trace prefix string. The trace prefix includes the indentation, the instance name and
     * calling method name.
     *
     * @param callerName specifies the name to identify the caller.
     * @param enter specifies true if it is a traceEnter call, false if it is a traceExit call.
     * @param newline specifies true if it should print a newline, false otherwise.
     * @return trace prefix string.
     */
    private String tracePrefix(String callerName, boolean enter, boolean newline)
    {
        StringBuilder prefix = new StringBuilder();

        if (enter)
        {
            indentLevel++;
        }

        for (int i = 0; i < indentLevel; i++)
        {
            prefix.append("| ");
        }

        String methodName = new Throwable().getStackTrace()[2].getMethodName();
        prefix.append(callerName).append(".").append(methodName);

        if (enter)
        {
            prefix.append(newline ? "()\n" : "(");
        }
        else
        {
            prefix.append(newline ? "!\n" : "");
            indentLevel--;
        }

        return prefix.toString();
    }   //tracePrefix
    //
    // Deprecated: ends here
    //

    /**
     * This enum specifies the different debug message levels. They are used in the traceMsg methods.
     */
    public enum MsgLevel
    {
        FATAL(1),
        ERR(2),
        WARN(3),
        INFO(4),
        DEBUG(5),
        VERBOSE(6);

        private final int value;

        MsgLevel(int value)
        {
            this.value = value;
        }   //MsgLevel

        public int getValue()
        {
            return this.value;
        }   //getValue
    }   //enum MsgLevel

    /**
     * This interface provides a platform independent way to write to the debug log. It is mainly for TrcLib which is
     * platform agnostic. A platform dependent class will implement methods in this interface.
     */
    public interface DbgLog
    {
        /**
         * This method is called to print a message with the specified message level to the debug console.
         *
         * @param level specifies the message level.
         * @param msg specifies the message.
         */
        void msg(MsgLevel level, String msg);
        //
        // Deprecated: starts here
        //
        /**
         * This method is called to print a message to the debug console.
         *
         * @param msg specifies the message.
         */
        void traceMsg(String msg);
        //
        // Deprecated: ends here
        //
    }   //interface DbgLog

    private static DbgLog dbgLog = null;
    private static TrcDbgTrace globalTracer = null;
    private static TrcTraceLogger traceLogger = null;

    private final String instanceName;
    private MsgLevel msgLevel = MsgLevel.INFO;

    /**
     * Constructor: Create an instance of the object. This constructor is intended for internal use for creating
     * the global tracer. For creating other tracers, use the constructor with just instanceName.
     *
     * @param instanceName specifies the instance name.
     * @param dbgLog specifies the dbgLog object to be set.
     */
    public TrcDbgTrace(String instanceName, DbgLog dbgLog)
    {
        this.instanceName = instanceName;

        if (dbgLog != null)
        {
            if (TrcDbgTrace.dbgLog == null)
            {
                TrcDbgTrace.dbgLog = dbgLog;
                TrcDbgTrace.globalTracer = this;
            }
            else
            {
                throw new IllegalStateException(
                    "This constructor can only be called internally to create the global Tracer.");
            }
        }
    }   //TrcDbgTrace

    /**
     * Constructor: Create an instance of the object.
     *
     * @param instanceName specifies the instance name.
     */
    public TrcDbgTrace(String instanceName)
    {
        this(instanceName, null);
    }   //TrcDbgTrace

    /**
     * This method sets the message level for this tracer.
     *
     * @param msgLevel specifies the message level.
     */
    public void setTraceMessageLevel(MsgLevel msgLevel)
    {
        this.msgLevel = msgLevel;
    }   //setTraceMessageLevel

    /**
     * This method returns the trace message level.
     *
     * @return trace message level.
     */
    public MsgLevel getTraceMessageLevel()
    {
        return msgLevel;
    }   //getTraceMessageLevel

    /**
     * This method returns a global debug trace object for tracing OpMode code. If it doesn't exist yet, one is
     * created. This is an easy way to quickly get some debug output without a whole lot of setup overhead as the
     * full module-based debug tracing.
     *
     * @return global opMode trace object.
     */
    public static TrcDbgTrace getGlobalTracer()
    {
        return globalTracer;
    }   //getGlobalTracer

    /**
     * This method opens a log file for writing all the trace messages to it.
     *
     * @param traceLogName specifies the full trace log file path name.
     * @return true if log file is successfully opened, false if it failed.
     */
    public static boolean openTraceLog(String traceLogName)
    {
        boolean success = false;

        if (traceLogger == null)
        {
            traceLogger = new TrcTraceLogger(traceLogName);
            success = true;
        }

        return success;
    }   //openTraceLog

    /**
     * This method opens a log file for writing all the trace messages to it. The log file is written to the specified
     * folder. The file name will be formed by concatenating the date-time stamp with the specified file name.
     *
     * @param folderPath specifies the folder path.
     * @param fileName specifies the file name, null if none provided.
     * @return true if log file is successfully opened, false if it failed.
     */
    public static boolean openTraceLog(String folderPath, String fileName)
    {
        //
        // Create the folder if it doesn't exist.
        //
        File folder = new File(folderPath);
        if (!folder.exists())
        {
            folder.mkdirs();
        }
        //
        // Create full log file path.
        //
        String logFileName = folderPath + File.separator + TrcTimer.getCurrentTimeString();

        if (fileName != null)
        {
            logFileName += "!" + fileName;
        }
        logFileName += ".log";

        return openTraceLog(logFileName);
    }   //openTraceLog

    /**
     * This method closes the trace log file.
     */
    public static void closeTraceLog()
    {
        if (traceLogger != null)
        {
            traceLogger.setEnabled(false);
            traceLogger = null;
        }
    }   //closeTraceLog

    /**
     * This method checks if the trace log is opened.
     *
     * @return true if trace log is opened, false otherwise.
     */
    public static boolean isTraceLogOpened()
    {
        return traceLogger != null;
    }   //isTraceLogOpened

    /**
     * This method returns the trace log file name if one is active.
     *
     * @return trace log file name if one is active, null if none.
     */
    public static String getTraceLogName()
    {
        return traceLogger != null? traceLogger.toString(): null;
    }   //getTraceLogName

    /**
     * This method enables/disables the trace log.
     *
     * @param enabled specifies true to enable trace log, false otherwise.
     */
    public static void setTraceLogEnabled(boolean enabled)
    {
        if (traceLogger != null)
        {
            traceLogger.setEnabled(enabled);
        }
    }   //setTraceLogEnabled

    /**
     * This method checks if the trace log is enabled.
     *
     * @return true if trace log is enabled, false if disabled.
     */
    public static boolean isTraceLogEnabled()
    {
        return (traceLogger != null && traceLogger.isEnabled());
    }   //isTraceLogEnabled

    /**
     * This method prints the stack of the given thread to the trace log.
     *
     * @param thread specifies the thread to print its stack.
     */
    public static void printThreadStack(Thread thread)
    {
        StringBuilder sb = new StringBuilder("Thread stack: ");

        for (StackTraceElement ste : thread.getStackTrace())
        {
            sb.append("\n").append(ste);
        }

        globalTracer.traceMsgWorker(moduleName, 2, MsgLevel.INFO, sb.toString());
    }   //printThreadStack

    /**
     * This method prints the stack of the current thread to the trace log.
     */
    public static void printThreadStack()
    {
        printThreadStack(Thread.currentThread());
    }   //printThreadStack

    /**
     * This method is the common worker for all the trace message methods.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param methodIndex specifies the index to the thread stack to obtain the method name.
     * @param level specifies the message level.
     * @param text specifies the message text.
     */
    private void traceMsgWorker(String callerInstance, int methodIndex, MsgLevel level, String text)
    {
        if (level.getValue() <= msgLevel.getValue())
        {
            String msg =
                callerInstance + "." + new Throwable().getStackTrace()[methodIndex].getMethodName() + "_" + level +
                " [" + TrcTimer.getModeElapsedTime() + "] " + text;
            dbgLog.msg(level, msg + "\n");
            if (traceLogger != null)
            {
                traceLogger.logMessage(msg);
            }
        }
    }   //traceMsgWorker

    /**
     * This method is called to print a fatal message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceFatal(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.FATAL.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, text);
        }
    }   //traceFatal

    /**
     * This method is called to print a fatal message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceFatal(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.FATAL.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, String.format(format, args));
        }
    }   //traceFatal

    /**
     * This method is called to print an error message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceErr(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.ERR.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.ERR, text);
        }
    }   //traceErr

    /**
     * This method is called to print an error message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceErr(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.ERR.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.ERR, String.format(format, args));
        }
    }   //traceErr

    /**
     * This method is called to print a warning message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceWarn(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.WARN.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.WARN, text);
        }
    }   //traceWarn

    /**
     * This method is called to print a warning message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceWarn(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.WARN.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.WARN, String.format(format, args));
        }
    }   //traceWarn

    /**
     * This method is called to print an information message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceInfo(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.INFO, text);
        }
    }   //traceInfo

    /**
     * This method is called to print an information message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceInfo(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.INFO, String.format(format, args));
        }
    }   //traceInfo

    /**
     * This method is called to print a debug message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceDebug(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.DEBUG.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, text);
        }
    }   //traceDebug

    /**
     * This method is called to print a debug message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceDebug(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.DEBUG.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, String.format(format, args));
        }
    }   //traceDebug

    /**
     * This method is called to print a verbose message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public void traceVerbose(String callerInstance, String text)
    {
        if (msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, text);
        }
    }   //traceVerbose

    /**
     * This method is called to print a verbose message.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void traceVerbose(String callerInstance, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, String.format(format, args));
        }
    }   //traceVerbose

    /**
     * This method is called to print a fatal message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceFatal(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.FATAL.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, text);
        }
    }   //globalTraceFatal

    /**
     * This method is called to print a fatal message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceFatal(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.FATAL.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.FATAL, String.format(format, args));
        }
    }   //globalTraceFatal

    /**
     * This method is called to print an error message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceErr(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.ERR.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.ERR, text);
        }
    }   //globalTraceErr

    /**
     * This method is called to print an error message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceErr(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.ERR.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.ERR, String.format(format, args));
        }
    }   //globalTraceErr

    /**
     * This method is called to print a warning message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceWarn(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.WARN.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.WARN, text);
        }
    }   //globalTraceWarn

    /**
     * This method is called to print a warning message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceWarn(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.WARN.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.WARN, String.format(format, args));
        }
    }   //globalTraceWarn

    /**
     * This method is called to print an information message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceInfo(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.INFO.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.INFO, text);
        }
    }   //globalTraceInfo

    /**
     * This method is called to print an information message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceInfo(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.INFO.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.INFO, String.format(format, args));
        }
    }   //globalTraceInfo

    /**
     * This method is called to print a debug message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceDebug(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.DEBUG.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, text);
        }
    }   //globalTraceDebug

    /**
     * This method is called to print a debug message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceDebug(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.DEBUG.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.DEBUG, String.format(format, args));
        }
    }   //globalTraceDebug

    /**
     * This method is called to print a verbose message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param text specifies the message text.
     */
    public static void globalTraceVerbose(String callerInstance, String text)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, text);
        }
    }   //globalTraceVerbose

    /**
     * This method is called to print a verbose message using the global tracer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public static void globalTraceVerbose(String callerInstance, String format, Object... args)
    {
        if (globalTracer.msgLevel.value >= MsgLevel.VERBOSE.value)
        {
            globalTracer.traceMsgWorker(callerInstance, 2, MsgLevel.VERBOSE, String.format(format, args));
        }
    }   //globalTraceVerbose

    /**
     * This method logs a MsgLevel.INFO entry that contains information about the match. The entry is in XML format
     * and is intended to be parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param infoName specifies the name to identify the information.
     * @param text specifies the message text.
     */
    public void logInfo(String callerInstance, String infoName, String text)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(callerInstance, 2, MsgLevel.INFO, "<Info name=\"" + infoName + "\" " + text + " />");
        }
    }   //logInfo

    /**
     * This method logs a MsgLevel.INFO entry that contains information about the match. The entry is in XML format
     * and is intended to be parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param infoName specifies the name to identify the information.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void logInfo(String callerInstance, String infoName, String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(
                callerInstance, 2, MsgLevel.INFO,
                "<Info name=\"" + infoName + "\" " + String.format(format, args) + " />");
        }
    }   //logInfo

    /**
     * This method logs a MsgLevel.INFO entry that contains an event. The entry is in XML format and is intended to be
     * parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param eventName specifies the name to identify the event.
     * @param text specifies the message text.
     */
    public void logEvent(String callerInstance, String eventName, String text)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(
                callerInstance, 2, MsgLevel.INFO,
                "<Event name=\"" + eventName + "\" time=\"" + TrcTimer.getModeElapsedTime() + "\" " + text + " />");
        }
    }   //logEvent

    /**
     * This method logs a MsgLevel.INFO entry that contains an event. The entry is in XML format and is intended to be
     * parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param eventName specifies the name to identify the event.
     * @param format specifies the format string of the message.
     * @param args specifies the message arguments.
     */
    public void logEvent(String callerInstance, final String eventName, final String format, Object... args)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceMsgWorker(
                callerInstance, 2, MsgLevel.INFO,
                "<Event name=\"" + eventName + "\" time=\"" + TrcTimer.getModeElapsedTime() + "\" " +
                String.format(format, args) + " />");
        }
    }   //logEvent

    /**
     * This method logs a MsgLevel.INFO entry that contains an event. The entry is in XML format and is intended to be
     * parsed by tools such as TrcTraceLogVisualizer.
     *
     * @param callerInstance specifies the name to identify the caller.
     * @param methodIndex specifies the index to the thread stack to obtain the method name.
     * @param eventName specifies the name to identify the event.
     * @param text specifies the message text.
     */
    private void logEventInternal(String callerInstance, int methodIndex, String eventName, String text)
    {
        traceMsgWorker(
            callerInstance, methodIndex, MsgLevel.INFO,
            "<Event name=\"" + eventName + "\" time=\"" + TrcTimer.getModeElapsedTime() + "\" " + text + " />");
    }   //logEventInternal

    /**
     * This method logs a state info event using the global tracer. The state info event can be used to debug an
     * autonomous state machine. If the state involves PID controlled driving, it also logs the robot's movement.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     * @param ppDrive specifies the purePursuitDrive object, can be null if the state does not involve pp drive.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    private void traceStateInfoWorker(
        String name, Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive, TrcPurePursuitDrive ppDrive,
        TrcRobotBattery battery)
    {
        if (state != null)
        {
            StringBuilder msg = new StringBuilder("tag=\"^^^^^\" " + name + ".state=\"" + state + "\"");

            if (driveBase != null)
            {
                if (pidDrive != null && pidDrive.isActive())
                {
                    TrcPose2D robotPose = driveBase.getFieldPosition();
                    TrcPose2D targetPose = pidDrive.getAbsoluteTargetPose();
                    msg.append(" RobotPose=")
                       .append(robotPose)
                       .append(" TargetPose=")
                       .append(targetPose);
                }

                if (ppDrive != null && ppDrive.isActive())
                {
                    TrcPose2D robotPose = driveBase.getFieldPosition();
                    TrcPose2D robotVel = driveBase.getFieldVelocity();
                    TrcPose2D targetPose = ppDrive.getTargetFieldPosition();
                    msg.append(" RobotPose=")
                       .append(robotPose)
                       .append(" TargetPose=")
                       .append(targetPose)
                       .append(" vel=")
                       .append(robotVel)
                       .append(" Path=")
                       .append(ppDrive.getPath());
                }
            }

            if (battery != null)
            {
                msg.append(String.format(
                    Locale.US, " volt=\"%.2fV(%.2fV)\"", battery.getVoltage(), battery.getLowestVoltage()));
            }

            logEventInternal(name, 4, "StateInfo", msg.toString());
        }
    }   //traceStateInfoWorker

    /**
     * This method logs a state info event using the global tracer. The state info event can be used to debug an
     * autonomous state machine. If the state involves PID controlled driving, it also logs the robot's movement.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     * @param ppDrive specifies the purePursuitDrive object, can be null if the state does not involve pp drive.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    public void traceStateInfo(
        String name, Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive, TrcPurePursuitDrive ppDrive,
        TrcRobotBattery battery)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceStateInfoWorker(name, state, driveBase, pidDrive, ppDrive, battery);
        }
    }   //traceStateInfo

    /**
     * This method logs a state info event. The state info event can be used to debug an autonomous state machine.
     * If the state involves PID controlled driving, it also logs the robot's movement.
     *
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param pidDrive specifies the pidDrive object, can be null if the state does not involve robot movement.
     */
    public void traceStateInfo(String name, Object state, TrcDriveBase driveBase, TrcPidDrive pidDrive)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceStateInfoWorker(name, state, driveBase, pidDrive, null, null);
        }
    }   //traceStateInfo

    /**
     * This method logs a state info event. The state info event can be used to debug an autonomous state machine.
     * If the state involves PID controlled driving, it also logs the robot's movement.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param driveBase specifies the robot drive base, can be null if the state does not involve robot movement.
     * @param ppDrive specifies the purePursuitDrive object, can be null if the state does not involve pp drive.
     */
    public void traceStateInfo(String name, Object state, TrcDriveBase driveBase, TrcPurePursuitDrive ppDrive)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceStateInfoWorker(name, state, driveBase, null, ppDrive, null);
        }
    }   //traceStateInfo

    /**
     * This method logs a state info event. The state info event can be used to debug an autonomous state machine.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     * @param battery specifies the robot battery object, can be null if not interested in battery info.
     */
    public void traceStateInfo(String name, Object state, TrcRobotBattery battery)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceStateInfoWorker(name, state, null, null, null, battery);
        }
    }   //traceStateInfo

    /**
     * This method logs a state info event. The state info event can be used to debug an autonomous state machine.
     *
     * @param name specifies the instance name of the state machine.
     * @param state specifies the current state of the state machine.
     */
    public void traceStateInfo(String name, Object state)
    {
        if (msgLevel.value >= MsgLevel.INFO.value)
        {
            traceStateInfoWorker(name, state, null, null, null, null);
        }
    }   //traceStateInfo

}   //class TrcDbgTrace
