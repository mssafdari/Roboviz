package com.tests;

import android.app.Activity;
import android.util.Log;
import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

public abstract class BaseTest {
    private static Activity activity;
    private static int passed = 0;
    private static int failed = 0;
    private static final String TAG = "BaseTest";

    // Abstract methods that subclasses must implement
    protected abstract String getTestTag();
    protected abstract String[] getTestNames();
    protected abstract boolean[] runTests();
    protected abstract String getError();

    // Template method for running all tests
    public String run(Boolean verbose) {
        String msg = "";
        String[] testNames = getTestNames();
        boolean[] testResults = runTests();
        String err = getError();
        // Reset counters before running tests
        passed = 0;
        failed = 0;

        for (int i = 0; i < testNames.length && i < testResults.length; i++) {
            msg += check(testResults[i], testNames[i]);
        }

        msg += "Passed: " + passed + "\n";
        msg += "Failed: " + failed + "\n";
        if(verbose)
        msg += "errors;(\n" + err;
        return msg;
    }

    // Common initialization
    public static void init(Activity activity) {
        BaseTest.activity = activity;
    }

    // Common logging method
    protected String logToFile(String message) {
        String newmsg = "";
        try {
            File logFile = new File(activity.getExternalFilesDir(null), "test_results.log");
            newmsg = activity.getExternalFilesDir(null).toString();
            FileOutputStream fos = new FileOutputStream(logFile, true);
            PrintWriter pw = new PrintWriter(fos);
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
            String timestamp = sdf.format(new Date());

            pw.println(timestamp + " - " + getTestTag() + " - " + message);
            pw.close();
            fos.close();

            Log.d(TAG, "Logged to file: " + logFile.getAbsolutePath());
            Log.d(TAG, "Logged to file: " + message);
        } catch (Exception e) {
            Log.e(TAG, "Failed to write to log file: " + e.getMessage());
            e.printStackTrace();
        }
        return newmsg;
    }

    // Common check method
    protected String check(boolean result, String name) {
        String res = "";
        if (result) {
            passed++;
            String msg = "✓ " + name;
            Log.d(getTestTag(), msg);
            System.out.println(msg);
            logToFile(msg);
            res += msg + "\n";
        } else {
            failed++;
            String msg = "✗ " + name;
            Log.d(getTestTag(), msg);
            System.out.println(msg);
            logToFile(msg);
            res += msg + "\n";
        }
        return res;
    }

    // Helper method to get current counters (for debugging)
    protected static String getStats() {
        return "Passed: " + passed + ", Failed: " + failed;
    }

    // Optional: method to reset counters
    protected static void resetCounters() {
        passed = 0;
        failed = 0;
    }
}
