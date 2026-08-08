package com.roboviz;

import android.content.Context;
import android.content.res.AssetManager;
import android.util.Log;

import java.io.InputStream;
import java.util.Arrays;

public class URDFUtils {

    private static final String TAG = "URDFUtils";

    /**
     * Check if a URDF file exists in assets
     * @param context The context to access assets
     * @param filename The name of the URDF file to check
     * @return true if file exists, false otherwise
     */
    public static boolean urdfFileExists(Context context, String filename) {
        try {
            AssetManager assetManager = context.getAssets();
            String[] files = assetManager.list("");

            if (files == null) {
                Log.e(TAG, "Failed to list assets");
                return false;
            }

            for (String file : files) {
                if (file.equals(filename)) {
                    return true;
                }
            }

            Log.d(TAG, "File '" + filename + "' not found. Available files: " + Arrays.toString(files));
            return false;

        } catch (Exception e) {
            Log.e(TAG, "Error checking for URDF file: " + e.getMessage());
            return false;
        }
    }

    /**
     * Get a list of all URDF files in assets
     * @param context The context to access assets
     * @return Array of URDF filenames (files ending with .urdf or .xml)
     */
    public static String[] getURDFFiles(Context context) {
        try {
            AssetManager assetManager = context.getAssets();
            String[] allFiles = assetManager.list("");

            if (allFiles == null) {
                return new String[0];
            }

            java.util.ArrayList<String> urdfFiles = new java.util.ArrayList<String>();

            for (int i = 0; i < allFiles.length; i++) {
                String file = allFiles[i];
                if (file.endsWith(".urdf") || file.endsWith(".urdf.xml") || file.endsWith(".xml")) {
                    urdfFiles.add(file);
                }
            }

            String[] result = new String[urdfFiles.size()];
            return urdfFiles.toArray(result);

        } catch (Exception e) {
            Log.e(TAG, "Error getting URDF files: " + e.getMessage());
            return new String[0];
        }
    }

    /**
     * Load a robot from a URDF file in assets
     * @param context The context to access assets
     * @param filename The name of the URDF file
     * @return Robot object or null if loading fails
     */
    public static Robot loadRobotFromURDF(Context context, String filename) {
        try {
            if (!urdfFileExists(context, filename)) {
                Log.e(TAG, "URDF file not found: " + filename);
                return null;
            }

            InputStream is = context.getAssets().open(filename);
            Robot robot = new UrdfParser().parse(is);
            is.close();

            Log.d(TAG, "Successfully loaded robot: " + robot.name + " from " + filename);
            return robot;

        } catch (Exception e) {
            Log.e(TAG, "Error loading URDF file '" + filename + "': " + e.getMessage());
            e.printStackTrace();
            return null;
        }
    }

    /**
     * Load a robot from the default URDF file
     * @param context The context to access assets
     * @return Robot object or null if loading fails
     */
    public static Robot loadDefaultRobot(Context context) {
        return loadRobotFromURDF(context, "ur5.urdf.xml");
    }

    /**
     * Get the list of available URDF files as a formatted string
     * @param context The context to access assets
     * @return Formatted string of available URDF files
     */
    public static String getAvailableURDFFilesString(Context context) {
        String[] files = getURDFFiles(context);
        if (files.length == 0) {
            return "No URDF files found in assets";
        }
        return "Available URDF files: " + Arrays.toString(files);
    }

    /**
     * Check if a URDF file exists and return a user-friendly message
     * @param context The context to access assets
     * @param filename The name of the URDF file to check
     * @return User-friendly message about file status
     */
    public static String getURDFFileStatus(Context context, String filename) {
        if (urdfFileExists(context, filename)) {
            return "URDF file '" + filename + "' found ✓";
        } else {
            return "URDF file '" + filename + "' not found ✗\n" + getAvailableURDFFilesString(context);
        }
    }
}
