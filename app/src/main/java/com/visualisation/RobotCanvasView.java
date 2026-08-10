package com.visualisation;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;
import java.util.ArrayList;
import com.math.se3group;
import com.math.Vector3;
import com.roboviz.RobotControlActivity;
import java.io.StringWriter;
import java.io.PrintWriter;

public class RobotCanvasView extends View
{
    private Paint jointPaint;
    private Paint linkPaint;
    private Paint textPaint;
    private ArrayList<se3group> jointPoses = new ArrayList<>();
    private float scale = 100f;  // Pixels per meter
    private float centerX = 500f;
    private float centerY = 800f;
    private String el="\n";

    // Isometric projection angles
    private static final double ISO_ANGLE_X = Math.toRadians(30);
    private static final double ISO_ANGLE_Z = Math.toRadians(45);

    public RobotCanvasView(Context context)
    {
        super(context);
        init();
    }

    public RobotCanvasView(Context context, AttributeSet attrs)
    {
        super(context, attrs);
        init();
    }

    private void init()
    {
        jointPaint = new Paint();
        jointPaint.setColor(Color.RED);
        jointPaint.setStyle(Paint.Style.FILL);
        jointPaint.setAntiAlias(true);

        linkPaint = new Paint();
        linkPaint.setColor(Color.BLUE);
        linkPaint.setStyle(Paint.Style.STROKE);
        linkPaint.setStrokeWidth(6f);
        linkPaint.setAntiAlias(true);

        textPaint = new Paint();
        textPaint.setColor(Color.BLACK);
        textPaint.setTextSize(30f);
        textPaint.setAntiAlias(true);
    }

    public void setJointPoses(ArrayList<se3group> poses)
    {
        this.jointPoses = poses;
    }

    @Override
    protected void onDraw(Canvas canvas)
    {
        RobotControlActivity.appendTitle("entering ondraw" + el, true);
        RobotControlActivity.appendTitle("scale is=" + scale + el, true);
        super.onDraw(canvas);
        canvas.drawColor(Color.WHITE);

        if (jointPoses == null || jointPoses.isEmpty())
        {
            canvas.drawText("No robot data", 100, 200, textPaint);
            return;
        }

        // Draw coordinate axes
        drawAxes(canvas);

        // Draw links and joints
        float prevX = centerX;
        float prevY = centerY;

        for (int i = 0; i < jointPoses.size(); i++)
        {
            se3group pose = jointPoses.get(i);
            if (pose == null) continue;

            Vector3 position = pose.getPosition();
            float[] screenPos = projectToIsometric(position);

            float x = screenPos[0] + centerX;
            float y = screenPos[1] + centerY;

            // Draw link from previous joint
            if (i > 0)
            {
                canvas.drawLine(prevX, prevY, x, y, linkPaint);
            }
            else
            {
                // Draw base link
                canvas.drawLine(centerX, centerY, x, y, linkPaint);
            }

            // Draw joint (sphere)
            canvas.drawCircle(x, y, 20f, jointPaint);

            // Draw joint label
            String label = "J" + i;
            canvas.drawText(label, x + 10, y - 10, textPaint);

            prevX = x;
            prevY = y;
        }

        // Draw end effector if available
        if (!jointPoses.isEmpty())
        {
            se3group lastPose = jointPoses.get(jointPoses.size() - 1);
            if (lastPose != null)
            {
                Vector3 endPos = lastPose.getPosition();
                float[] screenPos = projectToIsometric(endPos);
                float endX = screenPos[0] + centerX;
                float endY = screenPos[1] + centerY;

                Paint endPaint = new Paint();
                endPaint.setColor(Color.GREEN);
                endPaint.setStyle(Paint.Style.FILL);
                canvas.drawCircle(endX, endY, 25f, endPaint);
                canvas.drawText("End Effector", endX + 10, endY + 10, textPaint);
            }
        }
    }

    private float[] projectToIsometric(Vector3 pos)
    {
        double x = pos.x * scale;
        double y = pos.y * scale;
        double z = pos.z * scale;

        // Isometric projection matrix
        float screenX = (float)((x * Math.cos(ISO_ANGLE_Z) - y * Math.sin(ISO_ANGLE_Z)) * Math.cos(ISO_ANGLE_X));
        float screenY = (float)(-(x * Math.sin(ISO_ANGLE_Z) + y * Math.cos(ISO_ANGLE_Z)) * Math.sin(ISO_ANGLE_X) - z);

        return new float[]{screenX, screenY};
    }

    private float[] calculateBoundingBox(ArrayList<se3group> poses)
    {
        if (poses == null || poses.isEmpty()) return null;

        float minX = Float.MAX_VALUE, maxX = Float.MIN_VALUE;
        float minY = Float.MAX_VALUE, maxY = Float.MIN_VALUE;
        float minZ = Float.MAX_VALUE, maxZ = Float.MIN_VALUE;

        for (se3group pose : poses)
        {
            if (pose == null) continue;
            Vector3 pos = pose.getPosition();
            minX = Math.min(minX, (float)pos.x);
            maxX = Math.max(maxX, (float)pos.x);
            minY = Math.min(minY, (float)pos.y);
            maxY = Math.max(maxY, (float)pos.y);
            minZ = Math.min(minZ, (float)pos.z);
            maxZ = Math.max(maxZ, (float)pos.z);
        }

        return new float[]{minX, maxX, minY, maxY, minZ, maxZ};
    }

    public void zoomToFit()
    {
        try
        {
            RobotControlActivity.appendTitle(el + "entering zoomtofit\n", true);
            RobotControlActivity.appendLog("bounds=null"+(jointPoses==null)+el+jointPoses.isEmpty()+el,true);
            if (jointPoses == null || jointPoses.isEmpty()) return;

            // 1. Calculate 3D bounding box of the robot
            float[] bounds = calculateBoundingBox(jointPoses);
            RobotControlActivity.appendLog("bounds=null"+(bounds==null)+el,true);
            if (bounds == null) return;

            float minX = bounds[0], maxX = bounds[1];
            float minY = bounds[2], maxY = bounds[3];
            float minZ = bounds[4], maxZ = bounds[5];

            // 2. Create the 8 corner points of the 3D box
            Vector3[] corners = new Vector3[8];
            corners[0] = new Vector3(minX, minY, minZ);
            corners[1] = new Vector3(maxX, minY, minZ);
            corners[2] = new Vector3(minX, maxY, minZ);
            corners[3] = new Vector3(maxX, maxY, minZ);
            corners[4] = new Vector3(minX, minY, maxZ);
            corners[5] = new Vector3(maxX, minY, maxZ);
            corners[6] = new Vector3(minX, maxY, maxZ);
            corners[7] = new Vector3(maxX, maxY, maxZ);

            // 3. Project all 8 corners to screen space using temp scale = 1.0f
            float minScreenX = Float.MAX_VALUE, maxScreenX = Float.MIN_VALUE;
            float minScreenY = Float.MAX_VALUE, maxScreenY = Float.MIN_VALUE;

            for (Vector3 corner : corners)
            {
                float[] screenPos = projectToIsometricWithScale(corner, 1.0f);
                minScreenX = Math.min(minScreenX, screenPos[0]);
                maxScreenX = Math.max(maxScreenX, screenPos[0]);
                minScreenY = Math.min(minScreenY, screenPos[1]);
                maxScreenY = Math.max(maxScreenY, screenPos[1]);
            }

            // 4. Calculate screen-space dimensions
            float robotWidth = maxScreenX - minScreenX;
            float robotHeight = maxScreenY - minScreenY;

            // Prevent division by zero if robot is a single point
            if (robotWidth < 0.001f) robotWidth = 0.1f;
            if (robotHeight < 0.001f) robotHeight = 0.1f;

            // 5. Get view dimensions
            int viewWidth = getWidth();
            int viewHeight = getHeight();
            RobotControlActivity.appendLog("view width="+viewWidth+el+"view height="+viewHeight+el,true);
            if (viewWidth == 0 || viewHeight == 0) return;

            // 6. Calculate new scale (with 20% padding)
            float padding = 0.8f;
            RobotControlActivity.appendLog("before computing scale"+el+robotWidth+el+robotHeight+el,true);
            float scaleX = (viewWidth * padding) / robotWidth;
            float scaleY = (viewHeight * padding) / robotHeight;
            scale = Math.min(scaleX, scaleY);
            RobotControlActivity.appendLog("scale is=" + scale + el, true);
            // Safety clamp
            if (Float.isInfinite(scale) || Float.isNaN(scale) || scale <= 0)
            {
                scale = 1000f;
            }

            // 7. Calculate center offsets to perfectly center the robot
            // Find the center of the projected bounding box
            float midScreenX = (minScreenX + maxScreenX) / 2f;
            float midScreenY = (minScreenY + maxScreenY) / 2f;

            // Calculate where centerX/Y should be so the robot's center maps to screen center
            centerX = (viewWidth / 2f) - (midScreenX * scale);
            centerY = (viewHeight / 2f) - (midScreenY * scale);

            // Force a redraw
            invalidate();
        }
        catch (Exception e)
        {
            StringWriter sw = new StringWriter();
            PrintWriter pw = new PrintWriter(sw);
            e.printStackTrace(pw);
            String stackTrace = sw.toString();

            String log="";
            log += "Exception: " + e.getMessage() + " from IKBody\n";
            log += "Stack trace:\n" + stackTrace + "\n";
            log += "error in build m0i array" + el;
            RobotControlActivity.appendTitle(log,true);
            RobotControlActivity activity = (RobotControlActivity) getContext();
            activity.updateLogDisplay();
		}
    }

// Helper method: project using a specific temp scale
    private float[] projectToIsometricWithScale(Vector3 pos, float s)
    {
        double x = pos.x * s;
        double y = pos.y * s;
        double z = pos.z * s;

        float screenX = (float)((x * Math.cos(ISO_ANGLE_Z) - y * Math.sin(ISO_ANGLE_Z)) * Math.cos(ISO_ANGLE_X));
        float screenY = (float)(-(x * Math.sin(ISO_ANGLE_Z) + y * Math.cos(ISO_ANGLE_Z)) * Math.sin(ISO_ANGLE_X) - z);

        return new float[]{screenX, screenY};
    }



    private void drawAxes(Canvas canvas)
    {
        Paint axisPaint = new Paint();
        axisPaint.setStrokeWidth(3f);
        axisPaint.setAntiAlias(true);

        // X axis (red)
        axisPaint.setColor(Color.RED);
        float[] xEnd = projectToIsometric(new Vector3(0.5f, 0, 0));
        canvas.drawLine(centerX, centerY, centerX + xEnd[0], centerY + xEnd[1], axisPaint);
        canvas.drawText("X", centerX + xEnd[0], centerY + xEnd[1] - 10, textPaint);

        // Y axis (green)
        axisPaint.setColor(Color.GREEN);
        float[] yEnd = projectToIsometric(new Vector3(0, 0.5f, 0));
        canvas.drawLine(centerX, centerY, centerX + yEnd[0], centerY + yEnd[1], axisPaint);
        canvas.drawText("Y", centerX + yEnd[0], centerY + yEnd[1] - 10, textPaint);

        // Z axis (blue)
        axisPaint.setColor(Color.BLUE);
        float[] zEnd = projectToIsometric(new Vector3(0, 0, 0.5f));
        canvas.drawLine(centerX, centerY, centerX + zEnd[0], centerY + zEnd[1], axisPaint);
        canvas.drawText("Z", centerX + zEnd[0], centerY + zEnd[1] - 10, textPaint);
    }
}
