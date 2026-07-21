
import android.view.View;
import com.roboviz.Robot;
import java.util.List;
import com.math.se3group;
import android.graphics.Paint;
import com.math.Matrix;
import android.graphics.PointF;
import android.graphics.Canvas;
import android.content.Context;
import android.graphics.Color;
import com.math.Vector3;
// RobotVisualizationView.java
public class RobotVisualizationView extends View {
    
    private Robot robot;
    private List<se3group> jointTransforms;
    private Paint jointPaint, linkPaint, endEffectorPaint;
    private Matrix viewMatrix;
    private float scale = 100f; // pixels per meter
    private PointF offset = new PointF(500, 500);

    public RobotVisualizationView(Context context, Robot robot) {
        super(context);
        this.robot = robot;
        initPaints();
    }

    private void initPaints() {
        jointPaint = new Paint();
        jointPaint.setColor(Color.RED);
        jointPaint.setStyle(Paint.Style.FILL);

        linkPaint = new Paint();
        linkPaint.setColor(Color.BLUE);
        linkPaint.setStrokeWidth(4);

        endEffectorPaint = new Paint();
        endEffectorPaint.setColor(Color.GREEN);
        endEffectorPaint.setStyle(Paint.Style.FILL);
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        drawRobot(canvas);
    }

    private void drawRobot(Canvas canvas) {
        if (robot == null || robot.M0i.isEmpty()) return;

        canvas.translate(offset.x, offset.y);
        canvas.scale(scale, -scale); // Flip Y for robot coordinates

        // Draw each link
        for (int i = 0; i < robot.M0i.size() - 1; i++) {
            se3group T0i = robot.M0i.get(i);
            se3group T0i_next = robot.M0i.get(i + 1);

            Vector3 pos1 = T0i.getPosition();
            Vector3 pos2 = T0i_next.getPosition();

            // Draw link as line
            canvas.drawLine(f(pos1.x),f( pos1.y), f(pos2.x), f(pos2.y), linkPaint);

            // Draw joint as circle
            canvas.drawCircle(f(pos1.x), f(pos1.y), 0.1f, jointPaint);
        }

        // Draw end-effector
        se3group T_end = robot.M0i.get(robot.M0i.size() - 1);
        Vector3 endPos = T_end.getPosition();
        canvas.drawCircle(f(endPos.x), f(endPos.y), 0.15f, endEffectorPaint);
    }

    public void updateRobot(Robot robot) {
        this.robot = robot;
        invalidate(); // Redraw
    }
    private float f(double x){
        return (float)x;
    }
}
