package com.math;

public class Vector extends Matrix {
    // Constructor
    public Vector(int size) {
        super(size, 1);
    }

    public Vector(double[][] data) {
        super(data);
        if (getCols() != 1) {
            throw new IllegalArgumentException("Vector must have exactly 1 column");
        }
    }

    public Vector(double[] data) {
        super(data.length, 1);
        for (int i = 0; i < data.length; i++) {
            set(i, 0, data[i]);
        }
    }

    // Copy constructor
    public Vector(Vector other) {
        super(other.getData());
    }

    // Convenience methods
    public double get(int i) {
        return getData()[i][0];
    }

    public void set(int i, double value) {
        set(i, 0, value);
    }

    public int size() {
        return getRows();
    }

    // Override operations to return Vector when possible
    public Vector add(Vector other) {
        Vector added = new Vector(this.getRows());
        if (this.hasSameDimensions(other)) {
            for (int i = 0; i < this.rows; i++) {
                added.data[i][0] = this.data[i][0] + other.data[i][0];  // ← Add BOTH!
            }
            return added;
        }
        else {
            throw new IllegalArgumentException("Matrices must have the same dimensions.");
        }
    }

    public Vector subtract(Vector other) {
        Vector added = new Vector(this.getRows());
        if (this.hasSameDimensions(other)) {
            for (int i = 0; i < this.rows; i++) {
                added.data[i][0] = this.data[i][0] - other.data[i][0];  // ← Fix this too!
            }
            return added;
        }
        else {
            throw new IllegalArgumentException("Matrices must have the same dimensions.");
        }
    }

    public static Vector scalarMulti(double scale, Vector org) {
        Vector vec = new Vector(org.getRows());
        for (int j = 0; j < org.rows; j++) {
            vec.data[j][0] = org.data[j][0] * scale;  // ← Use assignment, not multiplication
        }
        return vec;
    }

    // Static factory methods
    public static Vector zeros(int size) {
        return new Vector(Matrix.zeros(size, 1).getData());
    }

    public static Vector ones(int size) {
        return new Vector(Matrix.ones(size, 1).getData());
    }
}
