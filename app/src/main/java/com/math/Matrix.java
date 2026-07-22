package com.math;
import java.util.Locale;

public class Matrix
{
    protected int rows;
    protected int cols;
    protected double[][] data;

	public Matrix(double[][] Data)
	{
		this.rows = Data.length;
		this.cols = Data[0].length;
		this.data = new double[this.rows][this.cols];

		for (int i = 0; i < rows; i++)
		{
			System.arraycopy(Data[i], 0, this.data[i], 0, cols);
		}
	}
    public Matrix(int numRows,int numCols){
		this(Matrix.zeros(numRows,numCols).data);
	}
    // Core methods every matrix should have
    public  Matrix add(Matrix other)
	{
		Matrix added= this.clone();
		if (this.hasSameDimensions(other))
		{
			for (int i=0;i < this.rows;i++)
			{
				for (int j=0;j < this.cols;j++)
				{
					added.data[i][j] += other.data[i][j];
		        }
			}
			return added;
		}
		else
		{
			throw new IllegalArgumentException(
				"Matrices must have the same dimensions."
			);
		}
	}
    public  Matrix subtract(Matrix other)
	{
		Matrix subtracted= this.clone();
		if (this.hasSameDimensions(other))
		{
			for (int i=0;i < this.rows;i++)
			{
				for (int j=0;j < this.cols;j++)
				{
					subtracted.data[i][j] -= other.data[i][j];
		        }
			}
			return subtracted;
		}
		else
		{
			throw new IllegalArgumentException(
				"Matrices must have the same dimensions."
			);
		}
	}
	
	public static Matrix scalarMulti(double scale,Matrix org){
		Matrix mat = org.clone();
		for (int j=0;j < org.rows;j++)
		{
			for (int k=0;k < org.cols;k++)
			{
				mat.data[j][k] *= scale;
			}
		}
		return mat;
	}
	
   /* public  Matrix multiply(Matrix other)
	{
		if (this.cols == other.rows)
		{
			Matrix result = new Matrix(Matrix.zeros(this.rows, other.cols).data);
			for (int i=0;i < other.cols;i++)
			{
				for (int j=0;j < this.rows;j++)
				{
					for (int k=0;k < this.cols;k++)
					{
						result.data[j][i] += this.data[j][k] * other.data[k][i];
					}
				}
			}
			return result;
		}
		else
		{
			throw new IllegalArgumentException(
				"Matrix dimensions don't match for multiplication."
			);
		}
	}*/
    public Matrix multiply(Matrix other) {
        if (this.cols != other.rows) {
            throw new IllegalArgumentException(
                "Matrix dimensions don't match for multiplication."
            );
        }

        Matrix result = new Matrix(Matrix.zeros(this.rows, other.cols).data);

        // Correct loop order: rows of first matrix
        for (int i = 0; i < this.rows; i++) {        // Change: rows first
            for (int j = 0; j < other.cols; j++) {   // Then columns of second
                double sum = 0;
                for (int k = 0; k < this.cols; k++) {
                    sum += this.data[i][k] * other.data[k][j];
                }
                result.data[i][j] = sum;
            }
        }
        return result;
    }
    
    public Matrix transpose()
	{
		Matrix result = new Matrix(new double[cols][rows]);

		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++)
			{
				result.data[j][i] = data[i][j];
			}
		}

		return result;
	}
	public double trace(){
		double tr=0;
		if(isSquare()){
			for(int i=0;i<this.rows;i++){
				tr+=this.data[i][i];
			}
			return tr;
		}
		else{
			throw new IllegalArgumentException(
				"Matrix should be square"
			);
		}
	}
	public boolean isSymmetric() {
		return isSymmetric(1e-10);
	}

	public boolean isSymmetric(double tolerance) {
		if (!isSquare()) {
			return false;
		}

		for (int i = 0; i < this.rows; i++) {
			for (int j = i + 1; j < this.cols; j++) {
				if (Math.abs(this.data[i][j] - this.data[j][i]) > tolerance) {
					return false;
				}
			}
		}
		return true;
	}
	public boolean isSkewSymmetric() {
		return isSkewSymmetric(1e-6);
	}

	public boolean isSkewSymmetric(double tolerance) {
		if (!isSquare()) {
			return false;
		}

		for (int i = 0; i < this.rows; i++) {
			for (int j = i + 1; j < this.cols; j++) {
				// A[i][j] should equal -A[j][i]
				if (Math.abs(this.data[i][j] + this.data[j][i]) > tolerance) {
					return false;
				}
			}
		}

		// Diagonal should be zero (or very close)
		for (int i = 0; i < this.rows; i++) {
			if (Math.abs(this.data[i][i]) > tolerance) {
				return false;
			}
		}
		return true;
	}
    public double determinant()
	{
		if (!isSquare())
		{
			throw new IllegalArgumentException("Matrix must be square to compute determinant");
		}
		return determinantRecursive(this.data);
	}

	private double determinantRecursive(double[][] matrix)
	{
		int n = matrix.length;

		// Base cases
		if (n == 1)
		{
			return matrix[0][0];
		}
		if (n == 2)
		{
			return matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0];
		}

		double det = 0.0;
		int sign = 1;

		// Expand along first row
		for (int j = 0; j < n; j++)
		{
			double[][] minor = getMinor(matrix, 0, j);
			det += sign * matrix[0][j] * determinantRecursive(minor);
			sign = -sign; // Flip sign for next column
		}

		return det;
	}

	private double[][] getMinor(double[][] matrix, int row, int col)
	{
		int n = matrix.length;
		double[][] minor = new double[n - 1][n - 1];
		int r = 0;

		for (int i = 0; i < n; i++)
		{
			if (i == row) continue;
			int c = 0;
			for (int j = 0; j < n; j++)
			{
				if (j == col) continue;
				minor[r][c] = matrix[i][j];
				c++;
			}
			r++;
		}
		return minor;
	}
	public Matrix pseudoInverse(){
		return Matrix.identity(4);//to do change
	}
    public Matrix inverse()
	{
		if (!isSquare())
		{
			throw new IllegalArgumentException("Matrix must be square to invert");
		}

		double det = determinant();
		if (Math.abs(det) < 1e-10)
		{
			throw new ArithmeticException("Matrix is singular (determinant = 0)");
		}

		int n = this.rows;
		double[][] inverse = invertMatrix(this.data);
		return new Matrix(inverse);
	}

// Your provided invertMatrix method (I'll fix it to work with Matrix)
	private double[][] invertMatrix(double[][] A)
	{
		int n = A.length;
		double[][] M = new double[n][2 * n];

		// Create augmented matrix [A | I]
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				M[i][j] = A[i][j];
			}
			M[i][n + i] = 1.0; // Identity on right
		}

		// Gauss-Jordan elimination
		for (int col = 0; col < n; col++)
		{
			// Find pivot with partial pivoting
			int pivotRow = col;
			double maxVal = Math.abs(M[col][col]);
			for (int r = col + 1; r < n; r++)
			{
				if (Math.abs(M[r][col]) > maxVal)
				{
					maxVal = Math.abs(M[r][col]);
					pivotRow = r;
				}
			}

			if (maxVal < 1e-10)
			{
				throw new ArithmeticException("Matrix is singular");
			}

			// Swap rows
			double[] temp = M[col];
			M[col] = M[pivotRow];
			M[pivotRow] = temp;

			// Normalize pivot row
			double pivot = M[col][col];
			for (int j = 0; j < 2 * n; j++)
			{
				M[col][j] /= pivot;
			}

			// Eliminate all other rows
			for (int r = 0; r < n; r++)
			{
				if (r == col) continue;
				double factor = M[r][col];
				for (int j = 0; j < 2 * n; j++)
				{
					M[r][j] -= factor * M[col][j];
				}
			}
		}

		// Extract inverse from right half
		double[][] inverse = new double[n][n];
		for (int i = 0; i < n; i++)
		{
			for (int j = 0; j < n; j++)
			{
				inverse[i][j] = M[i][n + j];
			}
		}
		return inverse;
	}
    public Matrix clone()
	{
		double[][] copy = new double[rows][cols];

		for (int i = 0; i < rows; i++)
		{
			System.arraycopy(data[i], 0, copy[i], 0, cols);
		}

		return new Matrix(copy);
	}

    // Utility methods
	public static Matrix rand(int rows,int cols){
		double[][] data =new double[rows][cols];
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				data[i][j]=Math.random();
			}
			}
			return new Matrix(data);
	}
	
	/*public static Matrix blockMatrix(Matrix[][] blocks) {
		// 1. Calculate total rows and columns
		int totalRows = 0;
		for (int i = 0; i < blocks.length; i++) {
			totalRows += blocks[i][0].rows; // Sum up row heights of first column
		}

		int totalCols = 0;
		for (int j = 0; j < blocks[0].length; j++) {
			totalCols += blocks[0][j].cols; // Sum up column widths of first row
		}

		// 2. Create the big empty result matrix
		Matrix result = new Matrix(totalRows, totalCols);

		// 3. Copy each block into the correct position
		int currentRow = 0;
		for (int i = 0; i < blocks.length; i++) {
			int currentCol = 0;
			int blockHeight = blocks[i][0].rows; // Height of this row of blocks

			for (int j = 0; j < blocks[i].length; j++) {
				Matrix sub = blocks[i][j];
				int blockWidth = sub.cols;

				// Copy the sub-matrix into the big matrix
				Matrix flatten = blockToMatrix(blocks);
				result.setSubMatrix(flatten.data,currentRow, currentCol, sub.data);

				currentCol += blockWidth; // Move right
			}
			currentRow += blockHeight; // Move down
		}

		return result;
	}*/
    public static Matrix blockMatrix(Matrix[][] blocks) {
        // 1. Calculate total rows and columns
        int totalRows = 0;
        for (int i = 0; i < blocks.length; i++) {
            totalRows += blocks[i][0].rows; 
        }

        int totalCols = 0;
        for (int j = 0; j < blocks[0].length; j++) {
            totalCols += blocks[0][j].cols; 
        }

        // 2. Create the big empty result matrix
        Matrix result = new Matrix(totalRows, totalCols);

        // 3. Copy each block into the correct position
        int currentRow = 0;
        for (int i = 0; i < blocks.length; i++) {
            int currentCol = 0;
            int blockHeight = blocks[i][0].rows; 

            for (int j = 0; j < blocks[i].length; j++) {
                Matrix sub = blocks[i][j];
                int blockWidth = sub.cols;

                // --- FIX: REMOVE THE CALL TO blockToMatrix HERE ---
                // Just pass the raw data of the current 'sub' block directly
                result.setSubMatrix(sub.data, currentRow, currentCol, sub.data);

                currentCol += blockWidth; // Move right
            }
            currentRow += blockHeight; // Move down
        }

        return result;
    }
	public static Matrix blockToMatrix(Matrix[][] block) {
		if (block == null || block.length == 0) {
			throw new IllegalArgumentException("Block cannot be null or empty");
		}

		int rows = block.length;
		int cols = block[0].length;

		// Validate that all sub-matrices have consistent dimensions
		for (int i = 0; i < rows; i++) {
			for (int j = 0; j < cols; j++) {
				if (block[i][j] == null) {
					throw new IllegalArgumentException("Null matrix at position [" + i + "][" + j + "]");
				}
			}
		}

		// Check that rows in each block row have the same height
		for (int i = 0; i < rows; i++) {
			int expectedHeight = block[i][0].data.length;
			for (int j = 1; j < cols; j++) {
				if (block[i][j].data.length != expectedHeight) {
					throw new IllegalArgumentException("Inconsistent row heights in block row " + i);
				}
			}
		}

		// Check that columns in each block column have the same width
		for (int j = 0; j < cols; j++) {
			int expectedWidth = block[0][j].data[0].length;
			for (int i = 1; i < rows; i++) {
				if (block[i][j].data[0].length != expectedWidth) {
					throw new IllegalArgumentException("Inconsistent column widths in block column " + j);
				}
			}
		}

		// Calculate total dimensions
		int totalRows = 0;
		for (int i = 0; i < rows; i++) {
			totalRows += block[i][0].data.length;
		}

		int totalCols = 0;
		for (int j = 0; j < cols; j++) {
			totalCols += block[0][j].data[0].length;
		}

		// Build the combined matrix
		double[][] result = new double[totalRows][totalCols];

		int rowOffset = 0;
		for (int i = 0; i < rows; i++) {
			int colOffset = 0;
			int blockHeight = block[i][0].data.length;

			for (int j = 0; j < cols; j++) {
				Matrix subMatrix = block[i][j];
				int blockWidth = subMatrix.data[0].length;

				// Copy data
				for (int r = 0; r < blockHeight; r++) {
					System.arraycopy(subMatrix.data[r], 0, 
									 result[rowOffset + r], colOffset, blockWidth);
				}

				colOffset += blockWidth;
			}
			rowOffset += blockHeight;
		}

		return new Matrix(result);
	}
	public static void setSubMatrix(double[][] bigMatrix, int startRow, int startCol, double[][] subMatrix) {
		for (int i = 0; i < subMatrix.length; i++) {
			// Copies the entire row from 'subMatrix' into 'bigMatrix' in one go
			System.arraycopy(subMatrix[i], 0, bigMatrix[startRow + i], startCol, subMatrix[i].length);
		}
	}
	
	public Matrix getSubMatrix(int startRow, int startCol, int numRows, int numCols) {
		Matrix sub = new Matrix(numRows, numCols);
		for (int i = 0; i < numRows; i++) {
			for (int j = 0; j < numCols; j++) {
				sub.set(i, j, this.data[startRow + i][startCol + j]);
			}
		}
		return sub;
	}
	
    public double get(int row, int col)
	{ return data[row][col]; }
    public void set(int row, int col, double value)
	{ data[row][col] = value; }
    public int getRows()
	{ return rows; }
    public int getCols()
	{ return cols; }
    public boolean isSquare()
	{ return rows == cols; }
	public boolean isEqual(Matrix mat){
		if(hasSameDimensions(mat)){
			double epsilon = 0.000001; // A tiny tolerance number
			boolean areEqual = true;
			for (int i = 0; i < this.data.length; i++) {
				for (int j = 0; j < this.data[i].length; j++) {
					// Check if the difference is smaller than the tolerance
					if (Math.abs(this.data[i][j] - mat.data[i][j]) > epsilon) {
						areEqual = false;
						break;
					}
				}
			}
			return areEqual;
		}
		else{
			return false;
		}
	}
	
	public static boolean isEqual(Matrix mat,Matrix mat2){
		if(hasSameDimensions(mat,mat2)){
			double epsilon = 0.000001; // A tiny tolerance number
			boolean areEqual = true;
			for (int i = 0; i < mat.data.length; i++) {
				for (int j = 0; j < mat.data[i].length; j++) {
					// Check if the difference is smaller than the tolerance
					if (Math.abs(mat2.data[i][j] - mat.data[i][j]) > epsilon) {
						areEqual = false;
						break;
					}
				}
			}
			return areEqual;
		}
		else{
			return false;
		}
	}
	
	public boolean hasSameDimensions(Matrix other)
	{
		if (this.rows == other.rows && this.cols == other.cols)
			return true;
		return false;
	}
	
	public static boolean hasSameDimensions(Matrix first,Matrix second)
	{
		if (first.rows == second.rows && first.cols == second.cols)
			return true;
		return false;
	}

    // Static factory methods
    public static Matrix identity(int size)
	{
		double[][] identity = new double[size][size];
		for (int i=0;i < size;i++)
		{
			identity[i][i] = 1;
		}
		return new Matrix(identity);
	}
    public static Matrix zeros(int rows, int cols)
	{
		double[][] zeros = new double[rows][cols];
		return new Matrix(zeros);
	}
    public static Matrix ones(int rows, int cols)
	{
		double[][] ones = new double[rows][cols];
		for (int i=0;i < rows;i++)
		{
			for (int j=0;j < cols;j++)
			{
				ones[i][j] = 1;
			}
		}
		return new Matrix(ones);
	}
	
	public double normFrobenius() {
        double sum = 0;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                sum += data[i][j] * data[i][j];
            }
        }
        return Math.sqrt(sum);
    }
	
	public double normMatrix1() {
        if (rows == 0) return 0;
        double maxColSum = 0;
        for (int j = 0; j < cols; j++) {
            double colSum = 0;
            for (int i = 0; i < rows; i++) {
                colSum += Math.abs(data[i][j]);
            }
            maxColSum = Math.max(maxColSum, colSum);
        }
        return maxColSum;
    }
	
	public double normMatrixInfinity() {
        if (cols == 0) return 0;
        double maxRowSum = 0;
        for (int i = 0; i < rows; i++) {
            double rowSum = 0;
            for (int j = 0; j < cols; j++) {
                rowSum += Math.abs(data[i][j]);
            }
            maxRowSum = Math.max(maxRowSum, rowSum);
        }
        return maxRowSum;
    }
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();

        // Find the maximum width needed for any element (for alignment)
        int maxWidth = 0;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                String value = String.format(Locale.US,"%.4f", data[i][j]);
                maxWidth = Math.max(maxWidth, value.length());
            }
        }

        // Build the string representation
        for (int i = 0; i < rows; i++) {
            sb.append("[ ");
            for (int j = 0; j < cols; j++) {
                // Format with 4 decimal places and align right
                String value = String.format(Locale.US,"%" + maxWidth + ".4f", data[i][j]);
                sb.append(value);
                if (j < cols - 1) {
                    sb.append("  ");
                }
            }
            sb.append(" ]");
            if (i < rows - 1) {
                sb.append("\n");
            }
        }

        return sb.toString();
    }

}
