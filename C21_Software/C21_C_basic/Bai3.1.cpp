#include <stdio.h>
#include <stdlib.h>
// Return an array from function is not easy, that is the point of this example
// There is serveral way to do this, by using pointer or create a struct typedef
// This is the example of using a pointer for 2d array.
// To multipy 2 matrix, the row of the first matrix must equal to the column of 
// the second matrix, so we use m to represent that number, n to represent first matrix
// column and k for second matrix row

struct Matrix{
	int row, col;
	float * data;
};
Matrix matrix_mul(Matrix a, Matrix b){
	Matrix temp;
	temp.data = (float *) calloc(b.row*a.col,(a.col*b.row)*sizeof(float));
	temp.row = b.row;
	temp.col = a.col;
    for(int i = 0; i < a.col; i++){
    	for(int j = 0; j < b.row; j++){
    		for(int k = 0; k < a.row; k++){
    			temp.data[i*b.row + j] += a.data[i*a.row + k]*b.data[k*b.row +j]; 
    		}
    	}
	}
   
    return temp;
}
int main(){
	
    Matrix a;
    Matrix b;
    Matrix c;
    float temp1[6] = {1,1,1,2,2,2};
    float temp2[6] = {4,4,5,5,6,6};
    a.row = 3;
    a.col = 2;
    a.data = temp1;
    b.row = 2;
    b.col = 3;
    b.data = temp2;
//	printf("%f", result);
    c = matrix_mul(a, b);
    for(int i = 0; i < 9; i++){
        printf("%f", c.data[i]);
	    printf("\n");
    } 
	return 0;
}
