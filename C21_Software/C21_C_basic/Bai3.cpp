#include <stdio.h>
#include <stdlib.h>
// Return an array from function is not easy, that is the point of this example
// There is serveral way to do this, by using pointer or create a struct typedef
// This is the example of using a pointer for 2d array.
// To multipy 2 matrix, the row of the first matrix must equal to the column of 
// the second matrix, so we use m to represent that number, n to represent first matrix
// column and l for second matrix row
float * matrix_mul(float a[][3], float b[][2], int m, int n, int l){
	float * temp_data = (float *) calloc(n*l,(n*l)*sizeof(float)); // This is a pointer to the heap section of memory, I use calloc to allocate n*l data for the 
	                                                               // function to return
    for(int i = 0; i < n; i++){
    	for(int j = 0; j < l; j++){
    		for(int k = 0; k < m; k++){
    			temp_data[i*n+ j] += a[i][k]*b[k][j];   
    		}
    	}
	}
    return temp_data;
}
int main(){
	float a[2][3] = {{1,1,1}, {2,2,2}};
	float b[3][2] = {{4,4}, {5,5}, {6,6}};
    static float *result = matrix_mul(a, b, 3, 2, 2);
    
//	printf("%f", result);
    
        for(int i = 0; i < 9; i++){
    
        	printf("%f", result[i]);
	  
	  printf("\n");
}
	free(result);
	return 0;
}
