#include <stdio.h>
float vector_mul(float *a, float*b, int size){
	float temp = 0;
	for(int i = 0; i<size; i++){
		temp += a[i]*b[i]; // This problem is not hard too, nothing too special
	}
    return temp;
}
int main(){
	float a[3] = {3.5,5,6};
	float b[3] = {1,2,3.4};
	
	float* v1 = a;
	float* v2 = a;
	float result = vector_mul(v1, v2, 3);
	printf("%f", result);
	return 0;
}
