#include <stdio.h>
void print_triangle(int n){
	for(int i = 0; i<n; i++){
		for(int j = 0; j<2*n -1; j++){
		    if( ((n - i) < j)   &&  (j < (n + i)))  // This problem is quiet simple, all you need is a little math
			{
				printf("#");
			}
		
		    else{
		    	printf(" ");
			}
	}
	printf("\n");
}
}
int main(){
	print_triangle(50);
	return 0;
}
