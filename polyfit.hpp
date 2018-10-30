#include <vector>
#include <cv.h>
#include <unistd.h>
#include <highgui.h>
#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;

vector<double> polyfit(vector<Point> vec_p, int degree){
	int n = degree;
	double X[2*n+1];
    int size = vec_p.size();
	for(int i = 0; i<2*n+1; i++){
		X[i]=0;
		for(int j = 0; j<size; j++){
			X[i]=X[i]+pow(vec_p[j].x, i);
		}
	}
	double B[n+1][n+2], a[n+1];
	for(int i = 0; i<=n; i++){
		for(int j = 0; j<=n; j++){
			B[i][j]=X[i+j];
		}
	}
	double Y[n+1];
	for(int i = 0; i<n+1; i++){
		Y[i] = 0;
		for(int j = 0; j < size; j++){
			Y[i]=Y[i]+pow(vec_p[j].x, i)*vec_p[j].y;
		}
	}
	for (int i = 0; i<=n; i++){
        B[i][n+1]=Y[i];
	}                
    n=n+1;    
	for(int i = 0; i<n; i++){
		for(int k = i+1; k < n; k++){
			if(B[i][i] < B[k][i]){
				for(int j=0; j<=n; j++){
					double temp = B[i][j];
					B[i][j] = B[k][j];
					B[k][j] = temp;
				}
			}
		}
	}
	for(int i = 0; i<n-1; i++){
		for(int k = i+1; k<n; k++){
			double t = B[k][i]/B[i][i];
			for (int j = 0; j <= n; j++){
				B[k][j] = B[k][j] - t*B[i][j];
			}
		}
	} 
	for(int i = n-1; i >= 0; i--){
		a[i] = B[i][n];
		for(int j = 0; j<n; j++){
			if(j != i){
				a[i] = a[i] - B[i][j] * a[j];
			}
		}
		a[i] = a[i] / B[i][i];
	}
	for(int i = n-1; i >= 0; i--){
		printf("%fx^%d", a[i], i);
		if(i != 0){
			printf(" + ");
		}
		else{
			printf("\n");
		}
	}
	vector<double> result;
	for(int i = 0; i < n; i++){
		result.push_back(a[i]);
	}
	return result;
}	