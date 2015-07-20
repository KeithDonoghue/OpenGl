#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;



int main(int argc, char** argv)
{
 
	Mat image = imread("maxresdefault.jpg", -1);
	Mat channels[4];
	Mat images = image.clone();
	split(images,channels);
	//Mat image1 = Mat::ones(image.rows, image.cols,CV_8U);
	cvtColor(images,images,CV_BGR2GRAY);
	threshold(images,images,100,1,THRESH_BINARY);
	imshow("source", images);
	channels[3] = images;
	waitKey();	
	Mat image2;
	merge(channels,4,image2);
	for(int i = 0 ; i <  4; i++){
	imshow("source", channels[i]);
	waitKey();
	}
	imwrite("alpha_explosion.png", image2);
	waitKey();
	return 0;
}




