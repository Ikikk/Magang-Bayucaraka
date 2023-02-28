#include <opencv2/opencv.hpp>

#include <iostream>
using namespace std;
using namespace cv;

int main() {
    int len;
    string approx;
    // read img
    img = mread("shape1.jpeg");
    
    // resize img
    int h = img.rows;
    int w = img.cols;
    resize(img, img, Size(w/2, h/2));
    
    // convert to grayscale
    // gray;
    cvtColor(img, gray, COLOR_BGR2GRAY);
    
    // detecting edges
    // edges;
    Canny(gray, edges, 170, 255);
    
    // take areas with more intensity
    thresh;
    threshold(gray, thresh, 240, 255, THRESH_BINARY);
    
    // get all contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
    
    // for each contour
    for (int i = 0; i < contours.size(); i++)
    {
        // get length from each contour
        double peri = cv::arcLength(contours[i], true);
        // get how many dots
        vector<Point> approx;
        approxPolyDP(contours[i], approx, 0.02 * peri, true);
    }

    len = approx.length();

    drawCountours(img, [contour], -1, (0,0,255), 2);

    imshow('dsucne', img);
    waitKey(0);
    destroyAllWindows();
}