#include<iostream>
#include<ros/ros.h>
#include<rosbag/bag.h>
#include<rosbag/view.h>
#include<sensor_msgs/Image.h>
#include<std_msgs/Time.h>
#include<std_msgs/Header.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose.h>

#include "Thirdparty/DLib/FileFunctions.h"
#include <fstream>


using namespace std;
using namespace cv;

//Exampleto run
//rosrun BagFromImages stereo_ImageToBag /disk/BackupComputer01July2019/home/francois/Documents/dataset/KITTIDATA/sequences/00/image_0 /disk/BackupComputer01July2019/home/francois/Documents/dataset/KITTIDATA/sequences/00/image_1 /disk/BackupComputer01July2019/home/francois/Documents/dataset/KITTIDATA/poses/00.txt 10 bag1.bag

//fct
Mat ReadTxtToMat(string path);
Mat RT2Proj(Mat R, Mat T);
void VecKITTI2RT(Mat VecKitti, Mat &R, Mat &T);
void getQuaternion(Mat R, double Q[]);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "BagFromImages");

    if(argc!=6)
    {
        cerr << "Usage: rosrun BagFromImages BagFromImages <path to left> <path to right> <path to pose> <frequency> <path to output bag>" << endl;
        return 0;
    }

    ros::start();

    // Open txt
    string PathToPose = argv[3];
    Mat PosesM = ReadTxtToMat(PathToPose);
    //Mat PosesM = ReadTxtToMat("/disk/BackupComputer01July2019/home/francois/Documents/dataset/KITTIDATA/poses/00.txt");
    

    // Frequency
    double freq = atof(argv[4]);

    // Output bag
    rosbag::Bag bag_out(argv[5],rosbag::bagmode::Write);
    ros::Time t = ros::TIME_MIN;

    const float T=1.0f/freq;
    ros::Duration d(T);

    // Read image sequence
    string argL1 = argv[1]; 
    cout << argL1 << endl;
    vector<String> filenamesL;
    glob(argL1, filenamesL);
    string argR1 = argv[2];
    cout << argR1 << endl;
    vector<String> filenamesR;
    glob(argR1, filenamesR);
    Mat ImageL, ImageR;

    // main loop
    cout << "start" << endl;
     bool endOfSeq = false; int countIm = 0;
    while (!endOfSeq){

        // Open images
        ImageL = imread(filenamesL[countIm]); // Load
        ImageR = imread(filenamesR[countIm]);
        if(ImageL.empty()) { cout << "End of Sequence" << endl; endOfSeq = true; break;}
        //cvtColor(ImageL,ImageL,CV_BGR2GRAY); cvtColor(ImageR,ImageR,CV_BGR2GRAY); // Conver to Gray if needed but you also have to modify image format so be careful...
        
        if(!ros::ok())
            break;

        // Read the line for the pose
        Mat Current_Pose = PosesM.row(countIm);
        Mat Current_R, Current_T; 
        double Current_quat [4] = { 0,0,0,1 };
        VecKITTI2RT(Current_Pose, Current_R, Current_T);
        getQuaternion(Current_R, Current_quat);

        // Convert to ROS (Pose)
        geometry_msgs::Pose Pose_msg;
        Pose_msg.position.x = Current_T.at<double>(0); Pose_msg.position.y = Current_T.at<double>(1); Pose_msg.position.z = Current_T.at<double>(2); // Position
        Pose_msg.orientation.x = Current_quat[0];  Pose_msg.orientation.y = Current_quat[1]; Pose_msg.orientation.z = Current_quat[2]; Pose_msg.orientation.w = Current_quat[3]; 

        // Conver to ros (LEFT)
        cv_bridge::CvImage cvImageL;
        cvImageL.image = ImageL;
        cvImageL.encoding = sensor_msgs::image_encodings::BGR8;
        cvImageL.header.stamp = t;

        // Conver to ros (RIGHT)
        cv_bridge::CvImage cvImageR;
        cvImageR.image = ImageL;
        cvImageR.encoding = sensor_msgs::image_encodings::BGR8;
        cvImageR.header.stamp = t;

        //Write in bag
        bag_out.write("/camera_left/image_color",ros::Time(t),cvImageL.toImageMsg());
        bag_out.write("/camera_right/image_color",ros::Time(t),cvImageR.toImageMsg());
        bag_out.write("/Pose",ros::Time(t),Pose_msg);
        t+=d;

        // Update Image read
        countIm++;
        cout << countIm << endl;

    }


    /*int i = 0;
    std::string file_name;
    int x,y,w,h,dummy;
    while(infile >> file_name >> dummy >> x >> y >> w >> h)
    {
        if(!ros::ok())
            break;
        cv::Mat im = cv::imread(file_name,CV_LOAD_IMAGE_COLOR);
        cv_bridge::CvImage cvImage;
        cvImage.image = im;
        cvImage.encoding = sensor_msgs::image_encodings::BGR8;
        cvImage.header.stamp = t;
        bag_out.write("/camera/image_color",ros::Time(t),cvImage.toImageMsg());
        t+=d;
        cout << i << endl;
        i++;
    }

    bag_out.close();*/

    // Close and shutdown everything~
    bag_out.close();
    ros::shutdown();

    return 0;
}


// txt2mat
Mat ReadTxtToMat(string path)
{
    std::ifstream file(path);
    Mat Matrix_Data;
    std::string line;
    vector<vector<double>> Matrix_data;
    int cols; int rows = 0;
    while (std::getline(file, line, '\n')) {  
        std::istringstream stream(line);

        char sep; //comma!
        double x;
        // read *both* a number and a comma:
        vector<double> TempVec;
        while (stream >> x) {
            TempVec.push_back(x);
        }
        Matrix_data.push_back(TempVec);
        rows++;
        cols = TempVec.size();
    }
    
    //Convert to matrix
    Mat m = Mat(rows, cols, CV_64F); // initialize matrix of uchar of 1-channel where you will store vec data
    for (int i=0; i<rows;i++) 
    {
        for (int j=0; j<cols;j++) 
        {
            //cout << Matrix_data[i][j] << endl;
            m.at<double>(i,j)=Matrix_data[i][j];
        }
    }

    return m;
}


Mat RT2Proj(Mat R, Mat T)
{
Mat Proj = (Mat_<double>(4,4) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);
R.copyTo(Proj(cv::Range(0,3), cv::Range(0,3)));
T.copyTo(Proj(Range(0, 3), Range(3,4)));
return Proj;
}

void VecKITTI2RT(Mat VecKitti, Mat &R, Mat &T)
{
 R = (Mat_<double>(3,3) << VecKitti.at<double>(0), VecKitti.at<double>(1), VecKitti.at<double>(2), VecKitti.at<double>(4), VecKitti.at<double>(5), VecKitti.at<double>(6), VecKitti.at<double>(8), VecKitti.at<double>(9), VecKitti.at<double>(10));
 T = (Mat_<double>(1,3) << VecKitti.at<double>(3), VecKitti.at<double>(7), VecKitti.at<double>(11));
}


// Not mine :: https://gist.github.com/shubh-agrawal/76754b9bfb0f4143819dbd146d15d4c8
void getQuaternion(Mat R, double Q[])
{
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
 
    if (trace > 0.0) 
    {
        double s = sqrt(trace + 1.0);
        Q[3] = (s * 0.5);
        s = 0.5 / s;
        Q[0] = ((R.at<double>(2,1) - R.at<double>(1,2)) * s);
        Q[1] = ((R.at<double>(0,2) - R.at<double>(2,0)) * s);
        Q[2] = ((R.at<double>(1,0) - R.at<double>(0,1)) * s);
    } 
    
    else 
    {
        int i = R.at<double>(0,0) < R.at<double>(1,1) ? (R.at<double>(1,1) < R.at<double>(2,2) ? 2 : 1) : (R.at<double>(0,0) < R.at<double>(2,2) ? 2 : 0); 
        int j = (i + 1) % 3;  
        int k = (i + 2) % 3;

        double s = sqrt(R.at<double>(i, i) - R.at<double>(j,j) - R.at<double>(k,k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;

        Q[3] = (R.at<double>(k,j) - R.at<double>(j,k)) * s;
        Q[j] = (R.at<double>(j,i) + R.at<double>(i,j)) * s;
        Q[k] = (R.at<double>(k,i) + R.at<double>(i,k)) * s;
    }
}