// DUORosNode.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
//#include <opencv2/opencv.hpp>
//#include <opencv2/core/mat.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <DUOLib.h>
// Include Dense3D API header file
#include <Dense3DMT.h>
#include "sample.h"
#include "geometry_msgs/Point.h"
#include "sensor_msgs/ChannelFloat32.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>	
#include <pcl/filters/voxel_grid.h>

//#pragma comment(lib,"pcl_common_release.lib")
//#pragma comment(lib,"pcl_features_release.lib")
//#pragma comment(lib,"pcl_io_ply_release.lib")
//#pragma comment(lib,"pcl_io_release.lib")
//#pragma comment(lib,"pcl_kdtree_release.lib")
//#pragma comment(lib,"pcl_keypoints_release.lib")
//#pragma comment(lib,"pcl_octree_release.lib")
//#pragma comment(lib,"pcl_outofcore_release.lib")
//#pragma comment(lib,"pcl_people_release.lib")
//#pragma comment(lib,"pcl_recognition_release.lib")
//#pragma comment(lib,"pcl_registration_release.lib")
//#pragma comment(lib,"pcl_sample_consensus_release.lib")
//#pragma comment(lib,"pcl_search_release.lib")
//#pragma comment(lib,"pcl_segmentation_release.lib")
//#pragma comment(lib,"pcl_surface_release.lib")
//#pragma comment(lib,"pcl_tracking_release.lib")
//#pragma comment(lib,"pcl_visualization_release.lib")

//#include <pcl_ros/transforms.h>

#include <sensor_msgs/pointcloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <iostream>



using namespace std;

#define WIDTH	640
#define HEIGHT	480
#define FPS     30

ros::Publisher cloud_pub;
ros::Publisher cloud_pub2;

#define CLOUD2

namespace cl2 {

	class Vec
	{
	public:
		union
		{
			struct { double x, y, z; };
			double v_[3];
		};
	public:
		Vec() : x(0.0), y(0.0), z(0.0) {}
		Vec(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
		Vec(cv::Vec3b v) : x(v.val[0]), y(v.val[1]), z(v.val[2]) {}

		Vec& operator=(const Vec& v)
		{
			x = v.x;   y = v.y;   z = v.z;
			return *this;
		}
		double operator[](int i) const { return v_[i]; }
		double& operator[](int i) { return v_[i]; }
		operator const double*() const { return v_; }
		operator double*() { return v_; }

		friend Vec operator+(const Vec &a, const Vec &b)
		{
			return Vec(a.x + b.x, a.y + b.y, a.z + b.z);
		}
		friend Vec operator-(const Vec &a, const Vec &b)
		{
			return Vec(a.x - b.x, a.y - b.y, a.z - b.z);
		}
		friend Vec operator-(const Vec &a)
		{
			return Vec(-a.x, -a.y, -a.z);
		}
		friend Vec operator*(const Vec &a, double k)
		{
			return Vec(a.x * k, a.y * k, a.z * k);
		}
		friend Vec operator*(double k, const Vec &a)
		{
			return a * k;
		}
		friend Vec operator/(const Vec &a, double k)
		{
			return Vec(a.x / k, a.y / k, a.z / k);
		}
		friend bool operator!=(const Vec &a, const Vec &b)
		{
			return !(a == b);
		}
		friend bool operator==(const Vec &a, const Vec &b)
		{
			const double epsilon = 1.0E-10f;
			return (a - b).squaredNorm() < epsilon;
		}
		Vec& operator+=(const Vec &a)
		{
			x += a.x; y += a.y; z += a.z;
			return *this;
		}
		Vec& operator-=(const Vec &a)
		{
			x -= a.x; y -= a.y; z -= a.z;
			return *this;
		}
		Vec& operator*=(double k)
		{
			x *= k; y *= k; z *= k;
			return *this;
		}
		Vec& operator/=(double k)
		{
			x /= k; y /= k; z /= k;
			return *this;
		}
		friend double operator*(const Vec &a, const Vec &b)
		{
			return a.x * b.x + a.y * b.y + a.z * b.z;
		}
		friend Vec operator^(const Vec &a, const Vec &b)
		{
			return cross(a, b);
		}
		friend Vec cross(const Vec &a, const Vec &b)
		{
			return Vec(a.y * b.z - a.z * b.y,
				a.z * b.x - a.x * b.z,
				a.x * b.y - a.y * b.x);
		}
		double squaredNorm() const { return x * x + y * y + z * z; }
		double norm() const { return sqrt(x * x + y * y + z * z); }
		double normalize()
		{
			const double n = norm();
			*this /= n;
			return n;
		}
		void rotate(double angle, Vec axis)
		{
			double rad = angle * (3.14159265358979323846 / 180.0);
			double cc = cos(rad);
			double ss = sin(rad);
			double a = axis.x * axis.x + (1 - axis.x * axis.x) * cc;
			double b = axis.x * axis.y * (1 - cc) - axis.z * ss;
			double c = axis.x * axis.z * (1 - cc) + axis.y * ss;
			double d = axis.x * axis.y * (1 - cc) + axis.z * ss;
			double e = axis.y * axis.y + (1 - axis.y * axis.y) * cc;
			double f = axis.y * axis.z * (1 - cc) - axis.x * ss;
			double g = axis.x * axis.z * (1 - cc) - axis.y * ss;
			double h = axis.y * axis.z * (1 - cc) + axis.x * ss;
			double i = axis.z * axis.z + (1 - axis.z * axis.z) * cc;
			double nx = x * a + y * b + z * c;
			double ny = x * d + y * e + z * f;
			double nz = x * g + y * h + z * i;
			x = nx;
			y = ny;
			z = nz;
		}
		static double angle(Vec a, Vec c, Vec b)
		{
			double s = acos((a - c) * (c - b) / ((a - c).norm() * (c - b).norm()));
			return s * (180.0 / 3.14159265358979323846);
		}
	};

	class TrackballCamera
	{
		Vec _position, _lookAt, _forward, _up, _left;
		double _angleX;
	public:
		TrackballCamera() {}
		TrackballCamera(Vec position, Vec lookat)
		{
			_position = position;
			_lookAt = lookat;
			_angleX = 0.0;
			update();
		}
		void update()
		{
			_forward = _lookAt - _position;
			_left = Vec(_forward.z, 0, -_forward.x);
			_up = cross(_left, _forward);
			_forward.normalize();
			_left.normalize();
			_up.normalize();
		}
		void show()
		{
		/*	gluLookAt(_position.x, _position.y, _position.z,
				_lookAt.x, _lookAt.y, _lookAt.z,
				0.0, 1.0, 0.0);*/
		}
		void rotate(double pos, Vec v)
		{
			Vec prevPos = _position;
			translate(-_lookAt);
			_position.rotate(pos / 500.0, v);
			translate(_lookAt);
			updateAngleX();
			if (_angleX < 5 || _angleX > 175)
			{
				_position = prevPos;
				updateAngleX();
			}
		}
		void translate(Vec v)
		{
			_position += v;
		}
		void translateLookAt(Vec v)
		{
			_lookAt += v;
		}
		void translateAll(Vec v)
		{
			translate(v);
			translateLookAt(v);
		}
		void zoom(double z)
		{
			double dist = (_position - _lookAt).norm();
			if (dist - z > z)
				translate(_forward * z);
		}
		Vec getPosition() { return _position; }
		Vec getPositionFromLookAt() { return _position - _lookAt; }
		Vec getLookAt() { return _lookAt; }
		Vec getForward() { return _forward; }
		Vec getUp() { return _up; }
		Vec getLeft() { return _left; }
		void setPosition(Vec p) { _position = p; updateAngleX(); }
		void setLookAt(Vec p) { _lookAt = p; updateAngleX(); }
	private:
		void updateAngleX()
		{
			_angleX = Vec::angle(Vec(_position.x, _position.y + 1, _position.z), _position, _lookAt);
		}
	};

	struct PointXYZRGB
	{
		double x, y, z;
		double r, g, b;
		PointXYZRGB() : x(0), y(0), z(0), r(0), g(0), b(0) {}
		PointXYZRGB(double x_, double y_, double z_) : x(x_), y(y_), z(z_), r(0), g(0), b(0) {}
		PointXYZRGB(double x_, double y_, double z_, double r_, double g_, double b_) : x(x_), y(y_), z(z_), r(r_), g(g_), b(b_) {}
		PointXYZRGB(Vec v, double r_, double g_, double b_) : x(v.x), y(v.y), z(v.z), r(r_), g(g_), b(b_) {}
		PointXYZRGB(Vec v, double c) : x(v.x), y(v.y), z(v.z), r(c), g(c), b(c) {}
	};

#if 0
	void addData(const Mat1b &image, const Mat3f &depth)
	{
		_cloudMx.lock();
		_cloud.clear();
		for (int y = 0; y < image.rows; y++)
			for (int x = 0; x < image.cols; x++)
			{
				Point p = Point(x, y);
				double c = image.at<uchar>(p) / 255.0;
				if (c == 0) continue;
				Vec v(depth.at<Vec3f>(p)[0], depth.at<Vec3f>(p)[1], depth.at<Vec3f>(p)[2]);
				if (v.z < 10000)
				{
					v /= 1000.0;
					_cloud.push_back(PointXYZRGB(v, c));
				}
			}
		_cloudMx.unlock();
	}
#endif


}
using namespace cl;

//#define COLOR

void addData(const Mat1b &image, const Mat3f &depth)
{
	
	sensor_msgs::PointCloud ROSCloud;
	geometry_msgs::Point32 ROSPoint;
	sensor_msgs::ChannelFloat32 chan;

	ROSCloud.header.frame_id = "odom";
	ROSCloud.header.stamp = ros::Time::now();

	chan.name = "intensity";
	
	cl::Vec v;
	double c;

	for (int y = 0; y < image.rows; y++)
		for (int x = 0; x < image.cols; x++)
		{
		Point p = Point(x, y);
#ifdef COLOR
		Vec3b colvec = image.at<Vec3b>(p);
#else
		c = image.at<uchar>(p) / 255.0; //greyscale intensity is what this is. 
		if (c == 0) continue;
#endif
			

	
			//cl::Vec v(depth.at<Vec3f>(p)[0], depth.at<Vec3f>(p)[1], depth.at<Vec3f>(p)[2]);
			//convert pos to v3, retrieve 3 floats 
			//v.x = ((const Vec3f*)(depth.data + depth->data->step.p[0] * p.y))[p.x];
			v.x = depth.at<Vec3f>(p)[0];
			v.y = depth.at<Vec3f>(p)[1];
			v.z = depth.at<Vec3f>(p)[2];
			
			
			if (v.z < 1000)//was 10000
			{
				v /= 1000.0;
				//_cloud.push_back(PointXYZRGB(v, c));
#ifndef COLOR //if Greyasccale
				auto temp=PointXYZRGB(v, c);
#else
				uchar r=colvec[0] ;
				uchar g=colvec[1] ;
				uchar b=colvec[2] ;

				auto temp = PointXYZRGB(v, r,g,b);

#endif
				ROSPoint.x = temp.x;
				ROSPoint.y = temp.y*-1;
				ROSPoint.z = temp.z;

				if (temp.z > 10) continue;
				ROSCloud.points.push_back(ROSPoint);
				
//				c = RGB(110, 100, 100);
				chan.values.push_back(c);
		//		cout << "color:" << c << endl;
				
				
				
			}

		}

	//cout << "Point sent--" << endl;

	ROSCloud.channels.push_back(chan);
	
#ifndef CLOUD2
	cloud_pub.publish(ROSCloud);
#else

	

	sensor_msgs::PointCloud2 rosPointCloud2;
	boost::shared_ptr<pcl::PCLPointCloud2> pcl_pc2( new pcl::PCLPointCloud2 );	boost::shared_ptr<pcl::PCLPointCloud2> filtered(new pcl::PCLPointCloud2);	sensor_msgs::convertPointCloudToPointCloud2(ROSCloud, rosPointCloud2);	pcl_conversions::toPCL(rosPointCloud2, *pcl_pc2); //correct
		pcl::VoxelGrid<pcl::PCLPointCloud2> filter;//	//	//	pcl::PCLPointCloud2Ptr pc2ptr(new pcl::PCLPointCloud2);//////	sensor_msgs::convertPointCloudToPointCloud2(ROSCloud, pcl_pc2);
//
//	
	
//	pcl::PointCloud2::Ptr cloud
//	boost::shared_ptr<sensor_msgs::PointCloud2> ddd;
	//pcl::PCLPointCloud2Ptr xxx(new pcl::PCLPointCloud2Ptr());

//	pcl::VoxelGrid<sensor_msgs::PointCloud2> downsample;
	pcl::VoxelGrid<pcl::PCLPointCloud2> downsample;

	downsample.setInputCloud(pcl_pc2);

	//bigger leaves are, the bigger the octcube and courser the points.
#define DIMS 0.009f
	downsample.setLeafSize(DIMS, DIMS, DIMS);
	
	downsample.filter(*filtered);

	//ROS_INFO_ONCE_NAMED("interop", "size:%d\n", filtered->data.size());

	//printf("size:%d\n", filtered->data.size());
	
	cloud_pub2.publish(filtered);

//
//	
//	
//	cloud_pub2.publish(pcl_pc2);
//	return;
////	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//
//	
//	pcl::PointCloud< pcl::PointXYZ> * cloud= new pcl::PointCloud< pcl::PointXYZ>();
//
//	pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(cloud);//	filter.setInputCloud(cloudPtr);////	// We set the size of every voxel to be 1x1x1cm//	// (only one point per every cubic centimeter will survive).//	//filter.setLeafSize(0.01f, 0.01f, 0.01f);//	filter.setLeafSize(0.95f, 0.95f, 0.95f);//////	printf("size:%d", cloudPtr->size());////	filter.filter(*cloudPtr);//	cloud_pub2.publish(*cloudPtr);


#endif



	
}

#if 0
void createAndAddPoint2()
{
	int channel_size = 4; //x y z plus color
	sensor_msgs::PointCloud2 output;
		
	output.header.stamp = ros::Time::now();
	output.header.frame_id = "odom";
		
	//TODO: do this at the end once we know how many points
	//output.width = input.points.size();
		
	output.height = 1;
	output.fields.resize(4);//was (3+channel.size, but we will hardcode a 4th color channel as a float
	// Convert x/y/z to fields
	output.fields[0].name = "x"; output.fields[1].name = "y"; output.fields[2].name = "z";
	int offset = 0;
	// All offsets are *4, as all field data types are float32
	for (size_t d = 0; d < output.fields.size(); ++d, offset += 4)
	{
		output.fields[d].offset = offset;
		output.fields[d].datatype = sensor_msgs::PointField::FLOAT32;
		output.fields[d].count = 1;
	}
	output.point_step = offset;
	output.row_step = output.point_step * output.width;
	// Convert the remaining of the channels to fields
	
	//for (size_t d = 0; d < input.channels.size(); ++d)
	//for (size_t d = 0; d < channel_size; ++d)
		//output.fields[3 + d].name = input.channels[d].name;
	output.fields[3].name = "intensity";

	
	output.data.resize(input.points.size() * output.point_step);
	output.is_bigendian = false;  // @todo ?
	output.is_dense = false;

	// Copy the data points
	for (size_t cp = 0; cp < input.points.size(); ++cp)
	{
		memcpy(&output.data[cp * output.point_step + output.fields[0].offset], &input.points[cp].x, sizeof(float));
		memcpy(&output.data[cp * output.point_step + output.fields[1].offset], &input.points[cp].y, sizeof(float));
		memcpy(&output.data[cp * output.point_step + output.fields[2].offset], &input.points[cp].z, sizeof(float));
		for (size_t d = 0; d < input.channels.size(); ++d)
		{
			if (input.channels[d].values.size() == input.points.size())
			{
				memcpy(&output.data[cp * output.point_step + output.fields[3 + d].offset], &input.channels[d].values[cp], sizeof(float));
			}
		}
	}


}
#endif

Vec3b HSV2RGB(float hue, float sat, float val)
{
	float x, y, z;

	if (hue == 1) hue = 0;
	else         hue *= 6;

	int i = static_cast<int>(floorf(hue));
	float f = hue - i;
	float p = val * (1 - sat);
	float q = val * (1 - (sat * f));
	float t = val * (1 - (sat * (1 - f)));

	switch (i)
	{
	case 0: x = val; y = t; z = p; break;
	case 1: x = q; y = val; z = p; break;
	case 2: x = p; y = val; z = t; break;
	case 3: x = p; y = q; z = val; break;
	case 4: x = t; y = p; z = val; break;
	case 5: x = val; y = p; z = q; break;
	}
	return Vec3b((uchar)(x * 255), (uchar)(y * 255), (uchar)(z * 255));
}


int DUOGo(int argc, char* argv[])
{
#ifndef BOILERPLATE

	printf("Dense3D Point Cloud Program\n");

	// Build color lookup table for depth display
	Mat colorLut = Mat(cv::Size(256, 1), CV_8UC3);
	for (int i = 0; i < 256; i++)
		colorLut.at<Vec3b>(i) = (i == 0) ? Vec3b(0, 0, 0) : HSV2RGB(i / 256.0f, 1, 1);

	DUOInstance duo;
	if (!OpenDUO(&duo))
	{
		printf("Could not open DUO\n");
		return 1;
	}

	Dense3DMTInstance dense3d;
	if (!Dense3DOpen(&dense3d, duo))
	{
		printf("Could not open Dense3DMT\n");
		return 1;
	}
	if (!SetDense3DLicense(dense3d, "D9TU5-TZUDZ-TE72X-485QO-F9R4W")) // <-- Put your Dense3D license
	{
		printf("Invalid or missing Dense3D license. To get your license visit https://duo3d.com/account\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}

	char tmp[260];
	// Get some DUO parameter values
	GetDUODeviceName(duo, tmp);
	printf("DUO Device Name:      '%s'\n", tmp);
	GetDUOSerialNumber(duo, tmp);
	printf("DUO Serial Number:    %s\n", tmp);
	GetDUOFirmwareVersion(duo, tmp);
	printf("DUO Firmware Version: v%s\n", tmp);
	GetDUOFirmwareBuild(duo, tmp);
	printf("DUO Firmware Build:   %s\n", tmp);
	printf("Dense3D Version:      v%s\n", Dense3DGetLibVersion());

	if (!SetDense3DImageInfo(dense3d, WIDTH, HEIGHT, FPS))
	{
		printf("SetDense3DImageInfo error\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}

	Dense3DParams params;
	params.scale = 3;
	params.mode = 0;
	params.numDisparities = 2;
	params.sadWindowSize = 6;
	params.preFilterCap = 28;
	params.uniqenessRatio = 27;
	params.speckleWindowSize = 52;
	params.speckleRange = 14;
	if (!SetDense3Params(dense3d, params))
	{
		printf("GetDense3Params error\n");
		// Close Dense3D library
		Dense3DClose(dense3d);
		return 1;
	}

	// Queue used to receive Dense3D frames
	Dense3DFrameQueue d3dq;

	if (!Dense3DStart(dense3d, [](const PDense3DFrame pFrameData, void *pUserData)
	{
		D3DFrame frame;
		Size frameSize(pFrameData->duoFrame->width, pFrameData->duoFrame->height);
		frame.leftImg = Mat(frameSize, CV_8U, pFrameData->duoFrame->leftData);
		frame.rightImg = Mat(frameSize, CV_8U, pFrameData->duoFrame->rightData);
		frame.disparity = Mat(frameSize, CV_32F, pFrameData->disparityData);
		frame.depth = Mat(frameSize, CV_32FC3, pFrameData->depthData);
		((Dense3DFrameQueue*)pUserData)->push(frame);
	}, &d3dq))
	{
		printf("Dense3DStart error\n");
		return 1;
	}

	// Set exposure, LED brightness and camera orientation
	SetDUOExposure(duo, 85);
	SetDUOLedPWM(duo, 28);
	SetDUOVFlip(duo, true);

	DUOResolutionInfo ri;
	GetDUOResolutionInfo(duo, &ri);
	double fov[4];
	GetDUOFOV(duo, ri, fov);

	//CloudViewer viewer;
//	viewer.setFov(fov[0], fov[1]);

	// Run capture loop until <Esc> key is pressed
#endif

	char temp[255];



	while (1) //((cvWaitKey(1) & 0xff) != 27)
	{
		D3DFrame d3DFrame;
		if (!d3dq.pop(d3DFrame))
			continue;

		// Update point cloud
		addData(d3DFrame.leftImg, d3DFrame.depth);

		Mat disp8;
		Mat rgbBDisparity;

#ifdef COLOR
		cout << "1" << endl;
		d3DFrame.disparity.convertTo(disp8, CV_8UC3);//disp8 created here
		cout << "2" << endl;
	//	cvtColor(disp8, rgbBDisparity, COLOR_BGR2XYZ);
		cout << "3" << endl;
		LUT(disp8, colorLut, rgbBDisparity);
#else
		d3DFrame.disparity.convertTo(disp8, CV_8UC1, 255.0 / (params.numDisparities * 16));
		cvtColor(disp8, rgbBDisparity, COLOR_GRAY2BGR);
		LUT(rgbBDisparity, colorLut, rgbBDisparity);
#endif


		// Display images
		//imshow("Left Image", d3DFrame.leftImg);
		//imshow("Right Image", d3DFrame.rightImg);
		//imshow("Disparity Image", rgbBDisparity);

	//	std::cin >> temp;
//		cout << "loop\n";

	}
//	viewer.close();
	destroyAllWindows();

	Dense3DStop(dense3d);
	Dense3DClose(dense3d);
	return 0;
}



int main(int argc, char ** argv)
{
	ros::init(argc, argv, "DUOPointCloud");
	ros::NodeHandle n;

	

	cloud_pub = n.advertise<sensor_msgs::PointCloud>("pointcloud", 100);
	cloud_pub2 = n.advertise<sensor_msgs::PointCloud2>("pointcloud2", 100);

//	sensor_msgs::convertPointCloudToPointCloud2(*msg, output));


	//ros::Rate loop_rate(10);
	//int count = 0;
	//while (ros::ok())
	//{
	//	chatter_pub.publish(msg);

	//	ros::spinOnce();

//		loop_rate.sleep();

	//}
	//	Mat1b m;
	DUOGo(argc, argv);


    return 0;
}

