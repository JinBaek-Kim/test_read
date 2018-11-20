#include <vector>
#include <opencv2/viz.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/transforms.h>

typedef struct blensor_data{
	double timestamp;
	double yaw;
	double pitch;
	double dist;
	double noisy_dist;
	double x;
	double y;
	double z;
	double noisy_x;
	double noisy_y;
	double noisy_z;
	double r;
	double g;
	double b;
	int64_t id;
}BlensorData;

typedef struct point{
	double x;
	double y;
	double z;
}Point;

typedef struct linesegment{
	Point start;
	Point end;
	Point direction; // end - start
	double length; // norm(direction)
}LineSegment;

typedef struct polygonsegment{
	Point centroid;
	Point normal;
	std::vector<Point> vertice;
	std::vector<LineSegment> edgeline;
}PolygonSegment;

typedef struct mapelements{
	std::vector<Point> points;
	std::vector<LineSegment> lines;
	std::vector<PolygonSegment> polygons;
}MapElements;

void printUsage (const char* progName);
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals);
boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2);

 // Customized by Jinbaek
boost::shared_ptr<pcl::visualization::PCLVisualizer> SensorVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvertPointCloud2PCL( std::vector<BlensorData> blensor_input_vec, int r, int g, int b );
std::vector<BlensorData> TransformBlensorData( std::vector<BlensorData> blensor_input_vec, Eigen::Affine3f transform );