#include <stdio.h>
#include <stdlib.h>

#include "opencv_read_evd_func.h"

void printUsage (const char* progName){
  std::cout << "\n\nUsage: "<<progName<<" [options]\n\n"
            << "Options:\n"
            << "-------------------------------------------\n"
            << "-h           this help\n"
            << "-s           Simple visualisation example\n"
            << "-r           RGB colour visualisation example\n"
            << "-c           Custom colour visualisation example\n"
            << "-n           Normals visualisation example\n"
            << "-a           Shapes visualisation example\n"
            << "-v           Viewports example\n"
            << "-i           Interaction Customization example\n"
            << "-f           Load Sensor Data File\n"
            << "\n\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> SensorVisualizer(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Sensor 3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sensor cloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sensor cloud");
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();
      return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals){
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud){
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
                                     cloud->points[cloud->size() - 1], "line");
  viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  viewer->addPlane (coeffs, "plane");
  coeffs.values.clear ();
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.3);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (1.0);
  coeffs.values.push_back (0.0);
  coeffs.values.push_back (5.0);
  viewer->addCone (coeffs, "cone");

  return (viewer);
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2){
  // --------------------------------------------------------
  // -----Open 3D viewer and add point cloud and normals-----
  // --------------------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->initCameraParameters ();

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor (0, 0, 0, v1);
  viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
  viewer->addCoordinateSystem (1.0);

  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
  viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

  return (viewer);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ConvertPointCloud2PCL( std::vector<BlensorData> blensor_input_vec, int r, int g, int b ){
    int i;
    pcl::PointXYZRGB point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
    for( i = 0 ; i < blensor_input_vec.size() ; i++ ){
        point.x = blensor_input_vec[i].x;
        point.y = blensor_input_vec[i].y;
        point.z = blensor_input_vec[i].z;
        point.r = r;
        point.g = g;
        point.b = b;
        output->points.push_back(point);
    }
    output->width = (int)output->points.size();
    output->height = 1;

    return(output);
}

std::vector<BlensorData> TransformBlensorData( std::vector<BlensorData> blensor_input_vec, Eigen::Affine3f transform ){
    int i;
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr source_noisy_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_noisy_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ source_point, noisy_source_point;
    std::vector<BlensorData> res_vec;
    BlensorData res;

    for( i = 0 ; i < blensor_input_vec.size() ; i++ ){
        source_point.x = blensor_input_vec[i].x;
        source_point.y = blensor_input_vec[i].y;
        source_point.z = blensor_input_vec[i].z;
        noisy_source_point.x = blensor_input_vec[i].noisy_x;
        noisy_source_point.y = blensor_input_vec[i].noisy_y;
        noisy_source_point.z = blensor_input_vec[i].noisy_z;

        source_cloud_ptr->points.push_back(source_point);
        source_noisy_cloud_ptr->points.push_back(noisy_source_point);
    }
    pcl::transformPointCloud(*source_cloud_ptr, *transformed_cloud_ptr, transform);
    pcl::transformPointCloud(*source_noisy_cloud_ptr, *transformed_noisy_cloud_ptr, transform);
    
    for( i = 0 ; i < blensor_input_vec.size() ; i++ ){
        res = blensor_input_vec[i];
        res.x = transformed_cloud_ptr->points[i].x;
        res.y = transformed_cloud_ptr->points[i].y;
        res.z = transformed_cloud_ptr->points[i].z;
        res.noisy_x = transformed_noisy_cloud_ptr->points[i].x;
        res.noisy_y = transformed_noisy_cloud_ptr->points[i].y;
        res.noisy_z = transformed_noisy_cloud_ptr->points[i].z;
        res_vec.push_back(res);
    }
    return (res_vec);
}