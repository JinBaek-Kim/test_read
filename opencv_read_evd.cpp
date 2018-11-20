#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include "opencv_read_evd_func.h"

unsigned int text_id = 0;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getKeySym () == "r" && event.keyDown ())
  {
    std::cout << "r was pressed => removing all text" << std::endl;

    char str[512];
    for (unsigned int i = 0; i < text_id; ++i)
    {
      sprintf (str, "text#%03d", i);
      viewer->removeShape (str);
    }
    text_id = 0;
  }
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event,
                         void* viewer_void)
{
  pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
  if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
      event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  {
    std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;

    char str[512];
    sprintf (str, "text#%03d", text_id ++);
    viewer->addText ("clicked here", event.getX (), event.getY (), str);
  }
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);

  viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
  viewer->registerMouseCallback (mouseEventOccurred, (void*)viewer.get ());

  return (viewer);
}

int main (int argc, char** argv){
    FILE *evd_File;
    long evd_size;
    double *temp, *display_point_cloud;
    int *init_tmp;
    int i, j, k;
    int hokuyo_scanning_num = 181;
    int hokuyo_scanpoint_num = 1080;
    int blensor_data_num = 15;
    int cur_data_pos, cur_scan_data_size;
    size_t read_size;
    std::vector<BlensorData> blensor_data_vec, transformed_blensor_data_vec, tmp_blensor_data_vec, tmp_transformed_blensor_data_vec;
    BlensorData tmp_blensordata;
    double initial_angle = 0.0; // for intial angle to rotate (degree)
    double rotating_angle_step = 0.0;
    double rotating_angle = 0.0; // for rotating angle per step (degree)
    float hokuyo_angle_resolution = 0.25;
    float hokuyo_angle;

    // Transformation
    /*  METHOD #2: Using a Affine3f
    This method is easier and less error prone
    */
    Eigen::Affine3f transform_world2sensor = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_sensor2world;
    Eigen::Affine3f transform_sensorpolar2sensorcartesian = Eigen::Affine3f::Identity();

    // --------------------------------------
    // -----Parse Command Line Arguments-----
    // --------------------------------------
    if (pcl::console::find_argument (argc, argv, "-h") >= 0){
        printUsage (argv[0]);
        return 0;
    }
    bool simple(false), rgb(false), custom_c(false), normals(false),
    shapes(false), viewports(false), interaction_customization(false), file_load(false);
    if (pcl::console::find_argument (argc, argv, "-s") >= 0){
        simple = true;
        std::cout << "Simple visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-c") >= 0){
        custom_c = true;
        std::cout << "Custom colour visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-r") >= 0){
        rgb = true;
        std::cout << "RGB colour visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-n") >= 0){
        normals = true;
        std::cout << "Normals visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-a") >= 0){
        shapes = true;
        std::cout << "Shapes visualisation example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-v") >= 0){
        viewports = true;
        std::cout << "Viewports example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-i") >= 0){
        interaction_customization = true;
        std::cout << "Interaction Customization example\n";
    }
    else if (pcl::console::find_argument (argc, argv, "-f") >= 0){
        file_load = true;
        std::cout << "Load Sensor Data EVD File\n";
        std::cout<<"Parsing contents: "<<argv[2]<<std::endl;
        evd_File = fopen(argv[2], "rb");
        std::string str1 (argv[2]);
        std::string str2 (argv[3]); // rotating start angle
        std::string str3 (argv[4]); // rotating angle per step

        initial_angle = std::stod(str2);
        rotating_angle_step = std::stod(str3);
        std::cout<<"Start angle: "<<initial_angle<<", Rotating angle step: "<<rotating_angle_step<<std::endl;

        std::string wstr, wstr2;

        std::cout<<"File Name size: "<<str1.size()<<std::endl;
        wstr = str1.substr(0, str1.size()-4);
        wstr += "_data.txt";
        //std::cout<<wstr<<std::endl;
        ofstream wFile(wstr);
        wstr2 = str1.substr(0, str1.size()-4);
        wstr2 += "_transformed_data.txt";
        ofstream wFile2(wstr2);

        fseek(evd_File, 0, SEEK_END);
        evd_size = ftell(evd_File);
        std::cout<<"EVD File size: "<<evd_size<<std::endl;
        rewind (evd_File);
        if( str1.compare(0,9,"rotating_") == 0 || str1.compare(0,7,"hokuyo_") == 0){
            cur_data_pos = 0; // current position of data
            init_tmp = (int*)malloc(sizeof(int)); 
            //std::cout<<"Rotating 2D Lidar Scan data size: "<<init_tmp[0]<<std::endl;
            read_size = 0;
            rotating_angle = initial_angle;
            while( cur_data_pos < evd_size ){
                read_size = fread(init_tmp,sizeof(int),1,evd_File); // read number of scan dat asize
                cur_data_pos += sizeof(int);
                fseek(evd_File, cur_data_pos, SEEK_SET);
                //std::cout<<"current scan data size: "<<init_tmp[0]<<std::endl;
                cur_scan_data_size = sizeof(double)*init_tmp[0]*blensor_data_num;
                temp = (double*)malloc(cur_scan_data_size);
                read_size = fread(temp,sizeof(double),init_tmp[0]*blensor_data_num, evd_File);
                cur_data_pos += cur_scan_data_size;
                fseek(evd_File, cur_data_pos, SEEK_SET);
                //std::cout<<"cur_data_pos: "<<cur_data_pos<<std::endl;

                // for rotating angle
                transform_world2sensor.translation() << 0.0, 0.0, 0.0;
                transform_world2sensor.rotate(Eigen::AngleAxisf (pcl::deg2rad(rotating_angle), Eigen::Vector3f::UnitZ()));
                rotating_angle = rotating_angle_step;
                transform_sensor2world = transform_world2sensor.inverse();
                //printf ("Matrix4f: \n");
                //std::cout << transform_world2sensor.matrix() << std::endl;
                //std::cout<<"Rotating angle: "<<rotating_angle<<std::endl;
                // Scan Data Store into Vector
                j = 1;
                for ( i = 0 ; i < init_tmp[0]*blensor_data_num ; i++ ){
                    //printf("%g ", temp[i]);
                    if( j < blensor_data_num ){
                        switch(j){
                            case 1:
                                tmp_blensordata.timestamp = temp[i];
                                break;
                            case 2:
                                tmp_blensordata.yaw = temp[i];
                                break;
                            case 3:
                                tmp_blensordata.pitch = temp[i];
                                break;
                            case 4:
                                tmp_blensordata.dist = temp[i];
                                break;
                            case 5:
                                tmp_blensordata.noisy_dist = temp[i];
                                break;
                            case 6:
                                tmp_blensordata.x = temp[i];
                                break;
                            case 7:
                                tmp_blensordata.y = temp[i];
                                break;
                            case 8:
                                tmp_blensordata.z = temp[i];
                                break;
                            case 9:
                                tmp_blensordata.noisy_x = temp[i];
                                break;
                            case 10:
                                tmp_blensordata.noisy_y = temp[i];
                                break;
                            case 11:
                                tmp_blensordata.noisy_z = temp[i];
                                break;
                            case 12:
                                tmp_blensordata.r = temp[i];
                                break;
                            case 13:
                                tmp_blensordata.g = temp[i];
                                break;
                            case 14:
                                tmp_blensordata.b = temp[i];
                                break;
                            default:
                                break;
                        }
                        j++;
                    }
                    else{
                        tmp_blensordata.id = (int32_t)temp[i];
                        blensor_data_vec.push_back(tmp_blensordata);
                        tmp_blensor_data_vec.push_back(tmp_blensordata);
                        j = 1;

                        //printf("\n");
                    }
                }// for ( i = 0 ; i < init_tmp[0] ; i++ )
                free(temp);
                // Transformation from sensor coordinate to world coordinate
                tmp_transformed_blensor_data_vec = TransformBlensorData( tmp_blensor_data_vec, transform_world2sensor );
                std::cout<<"tmp_transformed_blensor_data_vec.size(): "<<tmp_transformed_blensor_data_vec.size()<<std::endl;
                for( k = 0 ; k < tmp_transformed_blensor_data_vec.size() ; k++ ){
                    transformed_blensor_data_vec.push_back(tmp_transformed_blensor_data_vec[k]);
                }
                tmp_blensor_data_vec.clear();
                tmp_transformed_blensor_data_vec.clear();
                //std::cout<<"blensor_data_vec.size(): = "<<blensor_data_vec.size()<<" at every rotation step"<<std::endl;
            }//while( cur_data_pos < evd_size )
        } // rotating Hokuyo case
        else{
            // HDL Scan
            evd_size = evd_size - sizeof(int); // int: number of scan point
            evd_size = evd_size/(sizeof(double));
            std::cout<<"EVD data size: "<<evd_size<<std::endl;
            temp = (double*)malloc(sizeof(double)*evd_size);
            init_tmp = (int*)malloc(sizeof(int));
            read_size = fread(init_tmp,sizeof(int),1,evd_File);
            read_size = fread(temp,sizeof(double),evd_size,evd_File);
            std::cout<<"HDL64E Scan data size: "<<init_tmp[0]<<std::endl;
            // Scan Data Store into Vector
            j = 1;
            for (i = 0 ; i < evd_size ; i++ ){
                //printf("%g ", temp[i]);
                if( j < blensor_data_num ){
                    switch(j){
                        case 1:
                            tmp_blensordata.timestamp = temp[i];
                            break;
                        case 2:
                            tmp_blensordata.yaw = temp[i];
                            break;
                        case 3:
                            tmp_blensordata.pitch = temp[i];
                            break;
                        case 4:
                            tmp_blensordata.dist = temp[i];
                            break;
                        case 5:
                            tmp_blensordata.noisy_dist = temp[i];
                            break;
                        case 6:
                            tmp_blensordata.x = temp[i];
                            break;
                        case 7:
                            tmp_blensordata.y = temp[i];
                            break;
                        case 8:
                            tmp_blensordata.z = temp[i];
                            break;
                        case 9:
                            tmp_blensordata.noisy_x = temp[i];
                            break;
                        case 10:
                            tmp_blensordata.noisy_y = temp[i];
                            break;
                        case 11:
                            tmp_blensordata.noisy_z = temp[i];
                            break;
                        case 12:
                            tmp_blensordata.r = temp[i];
                            break;
                        case 13:
                            tmp_blensordata.g = temp[i];
                            break;
                        case 14:
                            tmp_blensordata.b = temp[i];
                            break;
                        default:
                            break;
                    }
                    j++;
                }
                else{
                    tmp_blensordata.id = (int)temp[i];
                    blensor_data_vec.push_back(tmp_blensordata);

                    int tmpi = blensor_data_vec.size();
                    while( blensor_data_vec[tmpi-1].timestamp == blensor_data_vec[tmpi-2].timestamp && tmpi > 1 ){
                        if( blensor_data_vec[tmpi-1].pitch < blensor_data_vec[tmpi-2].pitch ){
                            std::swap(blensor_data_vec[tmpi-2], blensor_data_vec[tmpi-1]);
                            tmpi--;
                        }
                        else
                            break;
                    }
                    j = 1;
                    //printf("\n");
                }
            }// for (i = 0 ; i < evd_size ; i++ )
        }// else for HDL
        //std::cout<<"i = "<<i<<std::endl;
        std::cout<<"Close EVD File"<<std::endl;
        fclose(evd_File);

        std::cout<<"blensor vector size: "<<blensor_data_vec.size()<<std::endl;
        // write scan data.txt

        for( i = 0 ; i < blensor_data_vec.size() ; i++ ){
            wFile<<blensor_data_vec[i].timestamp<<" "<<blensor_data_vec[i].yaw<<" "<<blensor_data_vec[i].pitch<<" "<<blensor_data_vec[i].dist<<
            " "<<blensor_data_vec[i].noisy_dist<<" "<<blensor_data_vec[i].x<<" "<<blensor_data_vec[i].y<<" "<<blensor_data_vec[i].z<<" "<<
            blensor_data_vec[i].noisy_x<<" "<<blensor_data_vec[i].noisy_y<<" "<<blensor_data_vec[i].noisy_z<<" "<<
            blensor_data_vec[i].r<<" "<<blensor_data_vec[i].g<<" "<<blensor_data_vec[i].b<<" "<<blensor_data_vec[i].id<<std::endl;
        }
        wFile.close();
        for( i = 0 ; i < transformed_blensor_data_vec.size() ; i++  ){
            wFile2<<transformed_blensor_data_vec[i].timestamp<<" "<<transformed_blensor_data_vec[i].yaw<<" "
            <<transformed_blensor_data_vec[i].pitch<<" "<<transformed_blensor_data_vec[i].dist<<" "
            <<transformed_blensor_data_vec[i].noisy_dist<<" "<<transformed_blensor_data_vec[i].x<<" "
            <<transformed_blensor_data_vec[i].y<<" "<<transformed_blensor_data_vec[i].z<<" "
            <<transformed_blensor_data_vec[i].noisy_x<<" "<<transformed_blensor_data_vec[i].noisy_y<<" "
            <<transformed_blensor_data_vec[i].noisy_z<<" "<<transformed_blensor_data_vec[i].r<<" "
            <<transformed_blensor_data_vec[i].g<<" "<<transformed_blensor_data_vec[i].b<<" "<<transformed_blensor_data_vec[i].id<<std::endl;
        }
        wFile2.close();
    }
    else{
        printUsage (argv[0]);
        return 0;
    }

    // ------------------------------------
    // -----Create example point cloud-----
    // ------------------------------------
    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "Genarating example point clouds.\n";
    // We're going to make an ellipse extruded along the z-axis. The colour for
    // the XYZRGB cloud will gradually go from red to green to blue.
    uint8_t r(255), g(15), b(15);
    for (float z(-1.0); z <= 1.0; z += 0.05){
        for (float angle(0.0); angle <= 360.0; angle += 5.0){
            pcl::PointXYZ basic_point;
            basic_point.x = 0.5 * cosf (pcl::deg2rad(angle));
            basic_point.y = sinf (pcl::deg2rad(angle));
            basic_point.z = z;
            basic_cloud_ptr->points.push_back(basic_point);

            pcl::PointXYZRGB point;
            point.x = basic_point.x;
            point.y = basic_point.y;
            point.z = basic_point.z;
            uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
                  static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
            point.rgb = *reinterpret_cast<float*>(&rgb);
            point_cloud_ptr->points.push_back (point);
        }
        if (z < 0.0){
            r -= 12;
            g += 12;
        }
        else{
            g -= 12;
            b += 12;
        }
    }

    basic_cloud_ptr->width = (int) basic_cloud_ptr->points.size();
    basic_cloud_ptr->height = 1;
    point_cloud_ptr->width = (int) point_cloud_ptr->points.size();
    point_cloud_ptr->height = 1;

    // ---------------------------------------------------------------
    // ----- Load Sensor Data -----
    // ---------------------------------------------------------------

    std::cout<<"Sensor Data Load"<<std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sensor_point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_sensor_point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    // For Visualization
    sensor_point_cloud_ptr = ConvertPointCloud2PCL(blensor_data_vec, 15, 0, 255);
    transformed_sensor_point_cloud_ptr = ConvertPointCloud2PCL(transformed_blensor_data_vec, 255, 0, 0);

    //transformed_sensor_point_cloud_ptr->width = (int)transformed_sensor_point_cloud_ptr->points.size();
    //std::cout<<"transformed sensor point size: "<<transformed_sensor_point_cloud_ptr->points.size()<<std::endl;
    //transformed_sensor_point_cloud_ptr->height = 1;

    // --------------------------------------
    // plane Test
    // ---------------------------------------
    pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients); 
    plane_1->values.resize (4); 
    plane_1->values[0] = 1; 
    plane_1->values[1] = 0; 
    plane_1->values[2] = 0; 
    plane_1->values[3] = -10; 

    // polygon test
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr polygon_test_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZRGB vertex;
    vertex.x = 1.0;
    vertex.y = 0.0;
    vertex.z = 0.0;
    vertex.r = 255;
    vertex.g = 0;
    vertex.b = 0;
    polygon_test_ptr->points.push_back(vertex);
    vertex.x = 1.0;
    vertex.y = 1.0;
    vertex.z = 0.0;
    vertex.r = 255;
    vertex.g = 0;
    vertex.b = 0;
    polygon_test_ptr->points.push_back(vertex);
    vertex.x = 2.0;
    vertex.y = 1.0;
    vertex.z = 0.0;
    vertex.r = 255;
    vertex.g = 0;
    vertex.b = 0;
    polygon_test_ptr->points.push_back(vertex);
    vertex.x = 2.0;
    vertex.y = 0.0;
    vertex.z = 0.0;
    vertex.r = 255;
    vertex.g = 0;
    vertex.b = 0;
    polygon_test_ptr->points.push_back(vertex);
    polygon_test_ptr->width = (int)polygon_test_ptr->points.size();
    polygon_test_ptr->height = 1;

    // planar polygon test
    pcl::PlanarPolygon<pcl::PointXYZRGB> planar_polygon_test;
    pcl::PointCloud<pcl::PointXYZRGB> contour_vertex;
    pcl::PointXYZRGB pvertex;

    pvertex.x = -1.0;
    pvertex.y = 0.0;
    pvertex.z = 0.0;
    pvertex.r = 255;
    pvertex.g = 0;
    pvertex.b = 0;
    contour_vertex.points.push_back(pvertex);

    pvertex.x = -1.0;
    pvertex.y = 1.0;
    pvertex.z = 0.0;
    pvertex.r = 255;
    pvertex.g = 0;
    pvertex.b = 0;
    contour_vertex.points.push_back(pvertex);

    pvertex.x = -2.0;
    pvertex.y = 1.0;
    pvertex.z = 0.0;
    pvertex.r = 255;
    pvertex.g = 0;
    pvertex.b = 0;
    contour_vertex.points.push_back(pvertex);

    pvertex.x = -3.0;
    pvertex.y = 0.5;
    pvertex.z = 0.0;
    pvertex.r = 255;
    pvertex.g = 0;
    pvertex.b = 0;
    contour_vertex.points.push_back(pvertex);

    pvertex.x = -2.0;
    pvertex.y = 0.0;
    pvertex.z = 0.0;
    pvertex.r = 255;
    pvertex.g = 0;
    pvertex.b = 0;
    contour_vertex.points.push_back(pvertex);

    contour_vertex.width = (int)contour_vertex.points.size();
    contour_vertex.height = 1;
    planar_polygon_test.setContour(contour_vertex);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> polygon_vertices_ptr_vec;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr polygon_vertices_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<pcl::PlanarPolygon<pcl::PointXYZRGB>> planar_polygon_vec;
    planar_polygon_vec.push_back(planar_polygon_test);

    // ----------------------------------------------------------------
    // -----Calculate surface normals with a search radius of 0.05-----
    // ----------------------------------------------------------------
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (point_cloud_ptr);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.05);
    ne.compute (*cloud_normals1);

    // ---------------------------------------------------------------
    // -----Calculate surface normals with a search radius of 0.1-----
    // ---------------------------------------------------------------
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch (0.1);
    ne.compute (*cloud_normals2);

    // Viewer Initiate -------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    if (simple){
        viewer = simpleVis(basic_cloud_ptr);
    }
    else if (rgb){
        viewer = rgbVis(point_cloud_ptr);
    }
    else if (custom_c){
        viewer = customColourVis(basic_cloud_ptr);
    }
    else if (normals){
        viewer = normalsVis(point_cloud_ptr, cloud_normals2);
    }
    else if (shapes){
        viewer = shapesVis(point_cloud_ptr);
    }
    else if (viewports){
        viewer = viewportsVis(point_cloud_ptr, cloud_normals1, cloud_normals2);
    }
    else if (interaction_customization){
        viewer = interactionCustomizationVis();
    }
    else if (file_load){
        viewer = SensorVisualizer(transformed_sensor_point_cloud_ptr);
        //viewer->addPlane(*plane_1, "plane", 0);
        //viewer->addPolygon<pcl::PointXYZRGB>(polygon_test_ptr, 255, 0, 0, "polygon", 0);
        //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "polygon");
        viewer->addPolygon<pcl::PointXYZRGB>(planar_polygon_test, 0, 0, 255, "planar polygon", 0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,"planar polygon");
        std::string spi_num("sphere_");
        for( int spi = 0 ; spi < contour_vertex.points.size() ; spi++ ){
            spi_num += std::to_string(spi);
            viewer->addSphere (contour_vertex.points[spi], 0.01, 255, 0, 0, spi_num);
        }
    }

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
}