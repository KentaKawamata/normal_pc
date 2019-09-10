#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/normal_3d.h>

// 法線を推定する
void estimateNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals )
{
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius <m>
    ne.setRadiusSearch (0.005);
    // Compute the features
    ne.compute (*cloud_normals);
}

int main(int argc, char* argv[])
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals( new pcl::PointCloud < pcl::Normal > );

    pcl::io::loadPLYFile("/mnt/container-data/model_for_RT/3d_model.ply", *cloud);
    std::cout << "Size of PointClud is " << cloud->size() << " \n";

    pcl::StopWatch time;
    time.reset();
                
    estimateNormal( cloud, cloud_normals );

    std::cout << "The running time is " << time.getTimeSeconds() << " seconds.\n";

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer( "Point Cloud Viewer" ) );
    const pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>::Ptr color
                (new pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
                (cloud, 255, 0.0, 0.0));

    viewer->addPointCloud(cloud, *color, "pointcloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud_normals, 100, 0.02, "normals");
		
    while(!viewer->wasStopped())
    {
        viewer->spinOnce();
    }

    return 0;
}
