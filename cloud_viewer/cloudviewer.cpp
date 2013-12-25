#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>



void 
viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
	viewer.setCameraPosition(0,0,-2,0,-1,1,0);
	viewer.resetCamera();
}

int
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::io::loadPCDFile(argv[1], *cloud);

  pcl::visualization::CloudViewer viewer("viewer");
  viewer.showCloud(cloud);
  viewer.runOnVisualizationThreadOnce (viewerOneOff);

   while (!viewer.wasStopped ())
   { Sleep(10);  }

   return(0);
}

