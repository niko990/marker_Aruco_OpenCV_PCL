#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <limits>
#include <stdio.h>
#include <cmath>

pcl::PointXYZRGBA center;
pcl::PointXYZRGBA center2;
pcl::PointXYZRGBA center3;
pcl::PointXYZRGBA center4;


void viewerOneOff (pcl::visualization::PCLVisualizer& viewer) {

    viewer.setBackgroundColor (1.0, 0.5, 1.0);
	viewer.setCameraPosition(0,0,-2,0,-1,1,0);
	viewer.resetCamera();
	viewer.addSphere(center, 0.02, 255, 0, 0, "sphere1");
	viewer.addSphere(center2, 0.02, 0, 255, 0, "sphere2");
	viewer.addSphere(center3, 0.02, 0, 0, 255, "sphere3");
	viewer.addSphere(center4, 0.02, 120, 120, 0, "sphere4");
}


pcl::PointXYZRGBA getNearestPoint (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, float i, float j){
	
	int k = (cloud->width * (int)(j + 0.5) ) + (int)(i + 0.5);
	pcl::PointXYZRGBA p = cloud->points[k];

 
	// Controllo se p è un NaN o infinito e lo ritorno se è un valore corretto
	if (!(p.x == std::numeric_limits<float>::quiet_NaN ()) && !(p.x == std::numeric_limits<float>::infinity())){
		cout << "\n getNearestPoint - Indice punto = " << k << " - " << p << "\n";
		return p;
	}
	
	// altrimenti calcolo l'errore commesso dall'approssimazione e cerco un punto nell'intorno
	// di p0 che è più vicino e lo ritorno (controllando che non sia NaN o inf)
	float di = i - (int)(i + 0.5);
	float dj = j - (int)(j + 0.5);
	
	if ( abs(di) <= abs(dj)){
		if (di > 0)
			p = cloud->points[k + 1];
		else
			p = cloud->points[k - 1];
	}
	else {
		if (dj > 0)
			p = cloud->points[k + cloud->width];
		else
			p = cloud->points[k - cloud->width];
	}

	if (!(p.x == std::numeric_limits<float>::quiet_NaN ()) && !(p.x == std::numeric_limits<float>::infinity())){
		cout << "\n getNearestPoint (seconda scelta) - Indice punto = " << k << " - " << p << "\n";
		return p;
	}

}


int main (int argc, char** argv){

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
	float mark_x, mark_y;	
	float mark_x2, mark_y2;
	float mark_x3, mark_y3;
	float mark_x4, mark_y4;


	if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloud) == -1) {
		PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
		return (-1);
	}

	std::cout << "Loaded " <<  cloud->width * cloud->height << " data points\n";
           
	cout << "Coordinata x marker: ";
	cin >> mark_x;
	cout << "Coordinata y marker: ";
	cin >> mark_y;
		cout << "Coordinata x marker: ";
	cin >> mark_x2;
	cout << "Coordinata y marker: ";
	cin >> mark_y2;
		cout << "Coordinata x marker: ";
	cin >> mark_x3;
	cout << "Coordinata y marker: ";
	cin >> mark_y3;
		cout << "Coordinata x marker: ";
	cin >> mark_x4;
	cout << "Coordinata y marker: ";
	cin >> mark_y4;


	//k = (cloud->width * ((int)mark_y)) + ((int)mark_x);
	
	//cout << "\nCalcolo per x = " << (int)(mark_x + 0.5) << " e y = " << (int)(mark_y + 0.5);

	center = getNearestPoint (cloud, mark_x, mark_y);
	center2 = getNearestPoint (cloud, mark_x2, mark_y2);
	center3 = getNearestPoint (cloud, mark_x3, mark_y3);
	center4 = getNearestPoint (cloud, mark_x4, mark_y4);

	//cout << "Trovato punto = " << center << "\n";
	/*std::cout << "    " << cloud->points[center].x
              << " "    << cloud->points[center].y
              << " "    << cloud->points[center].z << std::endl;*/



	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud(cloud);
	viewer.runOnVisualizationThreadOnce (viewerOneOff);

	while (!viewer.wasStopped ())
	{ Sleep(60);  }

  return (0);
}