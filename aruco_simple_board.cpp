#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "aruco.h"
#include "boarddetector.h"
#include "cvdrawingutils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace cv;
using namespace aruco; 




// VISUALIZZATORE PCD
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer) {
	
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
	viewer.setCameraPosition(0,0,-2,0,-1,1,0);
	viewer.resetCamera();
	/*
	for (int i=0; i < centersA.size(); i++)
		viewer.addSphere(centersA[i], 0.02, 255, 0, 0);*/

}


// CONVERSIONE DA PCD A IMMAGINE MAT
cv::Mat pcd2img (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
		cv::Mat test = cv::Mat(Size(cloud->width, cloud->height), CV_8UC3);
		int i, j;
		for (int k = 0; k < cloud->points.size (); k++) {
			i= k % cloud->width;
			j= k / cloud->width;
			test.at<cv::Vec3b>(j, i)[2] = cloud->points[k].r;
			test.at<cv::Vec3b>(j, i)[1] = cloud->points[k].g;
			test.at<cv::Vec3b>(j, i)[0] = cloud->points[k].b;
		}
		return test;
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



int main(int argc,char **argv) {
	try	{
		if(argc<4) {cerr<<"USO: pcd_A pcd_B boardConfig.yml [outImage]"<<endl; exit(0);}
		MarkerDetector MDetectorA, MDetectorB;
		vector<Marker> MarkersA, MarkersB;
		BoardConfiguration TheBoardConfig;
		BoardDetector TheBoardDetector;
		Board TheBoardDetected;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGBA>);
		//std::vector<pcl::PointXYZRGBA> point_vectorA, point_vectorB; 
		std::vector<float> x_a, y_a;
		cv::Mat InImageA, InImageB;
		

		// LETTURA DEGLI ARGOMENTI IN INPUT

		TheBoardConfig.readFromFile(argv[3]);

		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloudA) == -1) {
			PCL_ERROR ("Errore apertura PCD file!\n");
			return (-1);
		}

		// Check if the cloudA is organized
		if (cloudA->height == 1) {
			cout << ("Input cloudA is not organized.\n");
			return (-1);
		}

		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[2], *cloudB) == -1) {
			PCL_ERROR ("Errore apertura PCD file!\n");
			return (-1);
		}
		
		// Check if the cloudB is organized
		if (cloudB->height == 1) {
			cout << ("Input cloudB is not organized.\n");
			return (-1);
		}

		// Conversione da PCD a immagine per elaborazione con Aruco
		InImageA = pcd2img(cloudA);
		InImageB = pcd2img(cloudB);

		
		// INIZIO PROCEDURA DI DETECTING DEI MARKER
		MDetectorA.detect(InImageA,MarkersA);
		MDetectorB.detect(InImageB,MarkersB);

		// Stampa info su immagine A
		std::cout << "\n*** MARKERS IMMAGINE A ***\n";
		for(unsigned int i=0;i<MarkersA.size();i++){
			cout << MarkersA[i].id << endl;
			for (int v=0 ; v<4 ; v++){
				cout << "(" << MarkersA[i][v].x << "," << MarkersA[i][v].y << ") - ";
				//centersA[i] = getNearestPoint (cloudA, MarkersA[i][v].x, MarkersA[i][v].y);
				//x_a[i] = MarkersA[i][v].x;
				//y_a[i] = MarkersA[i][v].y;
			}
			cout << endl;
			MarkersA[i].draw(InImageA,Scalar(0,0,255),2);
		}
		
		// Stampa info su immagine B
		std::cout << "\n*** MARKERS IMMAGINE B ***\n";
		for(unsigned int i=0;i<MarkersB.size();i++){
			cout << MarkersB[i].id << endl;
			for (int v=0 ; v<4 ; v++){
				cout << "(" << MarkersB[i][v].x << "," << MarkersB[i][v].y << ") - ";		
			}
			cout << endl;
			MarkersB[i].draw(InImageB,Scalar(0,0,255),2);
		}
		
		
		//getNearestPoint (cloudA, x_a, y_a);

		//center = getNearestPoint (cloud, mark_x, mark_y);

		//show input with augmented information
		cv::imshow("inA",InImageA);
		cv::imshow("inB",InImageB);

		/*
		pcl::visualization::CloudViewer viewer("viewer");
		viewer.showCloud(cloudA);
		viewer.runOnVisualizationThreadOnce (viewerOneOff);

		while (!viewer.wasStopped ()) { Sleep(60);  }

		*/

		
		cv::waitKey(0);//wait for key to be pressed
		
		//if(argc==4) cv::imwrite(argv[3],InImage);
		
	} catch (std::exception &ex) {
		cout<<"Exception :"<<ex.what()<<endl;
	}

}