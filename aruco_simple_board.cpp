#include <iostream>
#include <fstream>
#include <sstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "aruco.h"
#include "boarddetector.h"
#include "cvdrawingutils.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl\common\transforms.h>

#define _USE_MATH_DEFINES

using namespace cv;
using namespace aruco; 
using namespace pcl;

/*
Vector<pcl::PointXYZRGBA> CentersA;

// VISUALIZZATORE PCD
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer) {
	
    viewer.setBackgroundColor (1.0, 0.5, 1.0);
	viewer.setCameraPosition(0,0,-2,0,-1,1,0);
	viewer.resetCamera();
	
	for (int i=0; i < CentersA.size(); i++)
		viewer.addSphere(CentersA[i], 0.02, 255, 0, 0);

}*/

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


// PROCEDURA DI RICERCA DEL PUNTO 3D CORRISPONDENTE ALLE COORDINATE (x,y) RILEVATE DA ARUCO
// RITORNA point.x = -1 SE NON è STATO TROVATA NESSUNA CORRISPONDENZA
void getNearestPoint 
	(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_markers_cloud, float i, float j) {
	
	int k = (cloud->width * (int)(j + 0.5) ) + (int)(i + 0.5);
	pcl::PointXYZRGBA p = cloud->points[k];

 
	// Controllo se p è un NaN o infinito e lo ritorno se è un valore corretto
	if (!(p.x == std::numeric_limits<float>::quiet_NaN ()) && !(p.x == std::numeric_limits<float>::infinity())){
		cout << "\n getNearestPoint - Indice punto = " << k << " - " << p << "\n";

		// inserisci nella nuvola di marker di output passata il primo punto scelto
		output_markers_cloud->points[k] = p;

		//return p;
	}
	else {
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
		}
		else{
			cout << "Error: non è stato possibile trovare una corrispondenza 3D per il punto (" << i << "," << j << ")\n";
			p.x = -1;
		}

		// inserisci nella nuvola di marker di output passata il secondo punto scelto
		output_markers_cloud->points[k] = p;
	}
	//return p;

	//cout << "\nEseguita getNearestPoint con coordinate (" << i << "," << j << ") . Risultato p = " << p;

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
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr markers_cloudA (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr markers_cloudB (new pcl::PointCloud<pcl::PointXYZRGBA>);	
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rotatedCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		cv::Mat InImageA, InImageB;
		pcl::PassThrough<pcl::PointXYZRGBA> pass;
		

		// LETTURA DEGLI ARGOMENTI IN INPUT

		TheBoardConfig.readFromFile(argv[3]);
		
		cout << "\nCaricamento nuvola A......";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloudA) == -1) {
			PCL_ERROR ("Errore apertura PCD file!\n");
			return (-1);
		}

		cout << "completato!\n";

		// Check if the cloudA is organized
		if (cloudA->height == 1) {
			cout << ("Input cloudA is not organized.\n");
			return (-1);
		}

		cout << "Caricamento nuvola B......";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[2], *cloudB) == -1) {
			PCL_ERROR ("Errore apertura PCD file!\n");
			return (-1);
		}
		
		cout << "completato!\n";

		// Check if the cloudB is organized
		if (cloudB->height == 1) {
			cout << ("Input cloudB is not organized.\n");
			return (-1);
		}

		// Conversione da PCD a immagine per elaborazione con Aruco
		InImageA = pcd2img(cloudA);
		InImageB = pcd2img(cloudB);

		// Inizializzazione delle nuvole che conterranno i punti marker rilevati
		markers_cloudA->width    = cloudA->width * cloudA->height;
		markers_cloudA->height   = 1;
		markers_cloudA->is_dense = false;
		markers_cloudA->points.resize (markers_cloudA->width * markers_cloudA->height);
		
		markers_cloudB->width    = cloudB->width * cloudB->height;
		markers_cloudB->height   = 1;
		markers_cloudB->is_dense = false;
		markers_cloudB->points.resize (markers_cloudB->width * markers_cloudB->height);


		// INIZIO PROCEDURA DI DETECTING DEI MARKER
		MDetectorA.detect(InImageA,MarkersA);
		MDetectorB.detect(InImageB,MarkersB);



		
		// Stampa info su immagine A
		std::cout << "\n*** MARKERS IMMAGINE A ***\n";
		for(unsigned int i=0;i<MarkersA.size();i++){
			cout << MarkersA[i].id << endl;
			for (int v=0 ; v<4 ; v++){
				//cout << "(" << MarkersA[i][v].x << "," << MarkersA[i][v].y << ") - ";
				getNearestPoint (cloudA, markers_cloudA, MarkersA[i][v].x, MarkersA[i][v].y);
			}
			cout << endl;
			MarkersA[i].draw(InImageA,Scalar(0,0,255),2);
		}
		
		// Stampa info su immagine B
		std::cout << "\n*** MARKERS IMMAGINE B ***\n";
		for(unsigned int i=0;i<MarkersB.size();i++){
			cout << MarkersB[i].id << endl;
			for (int v=0 ; v<4 ; v++){
				//cout << "(" << MarkersB[i][v].x << "," << MarkersB[i][v].y << ") - ";	
				getNearestPoint (cloudB, markers_cloudB, MarkersB[i][v].x, MarkersB[i][v].y);
			}
			cout << endl;
			MarkersB[i].draw(InImageB,Scalar(0,0,255),2);
		}
		


		//show input with augmented information
		cv::imshow("inA",InImageA);
		cv::imshow("inB",InImageB);
		


		// Filtraggio delle nuvole di marker per eliminare i punti nulli a 0
		pass.setInputCloud (markers_cloudA);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0.0001, 100);
		pass.filter (*markers_cloudA);

		pass.setInputCloud (markers_cloudB);
		pass.filter (*markers_cloudB);

		/*
		// PROVA DI ROTAZIONE DI markers_cloudB
		Eigen::Matrix4f transformationMatrix;
		float degree;
		cout << "Angolo di rotazione della nuvola cloudB = ";
		cin >> degree;
		float angle = degree * M_PI / 180.0;
		transformationMatrix << 
			cos(angle),  -sin(angle), 1, 0, 
			sin(angle),   cos(angle), 1, 0, 
			         0,            0, 1, 0, 
					 0,            0, 0, 1; 
		
		pcl::transformPointCloud(*markers_cloudB,*rotatedCloud, transformationMatrix);
		*/


		// Salvataggio delle due nuvole di marker
		pcl::io::savePCDFileASCII ("markers_cloudA.pcd", *markers_cloudA);
		pcl::io::savePCDFileASCII ("markers_cloudB.pcd", *markers_cloudB);
		//pcl::io::savePCDFileASCII ("rotated_markers_cloudB.pcd", *rotatedCloud);
		
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