#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
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
#include <pcl\registration\transformation_estimation.h>
#include <pcl\registration\transformation_estimation_svd.h>


using namespace aruco; 


// Numero massimo di ID che verranno riconosciuti nelle nuvole (<= numero di ID della board)
// Il vettore IDcomuni è inizialmente settato a questa dimensione, una volta riconosciuti i marker verrà ridimensionato 
// secondo il numero di ID che sono stati rilevati e che sono presenti in entrambe le nuvole
#define ID_COMUNI_SIZE 24

pcl::PointXYZRGBA centriA[ID_COMUNI_SIZE * 4], centriB[ID_COMUNI_SIZE * 4];


// VISUALIZZATORE PCD
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer) {
	int correspondence_number = 0;
	std::stringstream ss;
	
	viewer.setBackgroundColor (1.0, 0.5, 1.0);
	viewer.setCameraPosition(0,0,-2,0,-1,1,0);
	viewer.resetCamera();
		
	while( pcl::isFinite(centriA[correspondence_number] )){

		ss << "line" << correspondence_number;
		cout << "Next correspondence = " << correspondence_number << endl;
		viewer.addSphere(centriA[correspondence_number], 0.02, 255, 0, 0, ss.str(), 0);
		ss << "23";
		viewer.addSphere(centriB[correspondence_number], 0.02, 0, 255, 0, ss.str(), 0);
		//cout << "\nAggiungo nuova sfera: " << centriA[correspondence_number] << " - " << centriB[correspondence_number] << endl;
		ss << "d";
		viewer.addLine(centriA[correspondence_number], centriB[correspondence_number], 0, 0, 255, ss.str(), 0);
		correspondence_number++;
	}

}




// CONVERSIONE DA PCD A IMMAGINE cv::Mat
cv::Mat pcd2img (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud){
		cv::Mat test = cv::Mat(cv::Size(cloud->width, cloud->height), CV_8UC3);
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


// RICERCA DEL PUNTO 3D NELLA NUVOLA IN INPUT DATE X,Y
// RETURN pcl::PointXYZRGBA
pcl::PointXYZRGBA getNearestPoint (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, float i, float j) {
	
	int k = (cloud->width * (int)(j + 0.5) ) + (int)(i + 0.5);
	pcl::PointXYZRGBA p = cloud->points[k];

 
	// Controllo se p è un NaN o infinito e lo ritorno se è un valore corretto
	if (!(p.x == std::numeric_limits<float>::quiet_NaN ()) && !(p.x == std::numeric_limits<float>::infinity())){
		//cout << "\n getNearestPoint - Indice punto = " << k << " - " << p << "\n";

		// inserisci nella nuvola di marker di output passata il primo punto scelto
		//output_markers_cloud->points[k] = p;

		return p;
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
			//cout << "\n getNearestPoint (seconda scelta) - Indice punto = " << k << " - " << p << "\n";
		}
		else{
			cout << "Error: non è stato possibile trovare una corrispondenza 3D per il punto (" << i << "," << j << ")\n";
			p.x = -1;
		}

		// inserisci nella nuvola di marker di output passata il secondo punto scelto
		//output_markers_cloud->points[k] = p;
	}
	
	return p;

}



// RITORNA L'INDICE DEL PUNTO  3D CORRISPONDENTE NELLA NUVOLA cloud CON COORDINATE 2D i E j, -1 SE è NaN
int getNearestPointIndice (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, float i, float j) {
	
	int k = (cloud->width * (int)(j + 0.5) ) + (int)(i + 0.5);
		
	if (pcl_isfinite (cloud->points[k].x))
		return k;
	else
		return -1;
}



// FUNZIONE DI CARICAMENTO DELL'INPUT
int LoadInput(char** argv, BoardConfiguration TheBoardConfig, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB){

		TheBoardConfig.readFromFile(argv[3]);
		
		cout << "\nCaricamento nuvola A......";
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[1], *cloudA) == -1) {
			PCL_ERROR ("Errore apertura pcd_A file!\n");
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
			PCL_ERROR ("Errore apertura pcd_B file!\n");
			return (-1);
		}
		
		cout << "completato!\n";

		// Check if the cloudB is organized
		if (cloudB->height == 1) {
			cout << ("Input cloudB is not organized.\n");
			return (-1);
		}

}





int main(int argc,char **argv) {
	try	{
		if (argc < 4) { cerr << "USO: pcd_A pcd_B boardConfig.yml" << endl; exit(0);}
		MarkerDetector MDetectorA, MDetectorB;
		vector<Marker> MarkersA, MarkersB;
		BoardConfiguration TheBoardConfig;
		BoardDetector TheBoardDetector;
		Board TheBoardDetected;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr markers_cloudA (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr markers_cloudB (new pcl::PointCloud<pcl::PointXYZRGBA>);	
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_temp;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudAB (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PassThrough<pcl::PointXYZRGBA> pass;	
		cv::Mat InImageA, InImageB;
		int IDcomuni[ID_COMUNI_SIZE], pos = 0;


		// LETTURA DEGLI ARGOMENTI IN INPUT
		if (LoadInput (argv, TheBoardConfig, cloudA, cloudB) == -1) { cout << "\nExit!"; exit (0);}

		
		
		// Conversione da PCD a immagine per elaborazione con Aruco (LE NUVOLE DEVONO ESSERE ORGANIZZATE)
		InImageA = pcd2img(cloudA);
		InImageB = pcd2img(cloudB);



		// INIZIO PROCEDURA DI DETECTING DEI MARKER

		MDetectorA.detect(InImageA,MarkersA);
		MDetectorB.detect(InImageB,MarkersB);



		// RICERCA DELLE COSSIRPONDENZE DEGLI ID NEI VETTORI MarkersA e MarkersB

		
		// IDcomuni inizializzato a tutti 0
		for (int i=0; i < ID_COMUNI_SIZE; i++)
			IDcomuni[i] = -1;

		// Inizio ricerca, gli ID sono in ordine crescente
		for (int i = 0; i < MarkersA.size(); i++){
			for (int j = 0; j <	MarkersB.size(); j++){
					
				if (MarkersA[i].id != MarkersB[j].id)
					continue;

			IDcomuni[pos] = MarkersA[i].id;
			pos++;
			break;
			}
		}

		// stampo il vettore
		cout << "\nVettore degli IDcomuni" << endl;
		for (int i = 0; IDcomuni[i] != -1; i++)
			cout << "IDcomuni[" << i << "] = " << IDcomuni[i] <<  endl;








		

		// Stampa info su immagine A
		std::cout << "\n*** MARKERS IMMAGINE A ***\n";
		for(unsigned int i=0;i<MarkersA.size();i++){
			cout << MarkersA[i].id << endl;
					
			for (int v=0 ; v<4 ; v++){
				cout << "(" << MarkersA[i][v].x << "," << MarkersA[i][v].y << ") - ";
			}
			cout << endl;
			MarkersA[i].draw(InImageA,cv::Scalar(0,0,255),2);
		}
		

		// Stampa info su immagine B
		std::cout << "\n*** MARKERS IMMAGINE B ***\n";
		for(unsigned int i=0;i<MarkersB.size();i++){
			cout << MarkersB[i].id << endl;

			for (int v=0 ; v<4 ; v++){
				cout << "(" << MarkersB[i][v].x << "," << MarkersB[i][v].y << ") - ";
			}
			cout << endl;
			MarkersB[i].draw(InImageB,cv::Scalar(0,0,255),2);
		}
		


		//Visualizza immagini con informazioni sui Marker
		cv::imshow("inA",InImageA);
		cv::imshow("inB",InImageB);
		





		// INIZIO PROCEDURA DI TRASFORMAZIONE MEDIANTE LE CORRISPONDENZE TROVATE

		int idA = 0, idB = 0;
		for (int i = 0; IDcomuni[i] != -1; i++){

			if (MarkersA[idA].id == MarkersB[idB].id){

				// trovata corrispondenza tra 2 ID --> aggiungo in ordine i punti dei 4 vertici alle corrispondenze
				cout << "\n\nAggiunta corrispondenza #" << i+1 <<": ID = " << MarkersA[idA].id << endl;

				for (int v = 0; v < 4; v++) {
						
					markers_cloudA->push_back ( getNearestPoint(cloudA, MarkersA[idA][v].x, MarkersA[idA][v].y) );
					markers_cloudB->push_back ( getNearestPoint(cloudB, MarkersB[idB][v].x, MarkersB[idB][v].y) );

					cout << "\tVertice " << v << endl;
					cout << "\t\tCoordinate A = " << markers_cloudA->at(i).x << ", "<< markers_cloudA->at(i).y << ", " << markers_cloudA->at(i).z << endl; 
					cout << "\t\tCoordinate B = " << markers_cloudB->at(i).x << ", "<< markers_cloudB->at(i).y << ", " << markers_cloudB->at(i).z << endl; 
				}
				// ho trovato una corrispondenza quindi scorro entrambi gli indici
				idA++;
				idB++;
			} else {
				// ho 2 ID diversi, guardo se è diverso quello di A o quello di B
				if (IDcomuni[i] == MarkersA[idA].id){
					//IDcomuni è corrispondente in A ma non in B
					// scorro idB di una posizione e ritento la ricerca sulla stessa i
					idB++;
					i--;
				}
				else {
					// IDcomuni è corrispondente in B ma non in A
					// scorro idA di una posizione e ritento la ricerca sulla stessa i
					idA++;
					i--;
				}
			}
		}

		cout << "\n\nCorrispondenze trovate = " << markers_cloudA->size() << endl;


		// Creazione delle due matrici di trasformazione (con i metodi SVD e LM)
		Eigen::Matrix4f transformation_matrix_SVD; 


		// Creazione delle trasformazioni con il metodo LM e SVD
		pcl::registration::TransformationEstimationSVD <pcl::PointXYZRGBA, pcl::PointXYZRGBA> SVD; 



		// Salvataggio delle nuvole dei marker rilevati in cloudA e cloudB (opzionale)

		//pcl::io::savePCDFileASCII("markers_cloudA.pcd", *markers_cloudA);
		//pcl::io::savePCDFileASCII("markers_cloudB.pcd", *markers_cloudB);


		// Calcolo delle matrici di trasformazione SVD ed LM
		// http://docs.pointclouds.org/trunk/a01587.html#a69f01afaaaf5dd9c5fc4d44a4b0c7128
		// Esecuzione delle trasformazioni: scriverà in transformation_matrix_LM e _SVD i valori per la trasformazione
		SVD.estimateRigidTransformation(*markers_cloudA, *markers_cloudB, transformation_matrix_SVD);

		cout << "Matrice di trasformazione SVD:\n\n" << transformation_matrix_SVD << endl;
				

		// Rimozione dei punti Nan dalle nuvole di input
		std::vector<int> indices; 
		pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indices); 
		pcl::removeNaNFromPointCloud(*cloudB, *cloudB, indices); 


		/*
		pass.setInputCloud (cloudA);
		pass.setFilterFieldName ("z");
		pass.setFilterLimits (0, 2.5);
		pass.filter (*cloudA);

		pass.setInputCloud (cloudB);
		pass.filter (*cloudB);
		*/

		// Trasformazione con SVD
		cloud_temp.clear();
		pcl::transformPointCloud (*cloudA, cloud_temp, transformation_matrix_SVD);
		cloud_temp += *cloudB;
		pcl::io::savePCDFileASCII("cloudAB_SVD_out.pcd", cloud_temp);





		
		///////////////////////////////////////////////////////////////////////
		// Codice per la visualizzazione delle sfere dei marker sulle nuvole //

		*cloudAB = *cloudA;
		// traslazione della nuvola B di t!!!!!!!!!!!!!!!!!!!!
		Eigen::Matrix4f t;
		t << 1, 0 , 0, 1,
		    0, 1, 0, 0,
			0, 0 ,1, 0,
			0, 0, 0, 1;

		pcl::transformPointCloud(*cloudB, *cloudB, t);
		*cloudAB += *cloudB;

		// Inserisco nei vettori di centri i punti dei marker
		int i;
		for (i=0; i < markers_cloudA->size(); i++){
			centriA[i] = markers_cloudA->at(i);
			centriB[i] = markers_cloudB->at(i);
			centriB[i].x ++;
		}	
		centriA[i].x = std::numeric_limits<float>::infinity ();

		pcl::visualization::CloudViewer viewer("viewer");
		viewer.showCloud(cloudAB);
		viewer.runOnVisualizationThreadOnce (viewerOneOff);
		
		


		//wait for key to be pressed
		cv::waitKey(0); 
		
		
	} catch (std::exception &ex) {
		cout<<"Exception :"<<ex.what()<<endl;
	}

}

