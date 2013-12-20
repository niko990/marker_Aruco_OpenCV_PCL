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
#include <pcl\registration\transformation_estimation_svd.h>
#include <pcl\registration\transformation_estimation.h>
#include <pcl\common\transformation_from_correspondences.h>


//using namespace cv;
using namespace aruco; 
//using namespace pcl;

/*
struct marker_struct{
	int id;
	pcl::PointXYZRGBA v1;
	pcl::PointXYZRGBA v2;
	pcl::PointXYZRGBA v3;
	pcl::PointXYZRGBA v4;
};*/

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


// PROCEDURA DI RICERCA DEL PUNTO 3D CORRISPONDENTE ALLE COORDINATE (x,y) RILEVATE DA ARUCO
// RITORNA point.x = -1 SE NON è STATO TROVATA NESSUNA CORRISPONDENZA
void getNearestPoint 
	(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr output_markers_cloud, float i, float j) {
	
	int k = (cloud->width * (int)(j + 0.5) ) + (int)(i + 0.5);
	pcl::PointXYZRGBA p = cloud->points[k];

 
	// Controllo se p è un NaN o infinito e lo ritorno se è un valore corretto
	if (!(p.x == std::numeric_limits<float>::quiet_NaN ()) && !(p.x == std::numeric_limits<float>::infinity())){
		//cout << "\n getNearestPoint - Indice punto = " << k << " - " << p << "\n";

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
			//cout << "\n getNearestPoint (seconda scelta) - Indice punto = " << k << " - " << p << "\n";
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


// RICERCA DEL PUNTO 3D NELLA NUVOLA IN INPUT DATE X,Y
// RETURN EIGEN 3f
Eigen::Vector3f getNearestPoint3f (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, float i, float j) {
	
	int k = (cloud->width * (int)(j + 0.5) ) + (int)(i + 0.5);
	//pcl::PointXYZRGBA p = cloud->points[k];
	Eigen::Vector3f v (cloud->points[k].x, cloud->points[k].y, cloud->points[k].z);
	
	//cout << "ritorno " << v << endl;
	
	/*
	v.x << cloud->points[k].x;
	v.y << cloud->points[k].y;
	v.z << cloud->points[k].z;
	*/
	/*
	// Controllo se p è un NaN o infinito e lo ritorno se è un valore corretto
	if (!(v.x == std::numeric_limits<float>::quiet_NaN ()) && !(v.x == std::numeric_limits<float>::infinity())){
		//cout << "\n getNearestPoint - Indice punto = " << k << " - " << p << "\n";

		// inserisci nella nuvola di marker di output passata il primo punto scelto
		//output_markers_cloud->points[k] = p;

		return v;
	}
	else {
		// altrimenti calcolo l'errore commesso dall'approssimazione e cerco un punto nell'intorno
		// di p0 che è più vicino e lo ritorno (controllando che non sia NaN o inf)
		float di = i - (int)(i + 0.5);
		float dj = j - (int)(j + 0.5);
	
		if ( abs(di) <= abs(dj)){
			if (di > 0){
				v.x = cloud->points[k + 1].x;
				v.y = cloud->points[k + 1].y;
				v.z = cloud->points[k + 1].z;
			} else {
				v.x = cloud->points[k - 1].x;
				v.y = cloud->points[k - 1].y;
				v.z = cloud->points[k - 1].z;
			}
		}
		else {
			if (dj > 0){
				v.x = cloud->points[k + cloud->width].x;
				v.y = cloud->points[k + cloud->width].y;
				v.z = cloud->points[k + cloud->width].z;
			} else {
				v.x = cloud->points[k - cloud->width].x;
				v.y = cloud->points[k - cloud->width].y;
				v.z = cloud->points[k - cloud->width].z;
			}
		}

		if (!(v.x == std::numeric_limits<float>::quiet_NaN ()) && !(v.x == std::numeric_limits<float>::infinity())){
			//cout << "\n getNearestPoint (seconda scelta) - Indice punto = " << k << " - " << p << "\n";
		}
		else{
			cout << "Error: non è stato possibile trovare una corrispondenza 3D per il punto (" << i << "," << j << ")\n";
			v.x = -1;
		}

		// inserisci nella nuvola di marker di output passata il secondo punto scelto
		//output_markers_cloud->points[k] = p;
	}
	*/
	return v;

}



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





////////////////////////////////////////////////////////////////////////////////////////////////////////
///////     ERRORE !!!!!!!!!!!!!!!   FUNZIONE DA SISTEMARE !!!!!!!!!!!!!!!!!!!!!!!!!!!
////////////////////////////////////////////////////////////////////////////////////////////////////////
// Ritorna un vettore di interi contenente gli ID ordinati (crescenti) dei marker persenti in entrambe le nuvole
/*void ricercaCorrispondenzeID (std::vector<Marker> markerA, std::vector<Marker> markerB, int IDcomuni[]){
	//std::vector<int> IDcomuni;
	
	//IDcomuni->resize(min(markerA.size(), markerB.size()));
	IDcomuni->resize(min(markerA.size(), markerB.size()));
	int pos = 0, validID = 0;

	for (int i = 0; i < markerA.size();; i++){
		for (int j = 0; j <	markerB.size();; j++){
					
			if (markerA[i].id != markerB[j].id)
				continue;
			
			//cout << "i = " << i << " j = " << j << " - markerA[i].id = " << markerA[i].id << " marker[j].id = " << markerB[j].id << " * Uguale? "<< (markerA[i].id == markerB[j].id) << endl;

			IDcomuni[pos] = markerA[i].id;
			pos++;
			break;
		}
	}

	/*
	for (int i=0;i<IDcomuni.size();i++){
		if (IDcomuni[i] != 0)
			validID++;
		cout << "IDcomuni[" << i << "] = " <<IDcomuni[i]<< endl;
	}

	cout << "resize a -> " << validID << endl; 
	
	IDcomuni->resize(validID);

		for (int i=0;i<IDcomuni->size();i++)
			cout << "IDcomuni[" << i << "] = " <<IDcomuni[i]<< endl;

	//return IDcomuni;
}
*/









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
		cv::Mat InImageA, InImageB;
		pcl::PassThrough<pcl::PointXYZRGBA> pass;
		//std::vector<int> IDcomuni ;
		//int IDcomuni[];
		

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








		/*
		cout << "RICERCA DELLE CORRISPONDENZE!\n";
		cout << "#markersA = " << MarkersA.size() << ", #markersB = " << MarkersB.size() << ", IDcomuni.size() = " << IDcomuni.size() << endl;
		// Vettore contenente tutti gli ID dei marker presenti in entrambe le nuvole
		int IDcomuni [MarkersA.size()];
		ricercaCorrispondenzeID (MarkersA, MarkersB, IDcomuni);


		cout << "#markersA = " << MarkersA.size() << ", #markersB = " << MarkersB.size() << ", IDcomuni.size() = " << IDcomuni.size() << endl;
		for (int i = 0; i < IDcomuni.size(); i++)
		std::cout << "IDcomuni = " << IDcomuni[i] << endl;
		*/


		// RICERCA DELLE COSSIRPONDENZE DEGLI ID NEI VETTORI MarkersA e MarkersB
		const int IDcomuni_size = 20;
		int IDcomuni[IDcomuni_size];
		int pos = 0, validID = 0;

		// IDcomuni inizializzato a tutti 0
		for (int i=0; i < IDcomuni_size; i++)
			IDcomuni[i] = 0;

		// Inizio ricerca, gli ID sono ordine crescente
		for (int i = 0; i < MarkersA.size(); i++){
			for (int j = 0; j <	MarkersB.size(); j++){
					
				if (MarkersA[i].id != MarkersB[j].id)
					continue;
			cout << "i = " << i << " j = " << j << " - markerA[i].id = " << MarkersA[i].id << " marker[j].id = " << MarkersB[j].id << " * Uguale? "<< (MarkersA[i].id == MarkersB[j].id) << endl;
			IDcomuni[pos] = MarkersA[i].id;
			pos++;
			break;
			}
		}

		//cout << "min (A, B) = " << min(MarkersA.size(), MarkersB.size()) << endl;

		// Tronco il vettore alla lunghezza esatta del totale degli elementi trovati
		for (int i=0; i < IDcomuni_size ;i++){
			if (IDcomuni[i] == 0) {
				IDcomuni[i] = '\0';
				break;
			}
		}

		// stampo il vettore
		for (int i = 0; IDcomuni[i]; i++)
			cout << "IDcomuni[" << i << "] = " << IDcomuni[i] <<  endl;










		// Stampa info su immagine A
		std::cout << "\n*** MARKERS IMMAGINE A ***\n";
		for(unsigned int i=0;i<MarkersA.size();i++){
			//cout << MarkersA[i].id << endl;
			
			//markersA_struct[i].id = MarkersA[i].id;
			
			for (int v=0 ; v<4 ; v++){
				//cout << "(" << MarkersA[i][v].x << "," << MarkersA[i][v].y << ") - ";
				//getNearestPoint (cloudA, markers_cloudA, MarkersA[i][v].x, MarkersA[i][v].y);

				//markersA_struct[i].v1 = getNearestPoint (cloudA, MarkersA[i][v].x, MarkersA[i][v].y);
			}
			//cout << endl;
			MarkersA[i].draw(InImageA,cv::Scalar(0,0,255),2);
		}
		
		// Stampa info su immagine B
		std::cout << "\n*** MARKERS IMMAGINE B ***\n";
		for(unsigned int i=0;i<MarkersB.size();i++){
			//cout << MarkersB[i].id << endl;
			//markersB_struct[i].id = MarkersB[i].id;

			for (int v=0 ; v<4 ; v++){
				//cout << "(" << MarkersB[i][v].x << "," << MarkersB[i][v].y << ") - ";	
				//getNearestPoint (cloudB, markers_cloudB, MarkersB[i][v].x, MarkersB[i][v].y);

				//markersB_struct[i].v1 = getNearestPoint (cloudA, MarkersB[i][v].x, MarkersB[i][v].y);
			}
			//cout << endl;
			MarkersB[i].draw(InImageB,cv::Scalar(0,0,255),2);
		}
		
		// Salvataggio delle due nuvole di marker
		//pcl::io::savePCDFileASCII ("markers_cloudA.pcd", *markers_cloudA);
		//pcl::io::savePCDFileASCII ("markers_cloudB.pcd", *markers_cloudB);
		

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




		// INIZIO PROCEDURA DI TRASFORMAZIONE MEDIANTE LE CORRISPONDENZE TROVATE
		
		// Creazione dell'oggetto corrispondenze
		pcl::TransformationFromCorrespondences correspondences;
		// Creazione trasformazione SVD
		pcl::registration::TransformationEstimationSVD <pcl::PointXYZRGBA, pcl::PointXYZRGBA> SVD;
		// Creazione matrice di trasformazione
		Eigen::Matrix4f transformation_matrix;


		int idA = 0, idB = 0;
		for (int i = 0; i < IDcomuni[i]; i++){
			cout << "cerco per IDcomuni[" << i << "]" << endl;
			if (MarkersA[idA].id == MarkersB[idB].id){
				// trovata corrispondenza tra 2 ID --> aggiungo in ordine i punti dei 4 vertici alle corrispondenze
				for (int v = 0; v < 4; v++) {
					correspondences.add ((getNearestPoint3f(cloudA, MarkersA[idA][v].x, MarkersA[idA][v].y)), 
														 getNearestPoint3f(cloudB, MarkersB[idB][v].x, MarkersB[idB][v].y), 1.0);
					//cout << "Aggiunta corrispondenza #" << i << ": ID= " << MarkersA[idA].id << endl;
					cout << "Aggiunta corrispondenza: ID = " << MarkersA[idA].id << " - Vertice " << v << endl;
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

		cout << "\nSono state aggiunte " << correspondences.getNoOfSamples() << " corrispondenze!\n";	




		//transformation_matrix =	correspondences.getTransformation().matrix();
		// Esecuzione del metodo che scriverà in transformation_matrix i valori per la trasformazione
		//SVD.estimateRigidTransformation (&markers_cloudA, &markers_cloudB, correspondences, transformation_matrix);
		//SVD.estimateRigidTransformationSVD (markers_cloudA, markers_cloudB, correspondences, transformation_matrix);
		//cout << transformation_matrix << endl;
		


		//Eigen::Matrix4f trans; 
		//pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA,pcl::PointXYZRGBA> trans_est; 
		//trans_est.estimateRigidTransformation(*markers_cloudA,*markers_cloudB,correspondences, trans); 

		/*
		pcl::PointCloud<pcl::PointXYZRGBA> cloud_out;// (new pcl::PointCloud<pcl::PointXYZRGBA>);	
		pcl::transformPointCloud (*markers_cloudA, cloud_out, trans);

		pcl::io::savePCDFileASCII("cloud_out.pcd", cloud_out);*/








		/*
		pcl::visualization::PCLVisualizer visCorr; 
        //visCorr.addPointCloud(*markers_cloudA,"src"); 
        visCorr.addPointCloud(cloud_out.makeShared(),"trgt"); 
        visCorr.resetCamera(); 
        visCorr.spin(); 
		*/

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