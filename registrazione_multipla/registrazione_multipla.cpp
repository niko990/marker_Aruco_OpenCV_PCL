#include <iostream>
#include <sstream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <C:\Users\admin\Desktop\Marker\aruco-1.2.4\src\aruco.h>
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
#define BOARD_MARKERS 24
#define MARKERS_CONV_FACT 0.000768 // 100 pixels equals 76.8mm eq 0.0768 meters
// for aruco calibration see http://www.uco.es/investiga/grupos/ava/node/26


pcl::PointXYZRGBA centriA[BOARD_MARKERS * 4], centriB[BOARD_MARKERS * 4];


// VETTORE DELLE NUVOLE IN INPUT
// std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr>> input;



// VISUALIZZATORE PCD
void viewerOneOff (pcl::visualization::PCLVisualizer& viewer) {
	int correspondence_number = 0;
	std::stringstream ss;
	
	viewer.setBackgroundColor (1.0, 0.5, 1.0);
	viewer.setCameraPosition(0,0,-2,0,-1,1,0);
	viewer.resetCamera();
		
	while( pcl::isFinite(centriA[correspondence_number] )){

		ss << "line" << correspondence_number;
		//cout << "Next correspondence = " << correspondence_number << endl;
		viewer.addSphere(centriA[correspondence_number], 0.02, 255, 0, 0, ss.str(), 0);
		ss << "23";
		//viewer.addSphere(centriB[correspondence_number], 0.02, 0, 255, 0, ss.str(), 0);
		//cout << "\nAggiungo nuova sfera: " << centriA[correspondence_number] << " - " << centriB[correspondence_number] << endl;
		//ss << "d";
		//viewer.addLine(centriA[correspondence_number], centriB[correspondence_number], 0, 0, 255, ss.str(), 0);
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


// RICERCA DEL PUNTO 3D NELLA NUVOLA IN INPUT DATE X,Y
// RETURN pcl::PointXYZRGBA
pcl::PointXYZRGBA interpolatePoint (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud, float i, float j) {
	
	int i0 = (int)i;
	int j0 = (int)j;
    float di = i - i0;
	float dj = j - j0;

	int k0 = cloud->width * j0 + i0;

	// http://fastcpp.blogspot.it/2011/06/bilinear-pixel-interpolation-using-sse.html
	pcl::PointXYZRGBA p0 = cloud->at(i0,j0);
	pcl::PointXYZRGBA p1 = cloud->at(i0+1,j0);
	pcl::PointXYZRGBA p2 = cloud->at(i0,j0+1);	
	pcl::PointXYZRGBA p3 = cloud->at(i0+1,j0+1);	
	pcl::PointXYZRGBA p;
	
	// Pesi di ogni punto vicino
	float w0 = (1-di) * (1-dj);
	float w1 = di * (1-dj);
	float w2 = (1-di) * dj;
	float w3 = di * dj;

	p.x = p0.x*w0 + p1.x*w1 + p2.x*w2 + p3.x*w3;
	p.y = p0.y*w0 + p1.y*w1 + p2.y*w2 + p3.y*w3;
	p.z = p0.z*w0 + p1.z*w1 + p2.z*w2 + p3.z*w3;
	p.r = p0.r*w0 + p1.r*w1 + p2.r*w2 + p3.r*w3;
	p.g = p0.g*w0 + p1.g*w1 + p2.g*w2 + p3.g*w3;
	p.b = p0.b*w0 + p1.b*w1 + p2.b*w2 + p3.b*w3;
	p.a = p0.a*w0 + p1.a*w1 + p2.a*w2 + p3.a*w3;

	if(p.x != std::numeric_limits<float>::quiet_NaN () && p.x != std::numeric_limits<float>::infinity())
		return p;
	else
		return getNearestPoint(cloud,i,j);

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

// modifica parametri in (int argc, char **argv)
int LoadInput(char** argv, BoardConfiguration TheBoardConfig, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudB){



		/* 
		// VETTORE DI NUVOLE
	    for(int i=1; i<argc; i++){
			input.push_back( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>() ) );
			pcl::io::loadPCDFile(argv[i], *input[i-1]);
			PCL_INFO("cloud %s loaded \n", argv[i]);
			}
		}

		*/
	



	
		
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


// Load Board data and a set of 3D images (as organized PCD).
// For each image search for board matching simbols and align 3D, adding the point cloud to a big one
// containing all the aligned data.
// The final coordinate framework is that of the "virtual" board, which is thinked to be placed on the xy plane


int main(int argc,char **argv) {
	try	{
		if (argc < 3) { cerr << "USO: registrazione_multipla boardConfig.yml pc1.pcd [, pc2.pcd, pc3.pcd...] " << endl; exit(0);}
		
		// log file http://www.codeproject.com/Questions/97485/how-to-write-log-file-in-C
		std::ofstream log_file("mreg.log", std::ios_base::out | std::ios_base::app );
		log_file << "[START LOG]\n";

		MarkerDetector MDetectorA;
		vector<Marker> MarkersA;
		BoardConfiguration TheBoardConfig;
		BoardDetector TheBoardDetector;
		Board TheBoardDetected;
		// current and output clouds
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGBA>);
		// point clouds used for svd
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr markers_cloudA (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr markers_modelboard (new pcl::PointCloud<pcl::PointXYZRGBA>);
		// only for visualization
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudAB (new pcl::PointCloud<pcl::PointXYZRGBA>);

		pcl::PointCloud<pcl::PointXYZRGBA> cloud_temp;

		pcl::PassThrough<pcl::PointXYZRGBA> pass;	
		cv::Mat InImageA;
		int IDcomuni[BOARD_MARKERS], pos = 0;

		int visualize_data = 0; // number of cloud to visualize (0,1,2...)


		// LETTURA DEGLI ARGOMENTI IN INPUT
		//if (LoadInput (argv, TheBoardConfig, cloudA, cloudB) == -1) { cout << "\nExit!"; exit (0);}

		// load table
		log_file << "Read board...";
	    TheBoardConfig.readFromFile(argv[1]);
		log_file << "done/n";

		// iterate over point clouds
		//int ipc= 0;
		for (int ipc=0;ipc<argc-2;ipc++){
		
			cout << endl<< "***************" << endl;
			cout << "    CLOUD  " << ipc << endl;
			cout << "***************" << endl;

			log_file << endl << " CLOUD  " << ipc << endl << endl;
	    
			if (pcl::io::loadPCDFile<pcl::PointXYZRGBA> (argv[ipc+2], *cloudA) == -1) {
				PCL_ERROR ("Errore apertura pcd file!\n");
				log_file << ("[ERROR] Errore apertura pcd file!\n");
				continue;
			}

		
			// Conversione da PCD a immagine per elaborazione con Aruco (LE NUVOLE DEVONO ESSERE ORGANIZZATE)
			InImageA = pcd2img(cloudA);


			// INIZIO PROCEDURA DI DETECTING DEI MARKER

			MDetectorA.detect(InImageA,MarkersA);


			// Stampa info su immagine A
			log_file << "\n*** MARKERS IMMAGINE A ***\n";
			for(unsigned int i=0;i<MarkersA.size();i++){
				log_file << MarkersA[i].id << endl;
					
				for (int v=0 ; v<4 ; v++){
					log_file << "(" << MarkersA[i][v].x << "," << MarkersA[i][v].y << ") - ";
				}
				log_file << endl;
			}

		
				

			//Visualizza immagini con informazioni sui Marker
			if (ipc == visualize_data){
				for(unsigned int i=0;i<MarkersA.size();i++){
					for (int v=0 ; v<4 ; v++){
						MarkersA[i].draw(InImageA,cv::Scalar(0,0,255),2);
					}
				}
				cv::imshow("inA",InImageA);
			}
		

			// RICERCA DELLE CORRISPONDENZE DEGLI ID NEI VETTORI MarkersA e MarkersB

		
			//// IDcomuni inizializzato a tutti 0
			//for (int i=0; i < BOARD_MARKERS; i++)
			//	IDcomuni[i] = -1;

			//// Inizio ricerca, gli ID sono in ordine crescente
			//for (int i = 0; i < MarkersA.size(); i++){
			//	for (int j = 0; j <	MarkersB.size(); j++){
			//			
			//		if (MarkersA[i].id != MarkersB[j].id)
			//			continue;

			//	IDcomuni[pos] = MarkersA[i].id;
			//	pos++;
			//	break;
			//	}
			//}

			//// stampo il vettore
			//cout << "\nVettore degli IDcomuni" << endl;
			//for (int i = 0; IDcomuni[i] != -1; i++)
			//	cout << "IDcomuni[" << i << "] = " << IDcomuni[i] <<  endl;





			// FILL CORRISPONDENCES DATA USING IMAGE MARKERS DATA AND BOARD DATA
			markers_cloudA->clear();
			markers_modelboard->clear();
		
			for (int i = 0; i < MarkersA.size(); i++){

				int j=TheBoardConfig.getIndexOfMarkerId(MarkersA[i].id);

				log_file << "step => i= " << i << " j= " << j << endl; 
				for (int v = 0; v < 4; v++) {
					markers_cloudA->push_back ( interpolatePoint(cloudA, MarkersA[i][v].x, MarkersA[i][v].y) );
					// put z axis in the "up direction" and swap x - y axes;
					pcl::PointXYZRGBA pnt;
					pnt.x=-TheBoardConfig[j][v].y * MARKERS_CONV_FACT;
					pnt.y=-TheBoardConfig[j][v].x * MARKERS_CONV_FACT;
					pnt.z=-TheBoardConfig[j][v].z * MARKERS_CONV_FACT;
					markers_modelboard->push_back(pnt);

					log_file << "\tVertice " << v << "\t id =" << MarkersA[i].id << endl;
					log_file << "\t\tCoordinate Cloud = " << markers_cloudA->at(i*4+v).x << ", "<< markers_cloudA->at(i*4+v).y << ", " << markers_cloudA->at(i*4+v).z << endl; 
					log_file << "\t\tCoordinate Board = " << markers_modelboard->at(i*4+v).x << ", "<< markers_modelboard->at(i*4+v).y << ", " << markers_modelboard->at(i*4+v).z << endl; 
				}


			}



			// Creazione delle due matrici di trasformazione (con i metodi SVD e LM)
			Eigen::Matrix4f transformation_matrix_SVD; 
			// Creazione delle trasformazioni con il metodo LM e SVD
			pcl::registration::TransformationEstimationSVD <pcl::PointXYZRGBA, pcl::PointXYZRGBA> SVD; 

	
			// Salvataggio delle nuvole dei marker rilevati in cloudA e cloudB (opzionale)

			//pcl::io::savePCDFileASCII("markers_cloudA.pcd", *markers_cloudA);


			// Calcolo delle matrici di trasformazione SVD ed LM
			// http://docs.pointclouds.org/trunk/a01587.html#a69f01afaaaf5dd9c5fc4d44a4b0c7128
			// Esecuzione delle trasformazioni: scriverà in transformation_matrix_LM e _SVD i valori per la trasformazione
			SVD.estimateRigidTransformation(*markers_cloudA, *markers_modelboard, transformation_matrix_SVD);

			log_file << "Matrice di trasformazione SVD:\n\n" << transformation_matrix_SVD << endl;
				

			// Rimozione dei punti Nan dalle nuvole di input
			std::vector<int> indices; 
			pcl::removeNaNFromPointCloud(*cloudA, *cloudA, indices); 


			/*
			pass.setInputCloud (cloudA);
			pass.setFilterFieldName ("z");
			pass.setFilterLimits (0, 2.5);
			pass.filter (*cloudA);

			*/

			// Trasformazione con SVD
			cloud_temp.clear();
			pcl::transformPointCloud (*cloudA, cloud_temp, transformation_matrix_SVD);
			(*cloud_out) += cloud_temp;

			




		if (ipc == visualize_data){
		
			///////////////////////////////////////////////////////////////////////
			// Codice per la visualizzazione delle sfere dei marker sulle nuvole //

			*cloudAB = *cloudA;
			// traslazione della nuvola B di t!!!!!!!!!!!!!!!!!!!!
			Eigen::Matrix4f t;
			t << 1, 0 , 0, 1,
				0, 1, 0, 0,
				0, 0 ,1, 0,
				0, 0, 0, 1;

			//pcl::transformPointCloud(*cloudB, *cloudB, t);
			//*cloudAB += *cloudB;

			// Inserisco nei vettori di centri i punti dei marker
			int i;
			for (i=0; i < markers_cloudA->size(); i++){
				centriA[i] = markers_cloudA->at(i);
				centriB[i] = markers_modelboard->at(i);
				centriB[i].x = centriB[i].x + 1; // shift a meter to visualize separately
			}	
			centriA[i].x = std::numeric_limits<float>::infinity ();

			pcl::visualization::CloudViewer viewer("viewer");
			viewer.showCloud(cloudAB);
			viewer.runOnVisualizationThreadOnce (viewerOneOff);
		
		}	

		} // close for cycle



		pcl::io::savePCDFileASCII("cloudAB_SVD_out.pcd", *cloud_out);

		log_file << "Saved output cloud" << std::endl;
		log_file << "[END LOG]" << std::endl;
		cout << "Ok./n"
		//~log_file(); //log_file.close();

		//wait for key to be pressed
		cv::waitKey(0); 
		
		
	} catch (std::exception &ex) {
		cout<<"Exception :"<<ex.what()<<endl;
	}

}

