

ALLINEAMENTO NUVOLE DI PUNTI (PCL) ATTRAVERSO RILEVAZIONE DI MARKER (ARUCO)


0) PREREQUISITI

Scaricare tutto a 32bit.

PCL 1.6 -> http://www.pointclouds.org/downloads/windows.html
Qt -> http://download.qt-project.org/official_releases/qt/4.8/4.8.5/qt-win-opensource-4.8.5-vs2010.exe
Aruco -> http://sourceforge.net/projects/aruco/files/1.2.4/aruco-1.2.4.tgz/download
OpenCV -> https://sourceforge.net/projects/opencvlibrary/files/opencv-win/2.4.7/opencv-2.4.7.2.exe/download
Visual C++ 2010 Express -> http://go.microsoft.com/?linkid=9709954 
CMake -> http://www.cmake.org/cmake/resources/software.html
OpenNI SDK v1.5.7.10 -> http://www.openni.org/wp-content/uploads/2013/11/OpenNI-Win32-1.5.7.10-Dev.zip
OpenNI-Compliant Sensor Driver v5.1.6.6 -> http://www.openni.org/wp-content/uploads/2013/11/Sensor-Win32-5.1.6.6-Redist.zip
Microsoft Windows SDK for Windows 7 and .NET Framework 4 -> http://download.microsoft.com/download/A/6/A/A6AC035D-DA3F-4F0C-ADA4-37C8E5D34E3D/winsdk_web.exe



1) INSTALLAZIONE

	- Installare PCL 1.6.0 includendo tutte le componenti e aggiungendo il Path al sistema
	- Installare Qt nella cartella predefinita
	- Installare OpenNI SDK nella cartella predefinita
	- Installare OpenNI Sensor Driver nella cartella predefinita
	- Installare Visual Studio 2010 (web installer) e le SDK 7.1 (non serve il Database SQL)
	- Installare CMake e aggiungerlo al Path
	- Installare OpenCV (http://docs.opencv.org/doc/tutorials/introduction/windows_install/windows_install.html)
		a) Estrarre i file in una cartella (OpenCV\sources)
		b) Avviare CMake e selezionare la cartella di Input (OpenCV\sources) e di Output (OpenCV\build)
		c) Configure con compilatore Visual Studio 10
		d) Se si vuole usare OpenNI con OpenCV è necessario abilitare la voce "With_OPENNI" (visibile dopo aver abilitato le opzioni "Advanced" e "Grouped" in CMake)
		e) Generate
		f) Aggiungere nella variabile d'ambiente Path il percorso di OpenCV (per info o errori seguire la guida!)
	- Installare Aruco 
		a) Estrarre i file (Aruco\source) e creare una cartella di Output (Aruco\build)
		b) Seguire il file README, sottosezione Windows (compilare in CMake  e aggiungere eventuali riferimenti mancanti, generate, aprire ALL_BUILD e installare il tutto)
		
		
		
2) COMPILAZIONE FILE DEL PROGETTO

Avviare CMake, cartella di Input contenente i due file .cpp e il file CMakeLists.txt, creare una cartella di output ("build"), configurare e generare il progetto con Visual Studio 10.
Aprire ALL_BUILD e per i due progetti -> click destro -> Proprietà -> Directory di VC++ ed aggiungere:
	- Directory file eseguibili			OpencCV\build\bin\Release 
	- Directory di inclusione			OpencCV\build\install\include e Aruco\source\src
	- Directory librerie				OpencCV\build\lib\Release e Aruco\build\bin\Release
	
Ora andare alla voce Linker -> Input e in Dipendenze aggiuntive aggiungere tutti i nomi delle librerie presenti nel file ELENCO_LIB.
Confermare e provare a compilare i due progetti, in caso di errori verificare i percorsi dei file .h di PCL/Aruco/OpenCV.



3) CREAZIONE DELLA BOARD CON I MARKER

Una volta installato Aruco verranno creati alcuni eseguibili in Aruco\build\bin\Release, l'unico che serve è "aruco_create_board.exe".
Facendolo partire senza argomenti viene mostrato il funzionamento:

			Usage: X:Y boardImage.png boardConfiguration.yml .....
			
Il programma genera due file:
	- "boardImage.png" è l'immagine da stampare della board di X x Y marker
	- "boardConfiguration.yml" rappresenta la board nelle sue coordinate dei vertici, degli ID, etc... e verrà usata per rilevare la board nelle nuvole catturate









