#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;

	string title= "Kinect - Riconoscimento distanza naso-mento";

	ofSetupOpenGL(&window, 1024, 768, OF_WINDOW);

	window.setWindowTitle(title);
	//window.toggleFullscreen();

	ofRunApp(new testApp());
	/*
	in caso di crash improvvisi ci sono molteplici cause di cui tenere conto, una su tutte è lo scarso hardware a disposizione
	poi ci sono problemi relativi alle openGL utilizzate da linux, e poi l'effettiva compatibilità di
	openFramework con il sistema linux e le sue openGL
	*/
}
