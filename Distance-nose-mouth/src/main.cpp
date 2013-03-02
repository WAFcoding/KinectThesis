#include "testApp.h"
#include "ofAppGlutWindow.h"

int main() {
	ofAppGlutWindow window;
	ofSetupOpenGL(&window, 1024, 700, OF_WINDOW);

	string title= "Kinect - Riconoscimento distanza naso-mento";
	window.setWindowTitle(title);
	window.toggleFullscreen();

	ofRunApp(new testApp());
}
