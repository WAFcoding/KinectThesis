#include "testApp.h"


//--------------------------------------------------------------
void testApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);

	// enable depth->video image calibration
	kinect.setRegistration(true);

	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)

	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #

    float nearClip=500;//default 500
    float farClip=4000;//default 4000

	kinect.setDepthClipping(nearClip, farClip);//non vedo differenze
	float kinect_nearClip= kinect.getNearClipping();
	float kinect_farClip= kinect.getFarClipping();
	//printf("Kinect_nearClip= %f, kinect_farClip= %f\n", kinect_nearClip, kinect_farClip);

	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);

	nearThreshold = 230;
	farThreshold = 70;
	bThreshWithOpenCV = true;

	ofSetFrameRate(60);

	// zero the tilt on startup
	//angle = -20;//salvare su file l'ultima configurazione
	//kinect.setCameraTiltAngle(angle);

	// start from the front
	bDrawPointCloud = false;
	b_DrawClosestPoint = false;

    //PULSANTI---------------------------------
    //rect pcl
	b_pcl_color_over= false;
	b_pcl_press_enable= false;
    //rect exit
	b_exit_color_over= false;
	b_exit_press_enable= false;
    //rect angle up
	b_a_up_color_over= false;
	b_a_up_press_enable= false;
    //rect angle down
	b_a_down_color_over= false;
	b_a_down_press_enable= false;
	//rect clear color
	b_clear_color_over= false;
	b_clear_colol_press_enable= false;
	//step up
	b_step_up_color_over= false;
	b_step_up_press_enable= false;
	//step down
	b_step_down_color_over= false;
	b_step_down_press_enable= false;
	//point size up
	b_point_size_up_color_over= false;
	b_point_size_up_press_enable= false;
	//poin size down
	b_point_size_down_color_over= false;
	b_point_size_down_press_enable= false;
	//recognition
	b_recognition_color_over= false;
	b_recognition_press_enable= false;
    //----------------------------------------

    b_stop_update= false;

    b_clear_colors= false;

	b_draw_pcl_options= false;

	b_enable_distance_recognition= false;

	b_esayCam_enable= false;



}

//--------------------------------------------------------------
void testApp::update() {

	//ofBackground(100, 100, 100);
    ofBackgroundGradient(ofColor::white, ofColor::black, OF_GRADIENT_CIRCULAR);

	//kinect.setUseTexture(true);

    if(!b_stop_update)
        kinect.update();

    //rimuove dalla mesh tutti i vertici, colori e inidici
    //da usare per evitare stackoverflow
    mesh.clear();

	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {

		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);

		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		bThreshWithOpenCV= true;
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			// or we do it ourselves - show people how they can work with the pixels
			unsigned char * pix = grayImage.getPixels();

			int numPixels = grayImage.getWidth() * grayImage.getHeight();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}

		// update the cv images
		grayImage.flagImageChanged();

		// find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
		// also, find holes is set to true so we will get interior contours as well....
		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
	}
}

//--------------------------------------------------------------
void testApp::draw() {

    ofGradientMode mode=OF_GRADIENT_CIRCULAR;
    ofBackgroundGradient(ofColor(250), ofColor(60), mode);

	ofSetColor(255, 255, 255);

    if(b_esayCam_enable) easyCam.enableMouseInput();
    else easyCam.disableMouseInput();


	if(bDrawPointCloud) {

        drawPointCloud();
	}
	else if(b_DrawClosestPoint)
        drawClosestPoint();
	else {
		// draw from the live kinect
		kinect.drawDepth(big_rect_v_w + 10, 10, 400, 300);
		kinect.draw(big_rect_v_w + 420, 10, 400, 300);

		grayImage.draw(big_rect_v_w + 10, 320, 400, 300);
		contourFinder.draw(big_rect_v_w + 10, 320, 400, 300);
	}
    //

    //==========================================================
    //disegna i due rettangoli contenenti info e buttons
    drawInfo();

    drawRectangleMenu();
    //==========================================================
}

void testApp::drawInfo(){

    ofSetColor(ofColor::white);
    stringstream stream;
    stream << "min_vertex: " << min_vertex << endl
           << "first_min: " << first_min << endl
           << "second_min: " << second_min << endl
           << "dimensione punti: " << point_size << endl
           << "step di campionamento: " << step << endl
           << "easycam_distance: " << easyCam.getDistance() << endl
           << "punti totali= " <<  mesh_point << endl
           << "framerate: " << ofGetFrameRate() << endl
           << "mouse: " << mouseX << ", " << mouseY << endl;
    ofDrawBitmapString(stream.str(), 10, 50);

}

void testApp::drawRectangleMenu(){

	//big_rect_vertical
	ofSetColor(big_rect_color_v);
	ofNoFill();
	ofRect(big_rect_v_x, big_rect_v_y, big_rect_v_w, big_rect_v_h);

	//big_rect_vertical_buttons
	ofSetColor(big_rect_color_v);
	ofNoFill();
	ofRect(big_rect_v_b_x, big_rect_v_b_y, big_rect_v_b_w, big_rect_v_b_h);

    //rect_exit
    rect_exit_x= big_rect_v_b_x  + 15;
    rect_exit_y= big_rect_v_b_y + 10;
    rect_exit_string_x= rect_exit_x + 15;
    rect_exit_string_y= rect_exit_y + 20;
    if(b_exit_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
    ofRect(rect_exit_x, rect_exit_y, rect_w, rect_h);
    ofDrawBitmapString("EXIT", rect_exit_string_x, rect_exit_string_y);

    //rect_pcl
    rect_pcl_x= rect_exit_x + rect_w + 15;
    rect_pcl_y= rect_exit_y;
    rect_pcl_string_x= rect_pcl_x + 15;
    rect_pcl_string_y= rect_pcl_y + 20;
    if(b_pcl_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);//il cambio colore dovrebbe farlo in update
    ofRect(rect_pcl_x, rect_pcl_y, rect_w, rect_h);
    ofDrawBitmapString("PCL", rect_pcl_string_x, rect_pcl_string_y);

    //rect angle up
    rect_a_up_x= rect_exit_x;
    rect_a_up_y= rect_exit_y + rect_h + 10;
    rect_a_up_string_x= rect_a_up_x + 15;
    rect_a_up_string_y= rect_a_up_y + 20;
    if(b_a_up_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
    ofRect(rect_a_up_x, rect_a_up_y, rect_w, rect_h);
    ofDrawBitmapString("UP", rect_a_up_string_x, rect_a_up_string_y);

    //rect angle down
    rect_a_down_x= rect_pcl_x;
    rect_a_down_y= rect_a_up_y;
    rect_a_down_string_x= rect_a_down_x + 15;
    rect_a_down_string_y= rect_a_down_y + 20;
    if(b_a_down_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
    ofRect(rect_a_down_x, rect_a_down_y, rect_w, rect_h);
    ofDrawBitmapString("DOWN", rect_a_down_string_x, rect_a_down_string_y);

    //quando è attivo il pcl
    if(b_draw_pcl_options){

        //rect clear color
        rect_clear_color_x= rect_exit_x;
        rect_clear_color_y= rect_a_up_y + rect_h + 10;
        rect_clear_color_string_x= rect_clear_color_x + 15;
        rect_clear_color_string_y= rect_clear_color_y + 20;
        if(b_clear_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_clear_color_x, rect_clear_color_y, rect_w, rect_h);
        ofDrawBitmapString("COLORS", rect_clear_color_string_x, rect_clear_color_string_y);

        //rect step up
        rect_step_up_x= rect_clear_color_x;
        rect_step_up_y= rect_clear_color_y + rect_h + 10;
        rect_step_up_string_x= rect_step_up_x + 15;
        rect_step_up_string_y= rect_step_up_y + 20;
        if(b_step_up_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_step_up_x, rect_step_up_y, rect_w, rect_h);
        ofDrawBitmapString("STEP UP", rect_step_up_string_x, rect_step_up_string_y);

        //rect step down
        rect_step_down_x= rect_pcl_x;
        rect_step_down_y= rect_step_up_y;
        rect_step_down_string_x= rect_step_down_x + 15;
        rect_step_down_string_y= rect_step_down_y + 20;
        if(b_step_down_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_step_down_x, rect_step_down_y, rect_w, rect_h);
        ofDrawBitmapString("STEP DOWN", rect_step_down_string_x, rect_step_down_string_y);

        //rect point size up
        rect_psize_up_x= rect_step_up_x;
        rect_psize_up_y= rect_step_up_y + rect_h + 10;
        rect_psize_up_string_x= rect_psize_up_x + 15;
        rect_psize_up_string_y= rect_psize_up_y + 20;
        if(b_point_size_up_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_psize_up_x, rect_psize_up_y, rect_w, rect_h);
        ofDrawBitmapString("Point UP", rect_psize_up_string_x, rect_psize_up_string_y);

        //rect point size down
        rect_psize_down_x= rect_step_down_x;
        rect_psize_down_y= rect_psize_up_y;
        rect_psize_down_string_x= rect_psize_down_x + 15;
        rect_psize_down_string_y= rect_psize_down_y + 20;
        if(b_point_size_down_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_psize_down_x, rect_psize_down_y, rect_w, rect_h);
        ofDrawBitmapString("Point DOWN", rect_psize_down_string_x, rect_psize_down_string_y);

        //recognition
        rect_recognition_x= rect_psize_up_x;
        rect_recognition_y= rect_psize_up_y + rect_h + 10;
        rect_recognition_string_x= rect_recognition_x + 15;
        rect_recognition_string_y= rect_recognition_y + 20;
        if(b_recognition_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_recognition_x, rect_recognition_y, rect_w, rect_h);
        ofDrawBitmapString("RECOGNITION", rect_recognition_string_x, rect_recognition_string_y);
    }

}

void testApp::drawPointCloud() {

	int w = 640;//640
	int h = 480;//480

    //============================================================================
    //da dichiarare qui altrimenti overflow nell'occupazione di memoria
	mesh;//una struttura tridimensionale
	ofMesh mesh_restricted;

    /*OF_PRIMITIVE_TRIANGLES,
    OF_PRIMITIVE_TRIANGLE_STRIP,
    OF_PRIMITIVE_TRIANGLE_FAN,
    OF_PRIMITIVE_LINES,
    OF_PRIMITIVE_LINE_STRIP,
    OF_PRIMITIVE_LINE_LOOP,
    OF_PRIMITIVE_POINTS*/
    //riempio la struttura con i vertici di quello che il kinect vede
	mesh.setMode(OF_PRIMITIVE_POINTS);
	//mesh_restricted.setMode(OF_PRIMITIVE_POINTS);

	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
		    float tmp= kinect.getDistanceAt(x, y) ;
			if(tmp > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
    //============================================================================

    //============================================================================
	//per disegnare il punto più vicino
	int min= 4000;

	int n = mesh.getNumVertices();

	int far_distance= 1000;
	float nearestDistance = 500;//50 cm

	for(int i = 0; i < n; i++) {

        ofVec3f vertex= mesh.getVertex(i);

        int min_distance= vertex.z - nearestDistance;

		if(min_distance > 0){//siamo dopo nearestDistance

            //per il minimo (blu)
            if(min_distance < min){
                min= min_distance;
                min_vertex.x= vertex.x;
                min_vertex.y= vertex.y;
                min_vertex.z= nearestDistance + min;
                nearest_index= i;
            }

            /*//controllo che siamo nel range assegnato
            //il range è [point.z + nearestDistance, point.z + nearestDistance + far_distance]
            if(min_distance <= far_distance){

                mesh_restricted.addColor(mesh.getColor(i));
                mesh_restricted.addVertex(vertex);
            }*/
            //
		}
		else{//siamo prima nearestDistance

			min_vertex.x= vertex.x;
			min_vertex.y= vertex.y;
			min_vertex.z= nearestDistance;
		}
	}
    //============================================================================


    //============================================================================
	ofPushMatrix();

    easyCam.begin();

    easyCam.setDistance(distance_cam);

	//i puunti sono proiettati sottosopra e al contrario
	ofScale(1, -1, -1);
	ofTranslate(0, 0, translateZ);
	glEnable(GL_DEPTH_TEST);
    //

	if(b_clear_colors){
        mesh.clearColors();
        //mesh_restricted.clearColors();
        ofSetColor(ofColor::black);
	}

    //ofSetColor(ofColor::gray);
    //mesh_restricted.drawWireframe();

	glPointSize(point_size);

    //mesh_restricted.drawVertices();
	mesh.drawVertices();

	//disegno una sfera blu nel punto più vicino
    ofSetColor(0,0,255);
    ofFill();
    ofSphere(min_vertex, 5);

    easyCam.end();

    //deve stare alla fine altrimenti si perdono le modifiche fatte alla struttura
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
    //============================================================================
    //aggiorno il numero dei punti delle due mesh per poterlo stampare a video
    mesh_point= mesh.getNumVertices();
    mesh_restricted_point= mesh_restricted.getNumVertices();
}

void testApp::drawClosestPoint() {
/*
	int w = 640;//640
	int h = 480;//480

	ofMesh mesh;//una struttura tridimensionale
	ofMesh mesh_restricted;

    //riempio la struttura con i vertici di quello che il kinect vede
	mesh.setMode(OF_PRIMITIVE_POINTS);
	mesh_restricted.setMode(OF_PRIMITIVE_POINTS);

	int step = 100;//il parametro per la densità dei punti
	for(int y = 0; y < h; y += step) {
		for(int x = 0; x < w; x += step) {
		    float tmp= kinect.getDistanceAt(x, y) ;
			if(tmp > 0) {
				mesh.addColor(kinect.getColorAt(x,y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	//

	//per disegnare il punto più vicino
	int min_x= 0,
        min_y= 0,
        min_z= 0,
        min= 4000,
        second_min_x= 0,
        second_min_y= 0,
        second_min_z= 0,
        second_min= 4000;

	int n = mesh.getNumVertices();

	int far_distance= 500;
	float nearestDistance = 500;//50 cm

	ofVec3f nearestVertex;
	int nearestIndex;
	//ofVec3f point(0, 0, 0);
	for(int i = 0; i < n; i++) {
		ofVec3f vertex = mesh.getVertex(i);

        int min_distance= vertex.z - nearestDistance;
		if(min_distance > 0){//siamo dopo nearestDistance

            //per il minimo (blu)
            if(min_distance < min){
                min= min_distance;
                min_x= vertex.x;
                min_y= vertex.y;
                min_z= nearestDistance + min;

            }

            //per il secondo minimo (rosso)
            if(min_distance > min && min_distance < second_min){

                second_min= min_distance;
                second_min_x= vertex.x;
                second_min_y= vertex.y;
                second_min_z= nearestDistance + second_min;//vertex.z;
            }

            //printf("min= %d, second_min= %d\n", min, second_min);

            //controllo che siamo nel range assegnato
            //il range è [point.z + nearestDistance, point.z + nearestDistance + far_distance]
            if(min_distance <= far_distance){

                mesh_restricted.addColor(mesh.getColor(i));//mesh_restricted.addColor(kinect.getColorAt(vertex.x, vertex.y));
                mesh_restricted.addVertex(vertex);
            }
		}
		else{//siamo prima nearestDistance

			min_x= vertex.x;
			min_y= vertex.y;
			min_z= nearestDistance;
		}
	}
    //


	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards'
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -500); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POINT_SMOOTH);
	mesh_restricted.drawWireframe();
	glPointSize(2);//forse troppo grandi i punti, da controllare questa cosa
	//mesh.drawVertices();
    mesh_restricted.drawVertices();

	//disegno una sfera blu nel punto più vicino
    ofSetColor(0,0,255);
    ofFill();
    ofSphere(min_x, min_y, min_z, 5);

    //disegno una sfera nel secondo punto più vicino
    ofSetColor(ofColor::red);
    ofFill();
    ofSphere(second_min_x, second_min_y, second_min_z, 5);

    //deve stare alla fine altrimenti si perdono le modifiche fatte alla struttura
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();*/
}

//--------------------------------------------------------------
void testApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {

		case ' ':
			bThreshWithOpenCV = !bThreshWithOpenCV;
			break;

		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;

		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
        //regolano la traslazione lungo z
        case 'q':
            translateZ+= 100;
            break;

        case 'w':
            translateZ-= 100;
            break;
        //regolano la profondità delle riprese
        case 'a':
            distance_cam+= 50;
            break;

        case's':
            distance_cam-= 50;
            break;

	}
}

//--------------------------------------------------------------

void testApp::mouseMoved(int x, int y){

    //exit
    if(x >= rect_exit_x && x <= rect_exit_x+rect_w && y >= rect_exit_y && y <= rect_exit_y + rect_h){

        b_exit_color_over= true;
        b_exit_press_enable= true;
    }
    else if(!(x >= rect_exit_x && x <= rect_exit_x+rect_w && y >= rect_exit_y && y <= rect_exit_y + rect_h)){

        b_exit_color_over= false;
        b_exit_press_enable= false;
    }

    //ancgle up
    if(x >= rect_a_up_x && x <= rect_a_up_x+rect_w && y >= rect_a_up_y && y <= rect_a_up_y + rect_h){

        b_a_up_color_over= true;
        b_a_up_press_enable= true;
    }
    else if(!(x >= rect_a_up_x && x <= rect_a_up_x+rect_w && y >= rect_a_up_y && y <= rect_a_up_y + rect_h)){

        b_a_up_color_over= false;
        b_a_up_press_enable= false;
    }

    //angle down
    if(x >= rect_a_down_x && x <= rect_a_down_x+rect_w && y >= rect_a_down_y && y <= rect_a_down_y + rect_h){

        b_a_down_color_over= true;
        b_a_down_press_enable= true;
    }
    else if(!(x >= rect_a_down_x && x <= rect_a_down_x+rect_w && y >= rect_a_down_y && y <= rect_a_down_y + rect_h)){

        b_a_down_color_over= false;
        b_a_down_press_enable= false;
    }

    //clear color
    if(x >= rect_clear_color_x && x <= rect_clear_color_x+rect_w && y >= rect_clear_color_y && y <= rect_clear_color_y + rect_h){

        b_clear_color_over= true;
        b_clear_colol_press_enable= true;
    }
    else if(!(x >= rect_clear_color_x && x <= rect_clear_color_x+rect_w && y >= rect_clear_color_y && y <= rect_clear_color_y + rect_h)){

        b_clear_color_over= false;
        b_clear_colol_press_enable= false;
    }

    //pcl
    if(x >= rect_pcl_x && x <= rect_pcl_x+rect_w && y >= rect_pcl_y && y <= rect_pcl_y + rect_h){

        b_pcl_color_over= true;
        b_pcl_press_enable= true;
    }
    else if (!(x >= rect_pcl_x && x <= rect_pcl_x+rect_w && y >= rect_pcl_y && y <= rect_pcl_y + rect_h)){

        b_pcl_color_over= false;
        b_pcl_press_enable= false;
    }

    //step up
    if(x >= rect_step_up_x && x <= rect_step_up_x+rect_w && y >= rect_step_up_y && y <= rect_step_up_y + rect_h){

        b_step_up_color_over= true;
        b_step_up_press_enable= true;
    }
    else if(!(x >= rect_step_up_x && x <= rect_step_up_x+rect_w && y >= rect_step_up_y && y <= rect_step_up_y + rect_h)){

        b_step_up_color_over= false;
        b_step_up_press_enable= false;
    }

    //step down
    if(x >= rect_step_down_x && x <= rect_step_down_x+rect_w && y >= rect_step_down_y && y <= rect_step_down_y + rect_h){

        b_step_down_color_over= true;
        b_step_down_press_enable= true;
    }
    else if(!(x >= rect_step_down_x && x <= rect_step_down_x+rect_w && y >= rect_step_down_y && y <= rect_step_down_y + rect_h)){

        b_step_down_color_over= false;
        b_step_down_press_enable= false;
    }

    //point size up
    if(x >= rect_psize_up_x && x <= rect_psize_up_x+rect_w && y >= rect_psize_up_y && y <= rect_psize_up_y + rect_h){

        b_point_size_up_color_over= true;
        b_point_size_up_press_enable= true;
    }
    else if(!(x >= rect_psize_up_x && x <= rect_psize_up_x+rect_w && y >= rect_psize_up_y && y <= rect_psize_up_y + rect_h)){

        b_point_size_up_color_over= false;
        b_point_size_up_press_enable= false;
    }

    //point size down
    if(x >= rect_psize_down_x && x <= rect_psize_down_x+rect_w && y >= rect_psize_down_y && y <= rect_psize_down_y + rect_h){

        b_point_size_down_color_over= true;
        b_point_size_down_press_enable= true;
    }
    else if(!(x >= rect_psize_down_x && x <= rect_psize_down_x+rect_w && y >= rect_psize_down_y && y <= rect_psize_down_y + rect_h)){

        b_point_size_down_color_over= false;
        b_point_size_down_press_enable= false;
    }

    //recognition
    if(x >= rect_recognition_x && x <= rect_recognition_x+rect_w && y >= rect_recognition_y && y <= rect_recognition_y + rect_h){

        b_recognition_color_over= true;
        b_recognition_press_enable= true;
    }
    else if(!(x >= rect_recognition_x && x <= rect_recognition_x+rect_w && y >= rect_recognition_y && y <= rect_recognition_y + rect_h)){

        b_recognition_color_over= false;
        b_recognition_press_enable= false;
    }

}

void testApp::mouseDragged(int x, int y, int button){
}

//--------------------------------------------------------------

void testApp::mousePressed(int x, int y, int button){

    //pcl
    if(b_pcl_press_enable){

        bDrawPointCloud= !bDrawPointCloud;
        b_esayCam_enable= !b_esayCam_enable;
        b_draw_pcl_options= !b_draw_pcl_options;
    }
    //exit
    else if(b_exit_press_enable){

        exit();
        ofExit();
    }
    //TILT UP
    else if(b_a_up_press_enable){
        angle+= 5;
        if(angle>30) angle=30;
        kinect.setCameraTiltAngle(angle);
    }
    //TILT DOWN
    else if(b_a_down_press_enable){
        angle-= 5;
        if(angle<-30) angle=-30;
        kinect.setCameraTiltAngle(angle);
    }
    //clear color
    else if(b_clear_colol_press_enable){
        b_clear_colors= !b_clear_colors;
    }
    //step up
    else if(b_step_up_press_enable){
        step++;
        if(step > 30) step= 30;
        if(point_size > 15) point_size= 15;
    }
    //step down
    else if(b_step_down_press_enable){
        step--;
        if(step < 2) step= 2;
        if(point_size < 2) point_size= 2;
    }
    //point size up
    else if(b_point_size_up_press_enable){
        point_size++;
        if(point_size > 15) point_size= 15;
    }
    //point size down
    else if(b_point_size_down_press_enable){
        point_size--;
        if(point_size < 2) point_size= 2;
    }
    //recognition
    else if(b_recognition_press_enable){
        b_stop_update= !b_stop_update;
    }
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

