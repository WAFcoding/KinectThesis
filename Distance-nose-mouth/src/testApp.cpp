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
	//openGL
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_DEPTH_TEST);

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
    //sposta il first_min
    b_first_min_color_over= false;
    b_first_min_press_enable= false;
    //sposta il second_min
    b_second_min_color_over= false;
    b_second_min_press_enable= false;

    //freccie direzionali
    b_arrow_up_color_over= false;
    b_arrow_up_press_enable= false;

    b_arrow_down_color_over= false;
    b_arrow_down_press_enable= false;

    b_arrow_right_color_over= false;
    b_arrow_right_press_enable= false;

    b_arrow_left_color_over= false;
    b_arrow_left_press_enable= false;

    b_arrow_in_color_over= false;
    b_arrow_in_press_enable= false;

    b_arrow_out_color_over= false;
    b_arrow_out_press_enable= false;
    //----------------------------------------

    b_stop_update= false;

    b_clear_colors= false;

	b_draw_pcl_options= false;

	b_enable_distance_recognition= false;

	b_esayCam_enable= false;

    b_arrow_color_first_min= false;
    b_arrow_color_second_min= false;
    b_move_first_min= false;
    b_move_second_min= false;

    //=================================================
    //test
    //=================================================
}

//--------------------------------------------------------------
void testApp::update() {

    big_rect_v_b_r_x=  ofGetViewportWidth() - 250 - 5;
    //distanza naso mento
    distance= first_min.distance(second_min);
    //la distanza della proiezione della camera
    easyCam.setDistance(distance_cam);
    //la dimensione dei punti nel pcl
	glPointSize(point_size);
	//ofBackground(100, 100, 100);
    ofBackgroundGradient(ofColor::white, ofColor::black, OF_GRADIENT_CIRCULAR);

    big_rect_v_b_h= ofGetViewportHeight() - 5 - big_rect_v_h - 5 - 5;
    big_rect_v_b_r_x= ofGetViewportWidth() - 250 - 5;
    big_rect_easycam_w= big_rect_v_b_r_x - 5 - big_rect_v_b_w - 5 - 5;
    big_rect_easycam_h= ofGetViewportHeight() - 5 - 5;
    big_rect_v_b_l_f_h= big_rect_v_b_h - 1;
    big_rect_v_b_r_f_x= big_rect_v_b_r_x + 1;

	//kinect.setUseTexture(true);

    if(!b_stop_update)
        kinect.update();

	w = kinect.getWidth();//640
	h = kinect.getHeight();//480

    //rimuove dalla mesh tutti i vertici, colori e indici
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
    stream << "min index: " << min_index << endl
           << "min_vertex: " << min_vertex << endl
           << "first_min: " << first_min << endl
           << "first index: " << first_index << endl
           << "second_min: " << second_min << endl
           << "second index: " << second_index << endl
           << "distanza naso-mento: " << distance << endl
           << "dimensione punti: " << point_size << endl
           << "step di campionamento: " << step << endl
           << "easycam_distance: " << easyCam.getDistance() << endl
           << "verici totali " <<  mesh.getNumVertices() << endl
           << "indici totali: " << mesh.getNumIndices() << endl
           << "framerate: " << ofGetFrameRate() << endl
           << "posizione mouse: " << mouseX << ", " << mouseY << endl
           << "kinect width: " << w << endl
           << "kinect height: " << h << endl;
    ofDrawBitmapString(stream.str(), 10, 50);

}

void testApp::drawRectangleMenu(){

    //stringstream stream;

	//big_rect_vertical
	ofSetColor(big_rect_color_v);
	ofNoFill();
	ofRect(big_rect_v_x, big_rect_v_y, big_rect_v_w, big_rect_v_h);

	//big_rect_vertical_buttons left
	ofRect(big_rect_v_b_l_x, big_rect_v_b_l_y, big_rect_v_b_w, big_rect_v_b_h);

	//rect_exit
    rect_exit_x= big_rect_v_b_l_x  + 15;
    rect_exit_y= big_rect_v_b_l_y + 10;
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

    /*ofSetColor(ofColor::white);
    stream << "regola tilt";
    ofDrawBitmapString(stream.str(), rect_exit_x, rect_exit_y + rect_h + 20);
    stream.clear();*/

    //rect angle up
    rect_a_up_x= rect_exit_x;
    rect_a_up_y= rect_exit_y + rect_h + 10;
    rect_a_up_string_x= rect_a_up_x + 15;
    rect_a_up_string_y= rect_a_up_y + 20;
    if(b_a_up_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
    ofRect(rect_a_up_x, rect_a_up_y, rect_w, rect_h);
    ofDrawBitmapString("TILT UP", rect_a_up_string_x, rect_a_up_string_y);

    //rect angle down
    rect_a_down_x= rect_pcl_x;
    rect_a_down_y= rect_a_up_y;
    rect_a_down_string_x= rect_a_down_x + 15;
    rect_a_down_string_y= rect_a_down_y + 20;
    if(b_a_down_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
    ofRect(rect_a_down_x, rect_a_down_y, rect_w, rect_h);
    ofDrawBitmapString("TILT DOWN", rect_a_down_string_x, rect_a_down_string_y);

    //quando è attivo il pcl
    if(b_draw_pcl_options){

        //rect step up
        rect_step_up_x= rect_exit_x;
        rect_step_up_y= rect_a_up_y + rect_h + 10;
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
        ofDrawBitmapString("POINT UP", rect_psize_up_string_x, rect_psize_up_string_y);

        //rect point size down
        rect_psize_down_x= rect_step_down_x;
        rect_psize_down_y= rect_psize_up_y;
        rect_psize_down_string_x= rect_psize_down_x + 15;
        rect_psize_down_string_y= rect_psize_down_y + 20;
        if(b_point_size_down_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_psize_down_x, rect_psize_down_y, rect_w, rect_h);
        ofDrawBitmapString("POINT DOWN", rect_psize_down_string_x, rect_psize_down_string_y);

        //recognition
        rect_recognition_x= rect_psize_up_x;
        rect_recognition_y= rect_psize_up_y + rect_h + 10;
        rect_recognition_string_x= rect_recognition_x + 5;
        rect_recognition_string_y= rect_recognition_y + 20;
        if(b_recognition_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_recognition_x, rect_recognition_y, rect_w, rect_h);
        ofDrawBitmapString("RECOGNITION", rect_recognition_string_x, rect_recognition_string_y);

        //rect clear color
        rect_clear_color_x= rect_step_down_x;
        rect_clear_color_y= rect_recognition_y;
        rect_clear_color_string_x= rect_clear_color_x + 2;
        rect_clear_color_string_y= rect_clear_color_y + 20;
        if(b_clear_color_over) ofSetColor(rect_color_over); else ofSetColor(rect_color);
        ofRect(rect_clear_color_x, rect_clear_color_y, rect_w, rect_h);
        ofDrawBitmapString("CLEAR COLORS", rect_clear_color_string_x, rect_clear_color_string_y);

        ofNoFill();
        //big_rect_vertical_buttons right
        ofSetColor(big_rect_color_v);
        ofRect(big_rect_v_b_r_x, big_rect_v_b_r_y, big_rect_v_b_w, big_rect_v_b_h);

        //big_rect_easycam
        ofRect(big_rect_easycam_x, big_rect_easycam_y, big_rect_easycam_w, big_rect_easycam_h);

        //first_min
        rect_first_min_x= big_rect_v_b_r_x + 15;
        rect_first_min_y= big_rect_v_b_r_y + 10;
        rect_first_min_string_x= rect_first_min_x + 10;
        rect_first_min_string_y= rect_first_min_y + 20;
        if(b_first_min_color_over) ofSetColor(rect_first_min_color_over); else ofSetColor(rect_color);
        ofRect(rect_first_min_x, rect_first_min_y, rect_w, rect_h);
        ofDrawBitmapString("FIRST MIN", rect_first_min_string_x, rect_first_min_string_y);

        //second_min
        rect_second_min_x= rect_first_min_x + rect_w + 15;
        rect_second_min_y= rect_first_min_y;
        rect_second_min_string_x= rect_second_min_x + 10;
        rect_second_min_string_y= rect_second_min_y + 20;
        if(b_second_min_color_over) ofSetColor(rect_second_min_color_over); else ofSetColor(rect_color);
        ofRect(rect_second_min_x, rect_second_min_y, rect_w, rect_h);
        ofDrawBitmapString("SECOND MIN", rect_second_min_string_x, rect_second_min_string_y);

        //imposto i colori per le arrow
        ofSetColor(rect_color);
        if(b_arrow_color_first_min) ofSetColor(rect_first_min_color_over);
        if(b_arrow_color_second_min) ofSetColor(rect_second_min_color_over);
        ofSetLineWidth(3);

        //arrow up
        rect_arrow_up_x= big_rect_v_b_r_x + 15;
        rect_arrow_up_y= rect_first_min_y + rect_h + 20;
        rect_arrow_up_string_x= rect_arrow_up_x + 15;
        rect_arrow_up_string_y= rect_arrow_up_y + 20;
        ofRect(rect_arrow_up_x, rect_arrow_up_y, rect_arrow_w, rect_arrow_h);
        ofDrawBitmapString("UP", rect_arrow_up_string_x, rect_arrow_up_string_y);

        //arrow left
        rect_arrow_left_x= rect_arrow_up_x + rect_arrow_w + 15;
        rect_arrow_left_y= rect_arrow_up_y;
        rect_arrow_left_string_x= rect_arrow_left_x + 5;
        rect_arrow_left_string_y= rect_arrow_left_y + 20;
        ofRect(rect_arrow_left_x, rect_arrow_left_y, rect_arrow_w, rect_arrow_h);
        ofDrawBitmapString("LEFT", rect_arrow_left_string_x, rect_arrow_left_string_y);

        //arrow in
        rect_arrow_in_x= rect_arrow_left_x + rect_arrow_w + 15;
        rect_arrow_in_y= rect_arrow_up_y;
        rect_arrow_in_string_x= rect_arrow_in_x + 5;
        rect_arrow_in_string_y= rect_arrow_in_y + 20;
        ofRect(rect_arrow_in_x, rect_arrow_in_y, rect_arrow_w, rect_arrow_h);
        ofDrawBitmapString("IN", rect_arrow_in_string_x, rect_arrow_in_string_y);

        //arrow down
        rect_arrow_down_x= rect_arrow_up_x;
        rect_arrow_down_y= rect_arrow_up_y + rect_arrow_h + 10;
        rect_arrow_down_string_x= rect_arrow_down_x + 5;
        rect_arrow_down_string_y= rect_arrow_down_y + 20;
        ofRect(rect_arrow_down_x, rect_arrow_down_y, rect_arrow_w, rect_arrow_h);
        ofDrawBitmapString("DOWN", rect_arrow_down_string_x, rect_arrow_down_string_y);

        //arrow right
        rect_arrow_right_x= rect_arrow_left_x;
        rect_arrow_right_y= rect_arrow_down_y;
        rect_arrow_right_string_x= rect_arrow_right_x + 5;
        rect_arrow_right_string_y= rect_arrow_right_y + 20;
        ofRect(rect_arrow_right_x, rect_arrow_right_y, rect_arrow_w, rect_arrow_h);
        ofDrawBitmapString("RIGHT", rect_arrow_right_string_x, rect_arrow_right_string_y);

        //arrow out
        rect_arrow_out_x= rect_arrow_in_x;
        rect_arrow_out_y= rect_arrow_down_y;
        rect_arrow_out_string_x= rect_arrow_out_x + 5;
        rect_arrow_out_string_y= rect_arrow_out_y + 20;
        ofRect(rect_arrow_out_x, rect_arrow_out_y, rect_arrow_w, rect_arrow_h);
        ofDrawBitmapString("OUT", rect_arrow_out_string_x, rect_arrow_out_string_y);

        ofSetLineWidth(1);

        //big_rect_vertical_buttons_left_fill
       /* ofFill();
        ofSetColor(rect_fill_color);
        ofRect(big_rect_v_b_l_f_x, big_rect_v_b_l_f_y, big_rect_v_b_l_f_w, big_rect_v_b_l_f_h);
        ofNoFill();
        ofSetColor(rect_color);
        //big_rect_vertical_buttons_right_fill
        ofFill();
        ofSetColor(rect_fill_color);
        ofRect(big_rect_v_b_r_f_x, big_rect_v_b_r_f_y, big_rect_v_b_l_f_w, big_rect_v_b_l_f_h);
        ofNoFill();
        ofSetColor(rect_color);*/
    }

}
void testApp::drawPointCloud() {

	ofPushMatrix();

    //============================================================================
    /*OF_PRIMITIVE_TRIANGLES,
    OF_PRIMITIVE_TRIANGLE_STRIP,
    OF_PRIMITIVE_TRIANGLE_FAN,
    OF_PRIMITIVE_LINES,
    OF_PRIMITIVE_LINE_STRIP,
    OF_PRIMITIVE_LINE_LOOP,
    OF_PRIMITIVE_POINTS*/

    //riempio la struttura con i vertici di quello che il kinect vede
    int tmpIndex= 0;
    int near= 500;
    int far= 1500;
    for(int y = 0; y < h; y += step) {
        for(int x = 0; x < w; x += step) {
		    float tmp= kinect.getDistanceAt(x, y) ;
			if(tmp > 0) {

                //prendo solo la sagoma
                if((far - tmp > 0) && (tmp - near > 0)){
                    mesh.addColor(kinect.getColorAt(x, y));
                    mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
                    mesh.addIndex(tmpIndex);
                    tmpIndex++;

                }
			}
		}
	}
    //============================================================================

    //============================================================================
	//per disegnare il punto più vicino
	int min= 4000;//un valore alto fuori scala

	int n = mesh.getNumVertices();

    //sono da settare al momento, per ora vanno lasciate statiche
	int far_distance= 1000;
	float nearestDistance = 500;//50 cm

	for(int i = 0; i < n; i++) {

        ofVec3f vertex= mesh.getVertex(i);

        int min_distance= vertex.z - nearestDistance;

		if(min_distance > 0){//siamo dopo nearestDistance

            //per il minimo (blu)
            if(min_distance < min){
                min= min_distance;
                min_vertex.set(vertex.x, vertex.y, nearestDistance + min);
                /*min_vertex.x= vertex.x;
                min_vertex.y= vertex.y;
                min_vertex.z= nearestDistance + min;*/
                min_index= i;
            }
		}
		else{//siamo prima nearestDistance

			min_vertex.x= vertex.x;
			min_vertex.y= vertex.y;
			min_vertex.z= nearestDistance;
		}
	}
    //============================================================================


    //============================================================================
	mesh.setMode(OF_PRIMITIVE_POINTS);
    easyCam.begin();
	//i punti sono proiettati sottosopra e al contrario
	ofScale(-1, -1, -1);
	ofTranslate(0, 0, translateZ);

    //

	if(b_clear_colors){
        mesh.clearColors();
        ofSetColor(ofColor::black);
	}

    //mesh.drawWireframe();
	mesh.drawVertices();

	//disegno una sfera blu nel punto più vicino
    ofSetColor(0,0,255);
    ofFill();
    ofSphere(min_vertex, 5);

    //imposto i due cerchi
    ofNoFill();
    ofSetLineWidth(3);

    //disegno il first_min
    ofSetColor(ofColor::yellow);
    //first_index= min_index;
    //first_min= mesh.getVertex(first_index);
    first_min.set(first_min_x, first_min_y, first_min_z);
    ofCircle(first_min, 15);

    //disegno il second_min
    ofSetColor(ofColor::green);
    //second_index= min_index + 100;
    //second_min= mesh.getVertex(second_index);
    second_min.set(second_min_x, second_min_y, second_min_z);
    ofCircle(second_min, 8);

    //disegno una retta da first_min a second_min
    ofSetColor(ofColor::red);
    ofLine(first_min, second_min);
    ofSetLineWidth(1);

    //deve stare alla fine altrimenti si perdono le modifiche fatte alla struttura
    easyCam.end();
	ofPopMatrix();
    //============================================================================
}

//--------------------------------------------------------------
void testApp::exit() {
	//kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
}

//--------------------------------------------------------------
void testApp::keyPressed (int key) {
	switch (key) {
	    //cambia da first_min a second_min
	    case 'g':
            b_arrow_color_second_min= false;
            b_arrow_color_first_min= true;

            b_move_second_min= false;
            b_move_first_min= true;

            b_first_min_color_over= true;
            break;
        case 'v':
            b_arrow_color_first_min= false;
            b_arrow_color_second_min= true;

            b_move_first_min= false;
            b_move_second_min= true;

            b_second_min_color_over= true;
            break;
        //regolano lo spostamento di first_min e second_min
		case OF_KEY_UP:
            if(b_move_first_min)
                first_min_y--;
            else if(b_move_second_min)
                second_min_y--;
			break;
		case OF_KEY_DOWN:
            if(b_move_first_min)
                first_min_y++;
            else if(b_move_second_min)
                second_min_y++;
			break;
        case OF_KEY_LEFT:
            if(b_move_first_min)
                first_min_x++;
            else if(b_move_second_min)
                second_min_x++;
            break;
        case OF_KEY_RIGHT:
            if(b_move_first_min)
                first_min_x--;
            else if(b_move_second_min)
                second_min_x--;
            break;
        case 'i':
            if(b_move_first_min)
                first_min_z++;
            else if(b_move_second_min)
                second_min_z++;
            break;
        case 'o':
            if(b_move_first_min)
                first_min_z--;
            else if(b_move_second_min)
                second_min_z--;
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
        //clear colors
        case 'c':
            b_clear_colors= !b_clear_colors;
            break;
        //recognition
        case 'r':
            b_stop_update= !b_stop_update;
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

    //tilt up
    if(x >= rect_a_up_x && x <= rect_a_up_x+rect_w && y >= rect_a_up_y && y <= rect_a_up_y + rect_h){

        b_a_up_color_over= true;
        b_a_up_press_enable= true;
    }
    else if(!(x >= rect_a_up_x && x <= rect_a_up_x+rect_w && y >= rect_a_up_y && y <= rect_a_up_y + rect_h)){

        b_a_up_color_over= false;
        b_a_up_press_enable= false;
    }

    //tilt down
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

    //first_min
    if(x >= rect_first_min_x && x <= rect_first_min_x+rect_w && y >= rect_first_min_y && y <= rect_first_min_y + rect_h){

        b_first_min_color_over= true;
        b_first_min_press_enable= true;
    }
    else if(!(x >= rect_first_min_x && x <= rect_first_min_x+rect_w && y >= rect_first_min_y && y <= rect_first_min_y + rect_h)){

        b_first_min_color_over= false;
        b_first_min_press_enable= false;
    }

    //second_min
    if(x >= rect_second_min_x && x <= rect_second_min_x+rect_w && y >= rect_second_min_y && y <= rect_second_min_y + rect_h){

        b_second_min_color_over= true;
        b_second_min_press_enable= true;
    }
    else if(!(x >= rect_second_min_x && x <= rect_second_min_x+rect_w && y >= rect_second_min_y && y <= rect_second_min_y + rect_h)){

        b_second_min_color_over= false;
        b_second_min_press_enable= false;
    }

    //arrow up
    if(x >= rect_arrow_up_x && x <= rect_arrow_up_x+rect_w && y >= rect_arrow_up_y && y <= rect_arrow_up_y + rect_h){

        b_arrow_up_press_enable= true;
        b_arrow_down_press_enable= false;
        b_arrow_left_press_enable= false;
        b_arrow_right_press_enable= false;
        b_arrow_in_press_enable= false;
        b_arrow_out_press_enable= false;
    }
    else if(!(x >= rect_arrow_up_x && x <= rect_arrow_up_x+rect_w && y >= rect_arrow_up_y && y <= rect_arrow_up_y + rect_h)){

        b_arrow_up_press_enable= false;
    }

    //arrow down
    if(x >= rect_arrow_down_x && x <= rect_arrow_down_x+rect_w && y >= rect_arrow_down_y && y <= rect_arrow_down_y + rect_h){

        b_arrow_down_press_enable= true;
        b_arrow_up_press_enable= false;
        b_arrow_left_press_enable= false;
        b_arrow_right_press_enable= false;
        b_arrow_in_press_enable= false;
        b_arrow_out_press_enable= false;
    }
    else if(!(x >= rect_arrow_down_x && x <= rect_arrow_down_x+rect_w && y >= rect_arrow_down_y && y <= rect_arrow_down_y + rect_h)){

        b_arrow_down_press_enable= false;
    }

    //arrow left
    if(x >= rect_arrow_left_x && x <= rect_arrow_left_x+rect_w && y >= rect_arrow_left_y && y <= rect_arrow_left_y + rect_h){

        b_arrow_left_press_enable= true;
        b_arrow_up_press_enable= false;
        b_arrow_down_press_enable= false;
        b_arrow_right_press_enable= false;
        b_arrow_in_press_enable= false;
        b_arrow_out_press_enable= false;
    }
    else if(!(x >= rect_arrow_left_x && x <= rect_arrow_left_x+rect_w && y >= rect_arrow_left_y && y <= rect_arrow_left_y + rect_h)){

        b_arrow_left_press_enable= false;
    }

    //arrow right
    if(x >= rect_arrow_right_x && x <= rect_arrow_right_x+rect_w && y >= rect_arrow_right_y && y <= rect_arrow_right_y + rect_h){

        b_arrow_right_press_enable= true;
        b_arrow_up_press_enable= false;
        b_arrow_down_press_enable= false;
        b_arrow_left_press_enable= false;
        b_arrow_in_press_enable= false;
        b_arrow_out_press_enable= false;
    }
    else if(!(x >= rect_arrow_right_x && x <= rect_arrow_right_x+rect_w && y >= rect_arrow_right_y && y <= rect_arrow_right_y + rect_h)){

        b_arrow_right_press_enable= false;
    }

    //arrow in
    if(x >= rect_arrow_in_x && x <= rect_arrow_in_x+rect_w && y >= rect_arrow_in_y && y <= rect_arrow_in_y + rect_h){

        b_arrow_in_press_enable= true;
        b_arrow_up_press_enable= false;
        b_arrow_down_press_enable= false;
        b_arrow_left_press_enable= false;
        b_arrow_right_press_enable= false;
        b_arrow_out_press_enable= false;
    }
    else if(!(x >= rect_arrow_in_x && x <= rect_arrow_in_x+rect_w && y >= rect_arrow_in_y && y <= rect_arrow_in_y + rect_h)){

        b_arrow_in_press_enable= false;
    }

    //arrow out
    if(x >= rect_arrow_out_x && x <= rect_arrow_out_x+rect_w && y >= rect_arrow_out_y && y <= rect_arrow_out_y + rect_h){

        b_arrow_out_press_enable= true;
        b_arrow_up_press_enable= false;
        b_arrow_down_press_enable= false;
        b_arrow_left_press_enable= false;
        b_arrow_right_press_enable= false;
        b_arrow_in_press_enable= false;
}
    else if(!(x >= rect_arrow_out_x && x <= rect_arrow_out_x+rect_w && y >= rect_arrow_out_y && y <= rect_arrow_out_y + rect_h)){

        b_arrow_out_press_enable= false;
    }

    //easycam
    /*if(x >= big_rect_easycam_x && x <= big_rect_easycam_x + big_rect_easycam_w && y >= big_rect_easycam_y && y <= big_rect_easycam_y + big_rect_easycam_h){
        if(bDrawPointCloud){
            b_esayCam_enable= true;
        }
    }
    else if(!(x >= big_rect_easycam_x && x <= big_rect_easycam_x + big_rect_easycam_w && y >= big_rect_easycam_y && y <= big_rect_easycam_y + big_rect_easycam_h)){
        b_esayCam_enable= false;
    }*/

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
    }
    //step down
    else if(b_step_down_press_enable){
        step--;
        if(step < 1) step= 1;
    }
    //point size up
    else if(b_point_size_up_press_enable){
        point_size++;
        if(point_size > 15) point_size= 15;
    }
    //point size down
    else if(b_point_size_down_press_enable){
        point_size--;
        if(point_size < 1) point_size= 1;
    }
    //recognition
    else if(b_recognition_press_enable){
        b_stop_update= !b_stop_update;
    }
    //first min
    else if(b_first_min_press_enable){
        b_arrow_color_second_min= false;
        b_arrow_color_first_min= true;

        b_move_second_min= false;
        b_move_first_min= true;

        b_first_min_color_over= true;
    }
    //second min
    else if(b_second_min_press_enable){
        b_arrow_color_first_min= false;
        b_arrow_color_second_min= true;

        b_move_first_min= false;
        b_move_second_min= true;

        b_second_min_color_over= true;
    }
    //arrow
    else if(b_move_first_min){

        if(b_arrow_up_press_enable){
            first_min_y--;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_down_press_enable){
            first_min_y++;
            b_arrow_up_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_left_press_enable){
            first_min_x++;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_right_press_enable){
            first_min_x--;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_in_press_enable){
            //first_index++;
            first_min_z++;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_out_press_enable){
            //first_index--;
            first_min_z--;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
        }
    }
    else if(b_move_second_min){

        if(b_arrow_up_press_enable){
            second_min_y--;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_down_press_enable){
            second_min_y++;
            b_arrow_up_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_left_press_enable){
            second_min_x++;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_right_press_enable){
            second_min_x--;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_in_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_in_press_enable){
            //second_index++;
            second_min_z++;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_out_press_enable= false;
        }
        else if(b_arrow_out_press_enable){
            //second_index--;
            second_min_z--;
            b_arrow_up_press_enable= false;
            b_arrow_down_press_enable= false;
            b_arrow_left_press_enable= false;
            b_arrow_right_press_enable= false;
            b_arrow_in_press_enable= false;
        }
    }

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button)
{}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h)
{}

