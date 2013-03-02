#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp {
public:

	void setup();
	void update();
	void draw();
	void exit();

	void drawPointCloud();

	void keyPressed(int key);
	void mouseDragged(int x, int y, int button);
	void mousePressed(int x, int y, int button);
	void mouseReleased(int x, int y, int button);
	void windowResized(int w, int h);

	//
	void findClosestPoint(int* min_x, int* min_Y);
	void drawClosestPoint();
	void mouseMoved(int x, int y);
	void mouseMovedOnPCL(int x, int y, int z);
	void mousePressedOnPCL(int x, int y, int z);
	void drawRectangleMenu();
	void drawInfo();
	//

	ofxKinect kinect;

	ofxCvColorImage colorImg;

	ofxCvGrayscaleImage grayImage; // grayscale depth image
	ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
	ofxCvGrayscaleImage grayThreshFar; // the far thresholded image

	ofxCvContourFinder contourFinder;

	bool bThreshWithOpenCV;
	bool bDrawPointCloud;

	bool b_DrawClosestPoint;

	bool b_esayCam_enable;

	int nearThreshold;
	int farThreshold;

	int angle;

	int step= 3, point_size= 3;

    int vertex_circle_x, vertex_circle_y, vertex_circle_z;

	ofEasyCam cam;

	ofVec3f last_vertex;

	// used for viewing the point cloud
	ofEasyCam easyCam;

	//per i rect
	ofColor big_rect_color_o= ofColor::gray,
            big_rect_color_v= ofColor::white,
            rect_color= ofColor::white,
            rect_color_over= ofColor::blue,
            rect_label_color= ofColor::white,
            rect_pcl_color;

	bool b_pcl_color_over, b_pcl_press_enable,
         b_exit_color_over, b_exit_press_enable,
         b_a_up_color_over, b_a_up_press_enable,
         b_a_down_color_over, b_a_down_press_enable,
         b_step_up_color_over, b_step_up_press_enable,
         b_step_down_color_over, b_step_down_press_enable,
         b_recognition_color_over, b_recognition_press_enable;

    bool b_draw_pcl_options;

    bool b_enable_distance_recognition;

    //big_rect orizontal
	float big_rect_o_x= 0,
          big_rect_o_y= 750,
          big_rect_o_w= 1280,
          big_rect_o_h= 50;
    //big_rect_vertical
	float big_rect_v_x= 0,
          big_rect_v_y= 0,
          big_rect_v_w= 250,
          big_rect_v_h= 300;
    //rect generici
	float rect_w= 100, rect_h= 30;
    //rect pcl
    float rect_pcl_x, rect_pcl_y, rect_pcl_string_x, rect_pcl_string_y;
    //rect exit
    float rect_exit_x, rect_exit_y, rect_exit_string_x, rect_exit_string_y;
    //rect angle up
    float rect_a_up_x, rect_a_up_y, rect_a_up_string_x, rect_a_up_string_y;
    //rect angle down
    float rect_a_down_x, rect_a_down_y, rect_a_down_string_x, rect_a_down_string_y;
    //rect step up
    float rect_step_up_x, rect_step_up_y, rect_step_up_string_x, rect_step_up_string_y;
    //rect step down
    float rect_step_down_x, rect_step_down_y, rect_step_down_string_x, rect_step_down_string_y;
    //rect point size up
    float rect_psize_up_x, rect_psize_up_y, rect_psize_up_string_x, rect_psize_up_string_y;
    //rect point size down
    float rect_psize_down_x, rect_psize_down_y, rect_psize_down_string_x, rect_psize_down_string_y;
    //distance recognition
    float rect_recognition_x, rect_recognition_y, rect_recognition_string_x, rect_recognition_string_y;
};
