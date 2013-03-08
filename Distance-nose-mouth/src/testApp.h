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

	int step= 3, point_size= 3, nearest_index= -1, translateZ= -600;

	float distance_cam= 600;

    ofVec3f min_vertex, first_min, second_min;

    int vertex_circle_x= 0, vertex_circle_y= 0, vertex_circle_z= 0;

    ofMesh mesh;

    int mesh_point= 0, mesh_restricted_point= 0;

	ofEasyCam cam;

	ofVec3f last_vertex;

	// used for viewing the point cloud
	ofEasyCam easyCam;

    //==========================================================================================================
	//INTERFACCIA: pulsanti e info
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

         b_clear_color_over, b_clear_colol_press_enable,

         b_step_up_color_over, b_step_up_press_enable,

         b_step_down_color_over, b_step_down_press_enable,

         b_point_size_up_color_over, b_point_size_up_press_enable,

         b_point_size_down_color_over, b_point_size_down_press_enable,

         b_recognition_color_over, b_recognition_press_enable;

    bool b_stop_update;

    bool b_clear_colors;

    bool b_draw_pcl_options;

    bool b_enable_distance_recognition;

    //big_rect orizontal
	float big_rect_o_x= 0,
          big_rect_o_y= 750,
          big_rect_o_w= 1280,
          big_rect_o_h= 50;
    //big_rect_vertical
	float big_rect_v_x= 5,
          big_rect_v_y= 5,
          big_rect_v_w= 250,
          big_rect_v_h= 300;
    //big_rect_vertical_buttons
    float big_rect_v_b_x= big_rect_v_x,
          big_rect_v_b_y= big_rect_v_y + big_rect_v_h + 10,
          big_rect_v_b_w= 250,
          big_rect_v_b_h= 400;
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
    //rect clear color
    float rect_clear_color_x, rect_clear_color_y, rect_clear_color_string_x, rect_clear_color_string_y;
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
    //==========================================================================================================

};
