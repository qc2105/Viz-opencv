
//
//  main.cpp
//  Viz
//
//  Created by zhoushiwei on 15/4/23.
//  Copyright (c) 2015年 zhoushiwei. All rights reserved.
//

#include <opencv2/viz/widget_accessor.hpp>

#include <opencv2/viz/vizcore.hpp>

#include <opencv2/viz/viz3d.hpp>
#include <iostream>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "simple.h"
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkOBJReader.h>

using namespace cv;
using namespace std;
using namespace viz;

/**
 * @function help
 * @brief Display instructions to use this tutorial program
 */
void help()
{
    cout
    << "--------------------------------------------------------------------------"   << endl
    << "This program shows how to use makeTransformToGlobal() to compute required pose,"
    << "how to use makeCameraPose and Viz3d::setViewerPose. You can observe the scene "
    << "from camera point of view (C) or global point of view (G)"                    << endl
    << "Usage:"                                                                       << endl
    << "./transformations [ G | C ]"                                                 << endl
    << endl;
}


/**
 * @function cvcloud_load
 * @brief load bunny.ply
 */
Mat normal(1, 11510, CV_32FC3);
std::vector<cv::Point3i> faces_vertex;  //每一维表示三个顶点的组成
std::vector<cv::Point3i> faces_texel;
std::vector<cv::Point3i> faces_normal;
std::vector<cv::Point3f> faces_normal_vertex;

std::vector<cv::Point3d> positions;//模型XYZ顶点坐标
std::vector<cv::Point2d> texels; //UV坐标
std::vector<cv::Point3d> normals;
#include "ExtractOBJData.h"
ExtractOBJData EOBJ;
//void ComputeNormals()
//{
//    
//}
cv::Point3f getNormal(cv::Point3f &v1,cv::Point3f &v2,cv::Point3f &v3)
{
    cv::Point3f normal;
    cv::Point3f a=v3-v1;
    cv::Point3f b=v2-v1;
    normal=b.cross(a);
    float len = (float)(sqrt((normal.x*normal.x) + (normal.y*normal.y) + (normal.z*normal.z)));
   
    // avoid division by 0
    if (len == 0.0f)
        len = 1.0f;
    normal.x/=len;
    normal.y/=len;
    normal.z/=len;
    
    return normal;
}



static void onMouse( int event, int x, int y, int, void* )
{
 
    
    switch( event )
    {
        case EVENT_LBUTTONDOWN:
    
            break;
        case EVENT_LBUTTONUP:
           
            break;
            
    }
}


int main(int argn, char **argv)
{
 // show_mesh();
   //
//    show_cloud_shaded_by_normals();
//    show_image_3d();
   // show_image_method();
   // show_image_3d();
  //  show_textured_mesh();
    std::vector<Vec3d> points;
    std::vector<Vec2d> tcoords;
    std::vector<int> polygons;
    std::vector<int> normalpolygons;
    std::vector<string>shapesname;
   std::vector< std::vector<cv::Point3d> >mulmodelpositions;
    std::vector< std::vector<cv::Point2d> >mulmodeltexels;
    ifstream ifs("/Users/zhoushiwei/Dropbox/OpenGLAvatar/obj/FILES");
    while (!ifs.eof()) {
        string s;
        ifs>>s;
        shapesname.push_back(s);
        printf("%s\n",s.c_str());
        string path="/Users/zhoushiwei/Dropbox/OpenGLAvatar/obj/"+s;
        int modelposition,modeltexels,modelnormals,modelfaces;
        EOBJ.ExtractOBJDataInit(path, modelposition,modeltexels,modelnormals,modelfaces);
        EOBJ.extractOBJdata4(path, positions, texels, normals, faces_vertex,faces_texel,faces_normal);
        mulmodelpositions.push_back(positions);
        mulmodeltexels.push_back(texels);
        positions.clear();
        texels.clear();
        normals.clear();
        faces_vertex.clear();
        faces_texel.clear();
        faces_normal.clear();
        
    }
   
        string filepathOBJ="/Users/zhoushiwei/Dropbox/OpenGLAvatar/obj/Neutral.obj";
        int modelposition,modeltexels,modelnormals,modelfaces;
        EOBJ.ExtractOBJDataInit(filepathOBJ, modelposition,modeltexels,modelnormals,modelfaces);
        EOBJ.extractOBJdata4(filepathOBJ, positions, texels, normals, faces_vertex,faces_texel,faces_normal);
    for (int i=0; i<faces_vertex.size(); i++) {
        int polys[] = {3, faces_vertex.at(i).x,faces_vertex.at(i).y,faces_vertex.at(i).z};
        polygons.insert(polygons.end(), polys, polys + sizeof(polys)/sizeof(polys[0]));
    //   printf("f %d/%d %d/%d %d/%d\n",faces_vertex.at(i).x,faces_texel.at(i).x,faces_vertex.at(i).y,faces_texel.at(i).y,faces_vertex.at(i).z,faces_texel.at(i).z);
    }
    
   
    for(int k=0;k<mulmodelpositions.size();k++)
    {
        stringstream sss;
        sss<<k;
        string s="/Users/zhoushiwei/Dropbox/Viz/"+sss.str()+".obj";
         ofstream ifss(s);
        points.clear();
    for(size_t i = 0; i <positions.size(); ++i)
    {
//        float x=positions.at(i).x-mulmodelpositions.at(0).at(i).x;
//        float y=positions.at(i).y-mulmodelpositions.at(0).at(i).y;
//        float z=positions.at(i).z-mulmodelpositions.at(0).at(i).z;
//        printf("x %f y %f z %f\n",x,y,z);
     points.push_back(Vec3d(mulmodelpositions.at(k).at(i).x,mulmodelpositions.at(k).at(i).y,mulmodelpositions.at(k).at(i  ).z));
  //   points.push_back(Vec3d(positions.at(i).x,positions.at(i).y,positions.at(i).z));
        ifss<<"v "<<mulmodelpositions.at(k).at(i).x<<" "<<mulmodelpositions.at(k).at(i).y<<" "<<mulmodelpositions.at(k).at(i).z<<endl;
    }

       
    
    Mat lena = imread("/Users/zhoushiwei/Desktop/9497C0A9-54E7-431E-A8BA-AF771C98A632.png");
    cv::viz::Mesh mesh;
    mesh.cloud = Mat(points, true).reshape(3, 1);
       // cout<<mesh.cloud<<endl;
  //  mesh.tcoords = Mat(tcoords, true).reshape(2, 1);
    mesh.polygons = Mat(polygons, true).reshape(1, 1);
    
//         cout<<mesh.polygons<<endl;
//     cout<< mesh.cloud.rows<<"    "<< mesh.cloud.cols<<"  ";
//    cout<<mesh.polygons.rows<<"    "<<mesh.polygons.cols;
 //   mesh.texture = lena;
   
    Viz3d viz("show_textured_mesh");
        viz.setBackgroundMeshLab();
        viz.setWindowSize(cv::Size(480,480));
    viz.showWidget("coosys", WCoordinateSystem());
    Point3d cam_pos(0,0,0.0);//相机在世界坐标的位置 人脸朝向对应yaw，pitch
           Point3d cam_focal_point(0.0f,0.0f,-0.0f);//机镜头对准的物体在世界坐标的位置
               Point3d cam_y_dir(0.0f,0.0f,0.0f);//相机向上的方向在世界坐标中的方向
             Affine3d cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
    viz.showWidget("mesh", WMesh(mesh));
    viz.setRenderingProperty("mesh", SHADING, SHADING_PHONG);
    Mat normal2;
    viz::computeNormals(mesh, normal2);
        
        std::vector<cv::Point3f>facenormals;
        facenormals.resize(faces_vertex.size());
        for (int i=0; i<faces_vertex.size(); i++) {
      
            
                    int i0=faces_vertex.at(i).x;
                    int i1=faces_vertex.at(i).y;
                    int i2=faces_vertex.at(i).z;
                    cv::Point3f vi0,vi1,vi2;
                    vi0=cv::Point3f(mulmodelpositions.at(k).at(i0).x,mulmodelpositions.at(k).at(i0).y,mulmodelpositions.at(k).at(i0).z);
                    vi1=cv::Point3f(mulmodelpositions.at(k).at(i1).x,mulmodelpositions.at(k).at(i1).y,mulmodelpositions.at(k).at(i1).z);
                    vi2=cv::Point3f(mulmodelpositions.at(k).at(i2).x,mulmodelpositions.at(k).at(i2).y,mulmodelpositions.at(k).at(i2).z);
               //     printf("%d/%d,%d/%d,%d/%d\n",i0,index,i1,index1,i2,index2);
                  cv::Point3f normal =getNormal(vi0, vi1, vi2);
                   facenormals.at(i)=normal;
        }
        int shared;
        cv::Point3f vSum;
        cv::Point3f vNormal;
        std::vector<cv::Point3f> outnormals;
        int numberFace=(int)faces_vertex.size();
        for (int i=0; i<positions.size(); i++) {
            shared=0;
            vSum=cv::Point3f(0,0,0);
            for (int j=0; j<numberFace; j++) {
                int i0=faces_vertex.at(j).x;
                int i1=faces_vertex.at(j).y;
                int i2=faces_vertex.at(j).z;
                if (i0==i||i1==i||i2==i) {
                    vSum.x+=facenormals.at(j).x;
                    vSum.y+=facenormals.at(j).y;
                    vSum.z+=facenormals.at(j).z;
                    shared++;
                }
            }
            vNormal.x=vSum.x/shared;
            vNormal.y=vSum.y/shared;
            vNormal.z=vSum.z/shared;
            
            float len = (float)(sqrt((vNormal.x*vNormal.x) + (vNormal.y*vNormal.y) + (vNormal.z*vNormal.z)));
            
            // avoid division by 0
            if (len == 0.0f)
                len = 1.0f;
            vNormal.x/=len;
            vNormal.y/=len;
            vNormal.z/=len;
            
            outnormals.push_back(vNormal);
            printf("normals x %f,y %f z %f\n",vNormal.x,vNormal.y,vNormal.z);
        }
        for(int i=0; i<outnormals.size();i++){
             ifss<<"vn "<<outnormals.at(i).x<<" "<<outnormals.at(i).y<<" "<<outnormals.at(i).z<<endl;
        }
        
//        for (int i=0; i<normal2.cols; i+=3) {
//            ifss<<"vn "<<normal2.at<double>(0,i)<<" "<<normal2.at<double>(0,i+1)<<" "<<normal2.at<double>(0,i+2)<<endl;
//        }
        for (int i=0; i<texels.size(); i++) {
            ifss<<"vt "<<mulmodeltexels.at(k).at(i).x<<" "<<mulmodeltexels.at(k).at(i).y<<endl;
        }
        for (int i=0; i<faces_vertex.size(); i++) {
          
           ifss<<"f "<<faces_vertex.at(i).x+1<<"/"<<faces_texel.at(i).x+1<<"/"<<faces_vertex.at(i).x+1<<" "<<faces_vertex.at(i).y+1<<"/"<<faces_texel.at(i).y+1<<"/"<<faces_vertex.at(i).y+1<<" "<<faces_vertex.at(i).z+1<<"/"<<faces_texel.at(i).z+1<<"/"<<faces_vertex.at(i).z+1<<endl;
            //  ifss<<"f "<<faces_vertex.at(i).x+1<<"/"<<faces_texel.at(i).x+1<<" "<<faces_vertex.at(i).y+1<<"/"<<faces_texel.at(i).y+1<<" "<<faces_vertex.at(i).z+1<<"/"<<faces_texel.at(i).z+1<<endl;
            //   printf("f %d/%d %d/%d %d/%d\n",faces_vertex.at(i).x,faces_texel.at(i).x,faces_vertex.at(i).y,faces_texel.at(i).y,faces_vertex.at(i).z,faces_texel.at(i).z);
        }
        ifss.close();
        
        stringstream ss;
        ss<<k;
        string name=shapesname.at(k)+": "+ss.str();
        viz.showWidget("text2d", WText(name, Point(20, 20), 20, Color::green()));
        viz.spin();
     
       
        String path="/Users/zhoushiwei/Dropbox/Viz/"+ss.str()+".jpg";
        viz.saveScreenshot(path);
        waitKey(10);
    }
    float angle;
//    while(!viz.wasStopped())
//            {
//                angle += CV_PI * 0.01f;
//        
//                // カメラの姿勢を生成
//                Point3d cam_pos(20*cos(angle),0,25);
//                Point3d cam_focal_point(0.0f,0.0f,0.0f);
//                Point3d cam_y_dir(0.0f,0.0f,0.0f);
//                Affine3d cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
//        
//                // 視点をセット
//               // perfumeWindow.setViewerPose(cam_pose);
//        
//              //   perfumeWindow.setWidgetPose("Creating Widgets", cam_pose);
//              //  perfumeWindow.showWidget ("colorsa", ,
//                    //                              cam_pos);
//                Affine3d cloud_posea = Affine3f().translate(Vec3f(0.0f,0.0f,0.0f)).rotate(Vec3d(0.5,0.5,0));
//             //   perfumeWindow.showWidget("Creating Widgets", viz::WCoordinateSystem(), cloud_posea);
//                viz.showWidget("Achan", objAchan,cloud_posea);
//                perfumeWindow.spinOnce(1, true);
                
   //         }

    
    waitKey();
//    viz::Viz3d perfumeWindow("Creating Widgets");
//    perfumeWindow.setWindowSize(cv::Size(640,480));
//    /// Add coordinate axes
//    perfumeWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem(2));
//   
//    // Perfumeのウィジェットを生成
//    string filename1 = "/Users/zhoushiwei/Dropbox/人脸定位/Avatar/shape_0.1.obj";
//    string filename2 = "/Users/zhoushiwei/Dropbox/人脸定位/Avatar/shape_0.1.obj";
//    string filename3 = "/Users/zhoushiwei/Dropbox/人脸定位/Avatar/shape_0.1.obj";
//    WObj objAchan(filename1), objNocchi(filename2), objKashiyuka(filename3);
//    
//    
////    perfumeWindow.showWidget("Nocchi", objNocchi);
////    perfumeWindow.showWidget("Kashiyuka", objKashiyuka);
//    // setMouseCallback("Creating Widgets", onMouse, 0 );
//    float angle = 0.0;
//    
//            angle += CV_PI * 0.1f;
//    
//
//            Point3d cam_pos(0,0,5);//相机在世界坐标的位置 人脸朝向对应yaw，pitch
//            Point3d cam_focal_point(0.0f,0.0f,-30.0f);//机镜头对准的物体在世界坐标的位置
//            Point3d cam_y_dir(0.0f,0.0f,0.0f);//相机向上的方向在世界坐标中的方向
//            Affine3d cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
//    
//            // 显示出来
//            perfumeWindow.setViewerPose(cam_pose);
//    
//   Affine3d cloud_posea = Affine3f().translate(Vec3f(0.0f,0.0f,0.0f)).rotate(Vec3d(0,60,0));
//    perfumeWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem(), cloud_posea);
//    
//    while(!perfumeWindow.wasStopped())
//    {
//        angle += CV_PI * 0.01f;
//        
//        // カメラの姿勢を生成
//        Point3d cam_pos(20*cos(angle),0,25);
//        Point3d cam_focal_point(0.0f,0.0f,0.0f);
//        Point3d cam_y_dir(0.0f,0.0f,0.0f);
//        Affine3d cam_pose = viz::makeCameraPose(cam_pos, cam_focal_point, cam_y_dir);
//        
//        // 視点をセット
//       // perfumeWindow.setViewerPose(cam_pose);
//       
//      //   perfumeWindow.setWidgetPose("Creating Widgets", cam_pose);
//      //  perfumeWindow.showWidget ("colorsa", ,
//            //                              cam_pos);
//        Affine3d cloud_posea = Affine3f().translate(Vec3f(0.0f,0.0f,0.0f)).rotate(Vec3d(0.5,0.5,0));
//     //   perfumeWindow.showWidget("Creating Widgets", viz::WCoordinateSystem(), cloud_posea);
//        perfumeWindow.showWidget("Achan", objAchan,cloud_posea);
//        perfumeWindow.spinOnce(1, true);
//        
//    }
//    Vec3d rot_vec = Vec3d::all(0);
////    double translation_phase = 0.0, translation = 0.0;
////        while(!perfumeWindow.wasStopped())
////        {
////            /* Rotation using rodrigues */
////            /// Rotate around (1,1,1)
////         //   rot_vec[0] += CV_PI * 0.01;//绕x轴
////         //  rot_vec[1] += CV_PI * 0.01;
////    //        rot_vec[2] += CV_PI * 0.01;
////            rot_vec[2]=0;
////            /// Shift on (1,1,1)
////         //   translation_phase += CV_PI * 0.01;
////         //   translation = sin(translation_phase);
////            TickMeter tm;
////            tm.start();
////            /// Construct pose
////            translation=80.0f;
////            Affine3d pose(rot_vec, Vec3d(0.0, 0.0, translation));
////    
////          //  perfumeWindow.setWidgetPose("Kashiyuka", pose);
////            perfumeWindow.setViewerPose(pose);
////            perfumeWindow.spinOnce(1, true);
////            tm.stop();
////            printf("time %.2f\n",tm.getTimeMilli());
////        }
//
//    // 表示のスタート
//    perfumeWindow.spin();
//    
//    waitKey();
    
    /// Create a window
//    viz::Viz3d myWindow("Coordinate Frame");
//    myWindow.setWindowSize(cv::Size(640,480));
//    /// Add coordinate axes
//    myWindow.showWidget("Coordinate Widget", viz::WCoordinateSystem());
//    
//    /// Add line to represent (1,1,1) axis
//    viz::WLine axis(Point3f(-1.0, -1.0, -1.0), Point3d(1.0, 1.0, 1.0));
//    axis.setRenderingProperty(viz::LINE_WIDTH, 4.0);
//    myWindow.showWidget("Line Widget", axis);
//    Mat img=imread("/Users/zhoushiwei/Desktop/11.png");
//  
//    /// Construct a cube widget
//    viz::WCube cube_widget(Point3d(0.5, 0.5, 0.0), Point3d(0.0, 0.0, -0.5), true, viz::Color::blue());
//    cube_widget.setRenderingProperty(viz::LINE_WIDTH, 4.0);
//    
//    /// Display widget (update if already displayed)
//    myWindow.showWidget("Cube Widget", cube_widget);
//    Vec3d rot_vec = Vec3d::all(0);
//    double translation_phase = 0.0, translation = 0.0;
//    cv::VideoCapture cap;
//    cap.open(0);
//    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
//    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//    if( !cap.isOpened() )
//    {
//        help();
//        cout << "***Could not initialize capturing...***\n";
//        cout << "Current parameter's value: \n";
//        
//    }
//    Mat frame;
//    while (1) {
//        cap>>frame;
//  myWindow.setBackgroundTexture(frame);
//    /// Rodrigues vector
//    
////    while(!myWindow.wasStopped())
////    {
//        /* Rotation using rodrigues */
//        /// Rotate around (1,1,1)
//      //  rot_vec[0] += CV_PI * 0.01;//绕x轴
//       rot_vec[1] += CV_PI * 0.01;
////        rot_vec[2] += CV_PI * 0.01;
//        
//        /// Shift on (1,1,1)
//     //   translation_phase += CV_PI * 0.01;
//     //   translation = sin(translation_phase);
//        TickMeter tm;
//        tm.start();
//        /// Construct pose
//        Affine3d pose(rot_vec, Vec3d(0.0, 1.0, translation));
//        
//        myWindow.setWidgetPose("Cube Widget", pose);
//        
//        myWindow.spinOnce(1, true);
//        tm.stop();
//        printf("time %.2f\n",tm.getTimeMilli());
//    }
//    
//    cv::Mat cloud = cv::viz::readCloud("/Users/zhoushiwei/Downloads/CSE168-master/dragon.ply");
//    
//    cv::viz::Viz3d viz("abc");
//    viz.setBackgroundMeshLab();
//
//    viz.showWidget("coo", cv::viz::WCoordinateSystem(0));
//    viz.showWidget("cloud", cv::viz::WPaintedCloud(cloud));
//     viz.spin();
//    waitKey();
//    cv:: VideoCapture capture (0);
//    Viz3d viz ("dynamic");
//    //... We add contents...
//    
//    //we expose a camera rule hardly sideways
//    viz.setViewerPose(Affine3d().translate(cv::Vec(1.0,0.0,0.0));
//    while (!viz.wasStopped ())
//    {
//        //... We update contents...
//        //if it is necessary, we change poses at added vidzhetov
//        //if it is necessary, we replace clouds new gained with Kinect
//        //if it is necessary, we change a camera rule
//        
//        capture.grab ();
//        capture.retrieve (color, CV_CAP_OPENNI_BGR_IMAGE);
//        capture.retrieve (depth, CV_CAP_OPENNI_DEPTH_MAP);
//        Mat cloud = computeCloud (depth);
//        Mat display = normalizeDepth (depth);
//        
//        viz.showWidget ("cloud", WCloud (cloud, color));
//        viz.showWidget ("image", WImageOverlay (display, Rect (0, 0, 240, 160)));
//        
//        //otrisovyvaem the user feeding into in flow 30 msec also is processed
//        viz.spinOnce (30/*ms*/, true/*force_redraw*/));
//    }
//    
//    cv:: Mat cloud = cv:: viz:: readCloud ("/Users/zhoushiwei/Desktop/opencv-maste3.0/samples/cpp/tutorial_code/viz/bunny.ply");
//    
//    //we create an array of colors for a cloud and it is filled by its casual data
//    cv:: Mat colors (cloud.size (), CV_8UC3);
//    theRNG ().fill (colors, RNG:: UNIFORM, 50, 255);
//    
//    //we copy a cloud of points and we expose a part of points in NAN - such points will be ignored
//    float qnan = std:: numeric_limits<float>::quiet_NaN ();
//    cv:: Mat masked_cloud = cloud.clone ();
//    for (int i = 0; i < cloud.total (); ++ i)
//        if ((i % 16)!= 0)
//            masked_cloud.at<Vec3f> ( i) = Vec3f (qnan, qnan, qnan);
//    
//    Viz3d viz ("/Users/zhoushiwei/Desktop/opencv-maste3.0/samples/cpp/tutorial_code/viz/bunny.ply");
//    viz.showWidget ("colors", WCoordinateSystem ());
//    
//    //the Red dragon
//    viz.showWidget ("red" , WCloud(cloud, Color::red ()),
//                    Affine3d ().translate (Vec3d (-2.0, 0.0, 0.0)));
//
//    //the Dragon with casual colors
//    viz.showWidget ("colorsa", WCloud (cloud, colors),
//                    Affine3d ().translate (Vec3d (+1.0, 0.0, 0.0)));
//    
//    //the Dragon with casual colors and the filtered off points with an individual pose
//    viz.showWidget ("masked", WCloud (masked_cloud, colors), Affine3d:: Identity ());
//    
//    //Aвтоматическая a coloring, it is useful if we do not have colors
//    viz.showWidget ("painted", WPaintedCloud (cloud),
//                    Affine3d ().translate (Vec3d (+2.0, 0.0, 0.0)));
//    viz.spin ();
//    waitKey();
//    if (argn < 2)
//    {
//        cout << "Usage: " << endl << "./transformations [ G | C ]" << endl;
//        return 1;
//    }
//    
//    bool camera_pov = (argv[1][0] == 'C');
   
     return 0;
}