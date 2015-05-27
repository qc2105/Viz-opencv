//
//  ExtractOBJData.h
//  Avatar
//
//  Created by zhoushiwei on 14-5-4.
//  Copyright (c) 2014年 zhoushiwei. All rights reserved.
//

#ifndef Avatar_ExtractOBJData_h
#define Avatar_ExtractOBJData_h

#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/opencv.hpp>
using namespace std;

class ExtractOBJData{
    
    
    
    // Model Structure
    typedef struct Model
    {
        int vertices;
        int positions;
        int texels;
        int normals;
        int faces;
    }
    Model;
    
    Model getOBJinfo(string fp)
    {
        Model model = {0};
        
        // Open OBJ file
        ifstream inOBJ;
        inOBJ.open(fp);
        if(!inOBJ.good())
        {
            cout << "ERROR OPENING OBJ FILE" << endl;
            exit(1);
        }
        
        // Read OBJ file
        while(!inOBJ.eof())
        {
            string line;
            getline(inOBJ, line);
          //  printf("%s\n",line.c_str());
            string type = line.substr(0,2);
            
            if(type.compare("v ") == 0)
                model.positions++;
            else if(type.compare("vt") == 0)
                model.texels++;
            else if(type.compare("vn") == 0)
                model.normals++;
            else if(type.compare("f ") == 0)
                model.faces++;
        }
        
        model.vertices = model.faces*3;
        
        // Close OBJ file
        inOBJ.close();
        
        return model;
    }
private:
    Model model;
    
    int OBJVertices;
    float OBJPositions;
    float OBJTexels;
    float OBJNormals;
    
    
public:
    
    //初始化
  void ExtractOBJDataInit( string &objpath,int &model_position,int &model_texels,int& model_normals,int &model_faces)
    {
        model=getOBJinfo(objpath);
        model_position=model.positions;
        model_texels=model.texels;
        model_normals=model.normals;
        model_faces=model.faces;
    }
    //解析文件
    void extractOBJdata(string fp,  std::vector<cv::Point3d> &positionscv,
                        std::vector<cv::Point2d> &texelscv,
                        std::vector<cv::Point3d> &normalscv,
                        std::vector<cv::Point3i> &faces_vertex,
                        std::vector<cv::Point3i> &faces_texel,
                        std::vector<cv::Point3i> &faces_normal)
        {
            
          
            float positions[model.positions][3];    // XYZ
            float texels[model.texels][2];          // UV
            float normals[model.normals][3];        // XYZ
            int faces[model.faces][9];              // PTN PTN PTN
            
            // Counters
            int p = 0;
            int t = 0;
            int n = 0;
            int f = 0;
            
            // Open OBJ file
            ifstream inOBJ;
            inOBJ.open(fp);
            if(!inOBJ.good())
            {
                cout << "ERROR OPENING OBJ FILE" << endl;
                exit(1);
            }
            
            // Read OBJ file
            while(!inOBJ.eof())
            {
                string line;
                getline(inOBJ, line);
                string type = line.substr(0,2);
                
                // Positions
                if(type.compare("v ") == 0)
                {
                    // Copy line for parsing
                    char* l = new char[line.size()+1];
                    memcpy(l, line.c_str(), line.size()+1);
                    
                    // Extract tokens
                    strtok(l, " ");
                    for(int i=0; i<3; i++)
                        positions[p][i] = atof(strtok(NULL, " "));
                    positionscv.push_back(cv::Point3d(positions[p][0],positions[p][1],positions[p][2]));
                 //   printf("%f %f %f\n",positions[p][0],positions[p][1],positions[p][2]);
                    // Wrap up
                    delete[] l;
                    p++;
                }
                
                // Texels
                else if(type.compare("vt") == 0)
                {
                    char* l = new char[line.size()+1];
                    memcpy(l, line.c_str(), line.size()+1);
                    
                    strtok(l, " ");
                    for(int i=0; i<2; i++)
                        texels[t][i] = atof(strtok(NULL, " "));
                    texelscv.push_back(cv::Point2d(texels[t][0],texels[t][1]));
                    delete[] l;
                    t++;
                }
                
                // Normals
                else if(type.compare("vn") == 0)
                {
                    char* l = new char[line.size()+1];
                    memcpy(l, line.c_str(), line.size()+1);
                    
                    strtok(l, " ");
                    for(int i=0; i<3; i++)
                        normals[n][i] = atof(strtok(NULL, " "));
                    normalscv.push_back(cv::Point3d(normals[n][0],normals[n][1],normals[n][2]));
                    delete[] l;
                    n++;
                }
                
                // Faces
                else if(type.compare("f ") == 0)
                {
                    char* l = new char[line.size()+1];
                    memcpy(l, line.c_str(), line.size()+1);
                    
                    strtok(l, " ");
                    for(int i=0; i<9; i++)
                        faces[f][i] = atof(strtok(NULL, " /"));
                    faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][3]-1,faces[f][6]-1));
                    faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][4]-1,faces[f][7]-1));
                    faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                    
//                    for(int i=0; i<6; i++)
//                        faces[f][i] = atof(strtok(NULL, " /"));
//                    faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][2]-1,faces[f][4]-1));
//                    faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][3]-1,faces[f][5]-1));
                    //  faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                    
                    delete[] l;
                    f++;
                }
            }
          //  printf("%d",p);
            // Close OBJ file
            inOBJ.close();
            
        }

    void extractOBJdata2(string fp,  std::vector<cv::Point3d> &positionscv,
                        std::vector<cv::Point2d> &texelscv,
                        std::vector<cv::Point3d> &normalscv,
                        std::vector<cv::Point3i> &faces_vertex,
                        std::vector<cv::Point3i> &faces_texel,
                        std::vector<cv::Point3i> &faces_normal)
    {
        
        
        float positions[model.positions][3];    // XYZ
        float texels[model.texels][2];          // UV
        float normals[model.normals][3];        // XYZ
        int faces[model.faces][9];              // PTN PTN PTN
        
        // Counters
        int p = 0;
        int t = 0;
        int n = 0;
        int f = 0;
        
        // Open OBJ file
        ifstream inOBJ;
        inOBJ.open(fp);
        if(!inOBJ.good())
        {
            cout << "ERROR OPENING OBJ FILE" << endl;
            exit(1);
        }
        
        // Read OBJ file
        while(!inOBJ.eof())
        {
            string line;
            getline(inOBJ, line);
            string type = line.substr(0,2);
            
            // Positions
            if(type.compare("v ") == 0)
            {
                // Copy line for parsing
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                // Extract tokens
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    positions[p][i] = atof(strtok(NULL, " "));
                positionscv.push_back(cv::Point3d(positions[p][0],positions[p][1],positions[p][2]));
                //   printf("%f %f %f\n",positions[p][0],positions[p][1],positions[p][2]);
                // Wrap up
                delete[] l;
                p++;
            }
            
            // Texels
            else if(type.compare("vt") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<2; i++)
                    texels[t][i] = atof(strtok(NULL, " "));
                texelscv.push_back(cv::Point2d(texels[t][0],texels[t][1]));
                delete[] l;
                t++;
            }
            
            // Normals
            else if(type.compare("vn") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    normals[n][i] = atof(strtok(NULL, " "));
                normalscv.push_back(cv::Point3d(normals[n][0],normals[n][1],normals[n][2]));
                delete[] l;
                n++;
            }
            
            // Faces
            else if(type.compare("f ") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
//                for(int i=0; i<9; i++)
//                    faces[f][i] = atof(strtok(NULL, " //"));
//                faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][3]-1,faces[f][6]-1));
//                faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][4]-1,faces[f][7]-1));
//                faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
                                    for(int i=0; i<6; i++)
                                       faces[f][i] = atof(strtok(NULL, " //"));
                                 faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][2]-1,faces[f][4]-1));
                                  faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][3]-1,faces[f][5]-1));
                                  //  faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
                delete[] l;
                f++;
            }
        }
        //  printf("%d",p);
        // Close OBJ file
        inOBJ.close();
        
    }


    //解析文件
    void extractOBJdata3(string fp,  std::vector<cv::Point3d> &positionscv,
                        std::vector<cv::Point2d> &texelscv,
                        std::vector<cv::Point3d> &normalscv,
                        std::vector<cv::Point3i> &faces_vertex,
                        std::vector<cv::Point3i> &faces_texel,
                        std::vector<cv::Point3i> &faces_normal)
    {
        
        
        float positions[model.positions][3];    // XYZ
        float texels[model.texels][2];          // UV
        float normals[model.normals][3];        // XYZ
        int faces[model.faces][9];              // PTN PTN PTN
        
        // Counters
        int p = 0;
        int t = 0;
        int n = 0;
        int f = 0;
        
        // Open OBJ file
        ifstream inOBJ;
        inOBJ.open(fp);
        if(!inOBJ.good())
        {
            cout << "ERROR OPENING OBJ FILE" << endl;
            exit(1);
        }
        
        // Read OBJ file
        while(!inOBJ.eof())
        {
            string line;
            getline(inOBJ, line);
            string type = line.substr(0,2);
            
            // Positions
            if(type.compare("v ") == 0)
            {
                // Copy line for parsing
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                // Extract tokens
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    positions[p][i] = atof(strtok(NULL, " "));
                positionscv.push_back(cv::Point3d(positions[p][0],positions[p][1],positions[p][2]));
                // printf("%f %f %f\n",positions[p][0],positions[p][1],positions[p][2]);
                // Wrap up
                delete[] l;
                p++;
            }
            
            // Texels
            else if(type.compare("vt") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<2; i++)
                    texels[t][i] = atof(strtok(NULL, " "));
                texelscv.push_back(cv::Point2d(texels[t][0],texels[t][1]));
                delete[] l;
                t++;
            }
            
            // Normals
            else if(type.compare("vn") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    normals[n][i] = atof(strtok(NULL, " "));
                normalscv.push_back(cv::Point3d(normals[n][0],normals[n][1],normals[n][2]));
                delete[] l;
                n++;
            }
            
            // Faces
            else if(type.compare("f ") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<9; i++)
                    faces[f][i] = atof(strtok(NULL, " /"));
                faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][3]-1,faces[f][6]-1));
                faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][4]-1,faces[f][7]-1));
                faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
                //                    for(int i=0; i<6; i++)
                //                        faces[f][i] = atof(strtok(NULL, " //"));
                //                    faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][2]-1,faces[f][4]-1));
                //                    faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][3]-1,faces[f][5]-1));
                //                    //  faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
                delete[] l;
                f++;
            }
        }
        //  printf("%d",p);
        // Close OBJ file
        inOBJ.close();
        
    }
    void extractOBJdata4(string fp,  std::vector<cv::Point3d> &positionscv,
                         std::vector<cv::Point2d> &texelscv,
                         std::vector<cv::Point3d> &normalscv,
                         std::vector<cv::Point3i> &faces_vertex,
                         std::vector<cv::Point3i> &faces_texel,
                         std::vector<cv::Point3i> &faces_normal)
    {
        
        
        float positions[model.positions][3];    // XYZ
        float texels[model.texels][2];          // UV
        float normals[model.normals][3];        // XYZ
        int faces[model.faces][9];              // PTN PTN PTN
        
        // Counters
        int p = 0;
        int t = 0;
        int n = 0;
        int f = 0;
        
        // Open OBJ file
        ifstream inOBJ;
        inOBJ.open(fp);
        if(!inOBJ.good())
        {
            cout << "ERROR OPENING OBJ FILE" << endl;
            exit(1);
        }
        
        // Read OBJ file
        while(!inOBJ.eof())
        {
            string line;
            getline(inOBJ, line);
            string type = line.substr(0,2);
            
            // Positions
            if(type.compare("v ") == 0)
            {
                // Copy line for parsing
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                // Extract tokens
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    positions[p][i] = atof(strtok(NULL, " "));
                positionscv.push_back(cv::Point3d(positions[p][0],positions[p][1],positions[p][2]));
                // printf("%f %f %f\n",positions[p][0],positions[p][1],positions[p][2]);
                // Wrap up
                delete[] l;
                p++;
            }
            
            // Texels
            else if(type.compare("vt") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<2; i++)
                    texels[t][i] = atof(strtok(NULL, " "));
                texelscv.push_back(cv::Point2d(texels[t][0],texels[t][1]));
                delete[] l;
                t++;
            }
            
            // Normals
            else if(type.compare("vn") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    normals[n][i] = atof(strtok(NULL, " "));
                normalscv.push_back(cv::Point3d(normals[n][0],normals[n][1],normals[n][2]));
                delete[] l;
                n++;
            }
            
            // Faces
            else if(type.compare("f ") == 0)
            {
//                char* l = new char[line.size()+1];
//                memcpy(l, line.c_str(), line.size()+1);
           //     printf("%s\n",line.c_str());
                replace(line.begin(), line.end(), '/', ' ');
             //   printf("%s\n",line.c_str());
                istringstream ls(line);
                int vi1, vti1, vi2, vti2, vi3, vti3;
                string ss;
                ls>>ss;
                ls >> vi1 >> vti1 >> vi2 >> vti2 >> vi3 >> vti3;
            //   printf("vti1 %d  vti2 %d vit3 %d\n",vi1 ,  vi2 ,vi3);
                faces_texel.push_back(cv::Point3i(vti1-1, vti2-1,vti3-1));
                faces_vertex.push_back(cv::Point3i( vi1-1,vi2-1,vi3-1));
                
                if(ls.peek()==' '){
                    int vi4, vti4;
					ls >> vi4 >> vti4;
                    faces_vertex.push_back(cv::Point3i(vi1-1, vi3-1,vi4-1));
                    faces_texel.push_back(cv::Point3i( vti1-1,vti3-1,vti4-1));
                }
                
                
//                strtok(l, " ");
//                for(int i=0; i<9; i++)
//                    faces[f][i] = atof(strtok(NULL, " /"));
//                faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][3]-1,faces[f][6]-1));
//                faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][4]-1,faces[f][7]-1));
//                faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
                //                    for(int i=0; i<6; i++)
                //                        faces[f][i] = atof(strtok(NULL, " //"));
                //                    faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][2]-1,faces[f][4]-1));
                //                    faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][3]-1,faces[f][5]-1));
                //                    //  faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
             //   delete[] l;
                f++;
            }
        }
        //  printf("%d",p);
        // Close OBJ file
        inOBJ.close();
        
    }
 
//    cv::Point3f getNormal(const cv::Point3f& v1, const cv::Point3f& v2, const cv::Point3f& v3) {
//        cv::Point3f a = v1 - v2;
//        cv::Point3f b = v3 - v2;
//        cv::Point3f normal = b.cross(a);
//       
//        float length = (float)sqrt(normal.x*normal.x + normal.y*normal.y + normal.z*normal.z);
//        if( length > 0 ) {
//            normal.x /= length;
//            normal.y /= length;
//            normal.z /= length;
//     
//        return normal;
//    }
//        
//    void buildFaceNormals( std::vector<cv::Point3d> &positionscv,std::vector<cv::Point3i> &faces_vertex,
//                          std::vector<cv::Point3i> &faces_normal,std::vector<cv::Point3d> &normalscv,)
//    {
//        for (int i=0; i<faces_vertex.size(); i++) {
//           faces_vertex[i]
//            cv::Point3f normal = getNormal(positionscv[i]., mesh.getVertices()[i1], mesh.getVertices()[i2]);
//        }
//    }
    void extractOBJdata5(string fp,  std::vector<cv::Point3d> &positionscv,
                         std::vector<cv::Point2d> &texelscv,
                         std::vector<cv::Point3d> &normalscv,
                         std::vector<cv::Point3i> &faces_vertex,
                         std::vector<cv::Point3i> &faces_texel,
                         std::vector<cv::Point3i> &faces_normal)
    {
        
        
        float positions[model.positions][3];    // XYZ
        float texels[model.texels][2];          // UV
        float normals[model.normals][3];        // XYZ
        int faces[model.faces][9];              // PTN PTN PTN
        
        // Counters
        int p = 0;
        int t = 0;
        int n = 0;
        int f = 0;
        
        // Open OBJ file
        ifstream inOBJ;
        inOBJ.open(fp);
        if(!inOBJ.good())
        {
            cout << "ERROR OPENING OBJ FILE" << endl;
            exit(1);
        }
        
        // Read OBJ file
        while(!inOBJ.eof())
        {
            string line;
            getline(inOBJ, line);
            string type = line.substr(0,2);
            
            // Positions
            if(type.compare("v ") == 0)
            {
                // Copy line for parsing
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                // Extract tokens
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    positions[p][i] = atof(strtok(NULL, " "));
                positionscv.push_back(cv::Point3d(positions[p][0],positions[p][1],positions[p][2]));
                // printf("%f %f %f\n",positions[p][0],positions[p][1],positions[p][2]);
                // Wrap up
                delete[] l;
                p++;
            }
            
            // Texels
            else if(type.compare("vt") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<2; i++)
                    texels[t][i] = atof(strtok(NULL, " "));
                texelscv.push_back(cv::Point2d(texels[t][0],texels[t][1]));
                delete[] l;
                t++;
            }
            
            // Normals
            else if(type.compare("vn") == 0)
            {
                char* l = new char[line.size()+1];
                memcpy(l, line.c_str(), line.size()+1);
                
                strtok(l, " ");
                for(int i=0; i<3; i++)
                    normals[n][i] = atof(strtok(NULL, " "));
                normalscv.push_back(cv::Point3d(normals[n][0],normals[n][1],normals[n][2]));
                delete[] l;
                n++;
            }
            
            // Faces
            else if(type.compare("f ") == 0)
            {
                //                char* l = new char[line.size()+1];
                //                memcpy(l, line.c_str(), line.size()+1);
                //     printf("%s\n",line.c_str());
                replace(line.begin(), line.end(), '/', ' ');
                //   printf("%s\n",line.c_str());
                istringstream ls(line);
                int vi1, vti1 ,vnor1, vi2, vti2,vnor2, vi3, vti3,vnor3;
                string ss;
                ls>>ss;
                ls >> vi1 >> vti1 >>vnor1>> vi2 >> vti2 >>vnor2>> vi3 >> vti3>>vnor3;
                //   printf("vti1 %d  vti2 %d vit3 %d\n",vi1 ,  vi2 ,vi3);
                faces_texel.push_back(cv::Point3i(vti1-1, vti2-1,vti3-1));
                faces_vertex.push_back(cv::Point3i( vi1-1,vi2-1,vi3-1));
                
                if(ls.peek()==' '){
                    int vi4, vti4;
                    ls >> vi4 >> vti4;
                    faces_vertex.push_back(cv::Point3i(vi1-1, vi3-1,vi4-1));
                    faces_texel.push_back(cv::Point3i( vti1-1,vti3-1,vti4-1));
                }
                
                
                //                strtok(l, " ");
                //                for(int i=0; i<9; i++)
                //                    faces[f][i] = atof(strtok(NULL, " /"));
                //                faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][3]-1,faces[f][6]-1));
                //                faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][4]-1,faces[f][7]-1));
                //                faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
                //                    for(int i=0; i<6; i++)
                //                        faces[f][i] = atof(strtok(NULL, " //"));
                //                    faces_vertex.push_back(cv::Point3i( faces[f][0]-1, faces[f][2]-1,faces[f][4]-1));
                //                    faces_texel.push_back(cv::Point3i( faces[f][1]-1, faces[f][3]-1,faces[f][5]-1));
                //                    //  faces_normal.push_back(cv::Point3i( faces[f][2]-1, faces[f][5]-1,faces[f][8]-1));
                
                //   delete[] l;
                f++;
            }
        }
        //  printf("%d",p);
        // Close OBJ file
        inOBJ.close();
        
    }
    
    
    void writeCvertices(string fp, string name, Model model)
    {
        // Create C file
        ofstream outC;
        outC.open(fp);
        if(!outC.good())
        {
            cout << "ERROR CREATING C FILE" << endl;
            exit(1);
        }
        
        // Write to C file
        outC << "// This is a .c file for the model: " << name << endl;
        outC << endl;
        
        // Header
        outC << "#include " << "\"" << name << ".h" << "\"" << endl;
        outC << endl;
        
        // Vertices
        outC << "const int " << name << "Vertices = " << model.vertices << ";" << endl;
        outC << endl;
        
        // Close C file
        outC.close();
    }
    
    void writeCpositions(string fp, string name, Model model, int faces[][9], float positions[][3])
    {
        // Append C file
        ofstream outC;
        outC.open(fp, ios::app);
        printf("model face %d",model.faces);
        // Positions
        outC << "const float " << name << "Positions[" << model.vertices*3 << "] = " << endl;
        outC << "{" << endl;
        for(int i=0; i<model.faces; i++)
        {
            int vA = faces[i][0] - 1;
            int vB = faces[i][3] - 1;
            int vC = faces[i][6] - 1;
            printf("%d %d %d\n",vA,vB,vC);
            outC << positions[vA][0] << ", " << positions[vA][1] << ", " << positions[vA][2] << ", " << endl;
            outC << positions[vB][0] << ", " << positions[vB][1] << ", " << positions[vB][2] << ", " << endl;
            outC << positions[vC][0] << ", " << positions[vC][1] << ", " << positions[vC][2] << ", " << endl;
        }
        outC << "};" << endl;
        outC << endl;
        
        // Close C file
        outC.close();
    }
    
    void writeCtexels(string fp, string name, Model model, int faces[][9], float texels[][2])
    {
        // Append C file
        ofstream outC;
        outC.open(fp, ios::app);
        
        // Texels
        outC << "const float " << name << "Texels[" << model.vertices*2 << "] = " << endl;
        outC << "{" << endl;
        for(int i=0; i<model.faces; i++)
        {
            int vtA = faces[i][1] - 1;
            int vtB = faces[i][4] - 1;
            int vtC = faces[i][7] - 1;
            
            outC << texels[vtA][0] << ", " << texels[vtA][1] << ", " << endl;
            outC << texels[vtB][0] << ", " << texels[vtB][1] << ", " << endl;
            outC << texels[vtC][0] << ", " << texels[vtC][1] << ", " << endl;
            printf("%f %f,%f %f,%f %f\n", texels[vtA][0],texels[vtA][1], texels[vtB][0] ,texels[vtB][1]
                   ,texels[vtC][0] ,texels[vtC][1]);
            
        }
        outC << "};" << endl;
        outC << endl;
        
        // Close C file
        outC.close();
    }
    
    void writeCnormals(string fp, string name, Model model, int faces[][9], float normals[][3])
    {
        // Append C file
        ofstream outC;
        outC.open(fp, ios::app);
        
        // Normals
        outC << "const float " << name << "Normals[" << model.vertices*3 << "] = " << endl;
        outC << "{" << endl;
        for(int i=0; i<model.faces; i++)
        {
            int vnA = faces[i][2] - 1;
            int vnB = faces[i][5] - 1;
            int vnC = faces[i][8] - 1;
            
            outC << normals[vnA][0] << ", " << normals[vnA][1] << ", " << normals[vnA][2] << ", " << endl;
            outC << normals[vnB][0] << ", " << normals[vnB][1] << ", " << normals[vnB][2] << ", " << endl;
            outC << normals[vnC][0] << ", " << normals[vnC][1] << ", " << normals[vnC][2] << ", " << endl;
        }
        outC << "};" << endl;
        outC << endl;
        
        // Close C file
        outC.close();
    }
    
 //   void outputData();
};

#endif
