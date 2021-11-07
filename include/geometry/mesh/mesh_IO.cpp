/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% by: Alireza Ahmadi                                     %
% University of Bonn- MSc Robotics & Geodetic Engineering%
% Alireza.Ahmadi@uni-bonn.de                             %
% AlirezaAhmadi.xyz                                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/
#include "mesh_IO.h"

namespace DynaMap{
namespace geometry{
    Mesh_IO::Mesh_IO(unsigned int& vertexNum, unsigned int& triangleNum): verticesNum(vertexNum), trianglesNum(triangleNum){
        maxTriangleNum = 1000000;   // todo ...
        maxVertexNum = 1000000;   // todo ...
        triangles = new Polygon[maxTriangleNum];
        vertices = new Vertex[maxVertexNum];
        BBox._min = BBox._max = Eigen::Vector3d(0, 0, 0);
    }
    Mesh_IO::Mesh_IO(const char* filename) : verticesNum(0), trianglesNum(0) {
        maxTriangleNum = 1000000;   // todo ...
        maxVertexNum = 1000000;   // todo ...
        triangles = new Polygon[maxTriangleNum];
        // triangles = new Triangle[maxTriangleNum];
        vertices = new Vertex[maxVertexNum];
        BBox._min = BBox._max = Eigen::Vector3d(0, 0, 0);
        if (strstr(filename, ".obj") == NULL && strstr(filename, ".ply") == NULL){
            fprintf(stderr, "Not OBJ or PLY file!!!\n");
            exit(-1);
        }
        if (strstr(filename, ".ply") != NULL){
            // this->readPLY(filename);
            return;
        }else if (strstr(filename, ".obj") != NULL){
            this->readOBJ(filename);
            return;
        }
    }
    Mesh_IO::~Mesh_IO(){
        Free();
    }
    void Mesh_IO::Free(void){
        // delete[] triangles;
        delete[] vertices;
        delete[] triangles;
    }
    static int splitLine(const std::string& refLine, std::vector<std::string> words){
        std::string line(refLine);
        static std::string whitespace = " \n\t\r";
        if(line.size() == 0)
            return 0;
        for (auto it = line.begin(); it != line.end(); ++it)
            if (*it == '/') *it = ' ';

        std::string::size_type pos = 0;
        while(pos != std::string::npos) {
            pos = line.find_first_not_of(whitespace, pos);
            if(pos == std::string::npos)
                break;
            std::string::size_type eow = line.find_first_of(whitespace, pos);
            words.push_back(std::string(line, pos, eow - pos));
            pos = eow;
        }
        return words.size();
    }
    void Mesh_IO::Mesh2MeshSTDCompressed(void){
        std::cout << this->verticesNum << ",  " << this->trianglesNum << std::endl;
        std::vector<Vertex> FilterVetices;
        std::vector<Polygon> FilterTriangles;
        unsigned int idv1, idv2, idv3;
        Polygon tmpPoly;
        int v_size = 0;
        bool idv1isNew = true; 
        bool idv2isNew = true;
        bool idv3isNew = true;

        for (size_t t = 0; t < trianglesNum; t++){ 
            for (size_t i = 0; i < FilterVetices.size() ; i++){
                // get vertex i-th from the filtered vector
                float3 v_pose = FilterVetices[i].position;
                // compare with vertices on the currecnt triangle
                if((vertices[triangles[t].vertexIndex.x - 1].position == v_pose) && idv1isNew){
                    idv1isNew = false;
                    idv1 = i + 1;
                }
                if((vertices[triangles[t].vertexIndex.y - 1].position == v_pose) && idv2isNew){
                    idv2isNew = false;
                    idv2 = i + 1;
                }
                if((vertices[triangles[t].vertexIndex.z - 1].position == v_pose) && idv3isNew){
                    idv3isNew = false;
                    idv3 = i + 1;
                }
            }

            // adding vertices into vector
            if(idv1isNew){
                FilterVetices.push_back(vertices[t * 3]);
                idv1 = FilterVetices.size();
            }
            if(idv2isNew){
                FilterVetices.push_back(vertices[t * 3 + 1]);
                idv2 = FilterVetices.size();
            }
            if(idv3isNew){
                FilterVetices.push_back(vertices[t * 3 + 2]);
                idv3 = FilterVetices.size();
            }

            // adding triangles vertecies (IDs)
            tmpPoly.vertexIndex.x = idv1;
            tmpPoly.vertexIndex.y = idv2;
            tmpPoly.vertexIndex.z = idv3;

            tmpPoly.normalIndex.x = idv1;
            tmpPoly.normalIndex.y = idv2;
            tmpPoly.normalIndex.z = idv3;
            FilterTriangles.push_back(tmpPoly); 

            idv1isNew = true; 
            idv2isNew = true;
            idv3isNew = true;

            if(t % 10000 == 0)
            std::cout << "Vertices: "<< FilterVetices.size() << ", triangles: " << FilterTriangles.size() << std::endl;
        }
        std::cout << "Vertices: "<< FilterVetices.size() << ", triangles: " << FilterTriangles.size() << std::endl;

        trianglesNum =  FilterTriangles.size();
        verticesNum = FilterVetices.size();
        normalNum = FilterVetices.size();

        for(int i = 0; i < trianglesNum; i++)
            triangles[i] = FilterTriangles[i];
        for(int i = 0; i < verticesNum; i++)
            vertices[i] = FilterVetices[i];

        saveOBJ("../meshes/stdMesh.obj",1);
    }
    void Mesh_IO::Mesh2MeshSTD(void){
        std::cout << this->verticesNum << ",  " << this->trianglesNum << std::endl;

        Vertex *FilterVetices = new Vertex[verticesNum];
        Polygon *FilterTriangles = new Polygon[trianglesNum];

        unsigned int idv1, idv2, idv3;
        Polygon tmpPoly;
        int v_size = 0;
        bool idv1isNew = true; 
        bool idv2isNew = true;
        bool idv3isNew = true;
        int cntT = 0;
        int cntV = 0;
        int index = 0;
        for (size_t t = 0; t < trianglesNum; t++){ 
            index = t * 3;
            // add vertices (contains normals an color)
            FilterVetices[index] =     vertices[index];
            FilterVetices[index + 1] = vertices[index + 1];
            FilterVetices[index + 2] = vertices[index + 2];

            for (size_t i = 0; i < index ; i++){
                // get vertex i-th from the filtered vector
                float3 v_pose = FilterVetices[i].position;
                // compare with vertices on the currecnt triangle
                if((vertices[triangles[t].vertexIndex.x - 1].position == v_pose) && idv1isNew){
                    idv1isNew = false;
                    idv1 = i + 1;
                }
                if((vertices[triangles[t].vertexIndex.y - 1].position == v_pose) && idv2isNew){
                    idv2isNew = false;
                    idv2 = i + 1;
                }
                if((vertices[triangles[t].vertexIndex.z - 1].position == v_pose) && idv3isNew){
                    idv3isNew = false;
                    idv3 = i + 1;
                }
            }

            // adding vertices into vector
            if(idv1isNew){
                idv1 = triangles[t].vertexIndex.x;
            }
            if(idv2isNew){
                idv2 = triangles[t].vertexIndex.y;
            }
            if(idv3isNew){
                idv3 = triangles[t].vertexIndex.z;
            }
            
            // adding triangles vertecies (IDs)
            tmpPoly.vertexIndex.x = idv1;
            tmpPoly.vertexIndex.y = idv2;
            tmpPoly.vertexIndex.z = idv3;

            tmpPoly.normalIndex.x = idv1;
            tmpPoly.normalIndex.y = idv2;
            tmpPoly.normalIndex.z = idv3;
            
            FilterTriangles[cntT] = tmpPoly;
            cntT++; 

            idv1isNew = true; 
            idv2isNew = true;
            idv3isNew = true;

            cntV = index + 2;
            if(t % 10000 == 0)
            std::cout << "vertices: "<< cntV << ", triangles: " << cntT << std::endl;
        }
        std::cout << "vertices: "<< cntV << ", triangles: " << cntT << std::endl;
        vertices = FilterVetices;
        triangles = FilterTriangles;
        saveOBJ("../meshes/stdMesh.obj",1);

        delete[] FilterVetices;
        delete[] FilterTriangles;
    }
    void Mesh_IO::readOBJ(const char* filename){
        FILE* file = fopen(filename, "r");
        if (!file) {
            fprintf(stdout, "Unable to open %s\n", filename);
            return ;
        }
        char line[256];
        if (file == NULL) {
            fprintf(stderr, "ERROR READING!!!\n");
            exit(-1);
        }
        verticesNum = 0;
        normalNum = 0;
        trianglesNum = 0;
        textureNum = 0;
        int fileType = 2;
        bool FirstLine = true;
        while (fgets(line, 255, file)){
            // test file type 
            if (line[0] == 'v' && FirstLine){
                std::vector<std::string> words;
                int wordCount = splitLine(line, words);//already erased the 'f' token
                if (wordCount -1 == 3){
                    fileType = 0;
                    // std::cout << "fileType: " << fileType << std::endl;
                }else if (wordCount -1 == 6){
                    fileType = 1;
                    // std::cout << "fileType: " << fileType << std::endl;
                }else{
                    std::cout << "Fatal Error ... " << fileType << ", wordCount: " << wordCount << std::endl;
                }
                FirstLine = false;
            }
            if(fileType == 1 && line[0] == 'v' && !FirstLine){
                std::vector<float> numbersInLine = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                if (line[1] == ' '){
                    line[0] = line[1] = ' ';
                    sscanf(line, "%f %f %f %f %f %f", &numbersInLine[0], &numbersInLine[1], &numbersInLine[2],
                                                    &numbersInLine[3], &numbersInLine[4], &numbersInLine[5]);
                    vertices[verticesNum].position = make_float3(numbersInLine[0], numbersInLine[2], numbersInLine[4]);
                    vertices[verticesNum].normal = make_float3(numbersInLine[1], numbersInLine[3], numbersInLine[5]);
                    verticesNum++;
                    normalNum++;
                }
            }else if (fileType == 0 && line[0] == 'v' && !FirstLine){
                float3 numbersInLine=make_float3(0.0, 0.0, 0.0);
                if (line[1] == ' '){
                    line[0] = line[1] = ' ';
                    sscanf(line, "%f %f %f", &numbersInLine.x, &numbersInLine.y, &numbersInLine.z);
                    vertices[verticesNum].position = numbersInLine;
                    verticesNum++;
                }else if (line[1] == 'n'){
                    line[0] = line[1] = ' ';
                    sscanf(line, "%f %f %f", &numbersInLine.x, &numbersInLine.y, &numbersInLine.z);
                    vertices[normalNum].normal = numbersInLine;
                    normalNum++;
                }else if (line[1] == 't'){
                    line[0] = line[1] = ' ';
                    sscanf(line, "%f %f %f", &numbersInLine.x, &numbersInLine.y, &numbersInLine.z);
                    vertices[textureNum].color = numbersInLine;
                    textureNum++;
                }
            }else if (line[0] == 'f' && !FirstLine){
                // std::cout << "f: " << trianglesNum << std::endl;
                line[0] =  ' ';
                int3 vertexIdx = make_int3(0,0,0);
                int3 normIdx = make_int3(0,0,0);
                int3 textureIdx = make_int3(0,0,0);
                //////////////////////////////////////////////////////////////////////////
                std::vector<std::string> words;
                int wordCount = splitLine(line, words);  //already erased the 'f' 
                if (wordCount == 3){
                    sscanf(line, "%d %d %d", &vertexIdx.x, &vertexIdx.y, &vertexIdx.z);
                }
                else if (wordCount == 6){
                    sscanf(line, "%d//%d %d//%d %d//%d", &vertexIdx.x, &normIdx.x,
                        &vertexIdx.y, &normIdx.y, &vertexIdx.z, &normIdx.z);
                }
                else if (wordCount == 9){
                    sscanf(line, "%d/%d/%d %d/%d/%d %d/%d/%d", &vertexIdx.x, &textureIdx.x, &normIdx.x,
                        &vertexIdx.y, &textureIdx.y, &normIdx.y, &vertexIdx.z, &textureIdx.z, &normIdx.z);
                }
                //////////////////////////////////////////////////////////////////////////
                Polygon tmp_poly; 

                tmp_poly.vertexIndex.x = vertexIdx.x - 1;
                tmp_poly.vertexIndex.y = vertexIdx.y - 1;
                tmp_poly.vertexIndex.z = vertexIdx.z - 1;

                if (normIdx.x != 0 || normIdx.y != 0 || normIdx.z != 0){
                    tmp_poly.normalIndex.x = normIdx.x - 1;
                    tmp_poly.normalIndex.y = normIdx.y - 1;
                    tmp_poly.normalIndex.z = normIdx.z - 1;
                }
                triangles[trianglesNum] = tmp_poly;
                ++trianglesNum;
            }else continue;
        }
        fclose(file);
        std::cout  << filename << "-> TNum: "<< trianglesNum << ", VNum: " << verticesNum  << ", NNum: " << normalNum << std::endl;
    }
    void Mesh_IO::saveOBJ(const char* filename, int mode)const {
    
        FILE* fp = fopen(filename, "w");
        if (!fp) return;
        if(mode == 0){

            fprintf(fp, "#vert = %d, face = %d\n", (int)verticesNum, (int)trianglesNum);

            for (int i=0; i< verticesNum; i++)
                fprintf(fp, "v %lf %lf %lf\n", vertices[i].position.x, vertices[i].position.y, vertices[i].position.z);

            for (int i=0; i< trianglesNum; i++)
                fprintf(fp, "f %d %d %d\n", (int)triangles[i].vertexIndex.x + 1, (int)triangles[i].vertexIndex.y + 1, (int)triangles[i].vertexIndex.z + 1);
            
            fclose(fp);

        }else if(mode == 1){

            fprintf(fp, "#vert = %d, normal: %d, face = %d\n", (int)verticesNum, (int)normalNum, (int)trianglesNum);

            for (int i = 0; i< verticesNum; i++)
                fprintf(fp, "v %lf %lf %lf\n", vertices[i].position.x, vertices[i].position.y, vertices[i].position.z);

            for (int i = 0; i < verticesNum; i++)
                fprintf(fp, "vn %lf %lf %lf \n", vertices[i].normal.x, vertices[i].normal.y, vertices[i].normal.z);

            for (int i = 0; i < trianglesNum; i++)
            fprintf(fp, "f %d//%d %d//%d %d//%d \n", triangles[i].vertexIndex.x + 1, triangles[i].normalIndex.x + 1, 
                                                     triangles[i].vertexIndex.y + 1, triangles[i].normalIndex.y + 1, 
                                                     triangles[i].vertexIndex.z + 1, triangles[i].normalIndex.z + 1);
            
            fclose(fp);
        }
        std::cout << "Mesh file: " << filename << " created... \n" << std:: endl;
        return;
    }
    void Mesh_IO::saveOBJVector(const char* filename, std::vector<Polygon>& Triangles, std::vector<Vertex>& Vetices)const {
        FILE* fp = fopen(filename, "w");
        if (!fp) return;
        fprintf(fp, "#vert = %d, face = %d\n", (int)Vetices.size(), (int)Triangles.size());
        for (int i = 0; i < Vetices.size(); i++)
            fprintf(fp, "v %lf %lf %lf \n", Vetices[i].position.x,
                                            Vetices[i].position.y,
                                            Vetices[i].position.z);

        for (int i = 0; i < Triangles.size(); i++)
            fprintf(fp, "vn %lf %lf %lf \n", Vetices[i].normal.x,
                                             Vetices[i].normal.y,
                                             Vetices[i].normal.z);

        for (int i = 0; i < Triangles.size(); i++)
            fprintf(fp, "f %d %d %d \n", Triangles[i].vertexIndex.x,
                                         Triangles[i].vertexIndex.y, 
                                         Triangles[i].vertexIndex.z);

        fclose(fp);
        std::cout << "Mesh file: " << filename << " created... \n" << std:: endl;
        return;
    }
    void Mesh_IO::updateVerticesNormals(void){

        for (int i=0; i<verticesNum; ++i){
            vertices[i].normal = make_float3(0.0f, 0.0f, 0.0f);
        }
        for (int tri=0; tri < trianglesNum; tri++){

            float3 v0 = vertices[triangles[tri].vertexIndex.x].position;
            float3 v1 = vertices[triangles[tri].vertexIndex.y].position;
            float3 v2 = vertices[triangles[tri].vertexIndex.z].position;

            float3 faceNormal = cross((v1 - v0), (v2 - v0));
            triangles[tri].normal = normalize(faceNormal);
            
            vertices[triangles[tri].vertexIndex.x].normal += faceNormal;
            vertices[triangles[tri].vertexIndex.y].normal += faceNormal;
            vertices[triangles[tri].vertexIndex.z].normal += faceNormal;
        }
        for (int i=0; i<verticesNum; ++i) {
            vertices[i].normal = normalize(vertices[i].normal);
            // std::cout << vertices[i].normal.x <<" , " << vertices[i].normal.y << ", " << vertices[i].normal.z << std::endl;
        }
        normalNum = verticesNum;
        updateNormalIndices();
    }
    void Mesh_IO::updateFacesNormals(void){
        for (int tri=0; tri < trianglesNum; tri++){
            triangles[tri].normal = make_float3(0.0f, 0.0f, 0.0f);

            triangles[tri].normal += vertices[triangles[tri].vertexIndex.x].normal;
            triangles[tri].normal += vertices[triangles[tri].vertexIndex.y].normal;
            triangles[tri].normal += vertices[triangles[tri].vertexIndex.z].normal;
            triangles[tri].normal = normalize(triangles[tri].normal);

            triangles[tri].normalIndex.x = triangles[tri].vertexIndex.x;
            triangles[tri].normalIndex.y = triangles[tri].vertexIndex.y;
            triangles[tri].normalIndex.z = triangles[tri].vertexIndex.z;
        }
        normalNum = verticesNum;
    }
    void Mesh_IO::updateNormalIndices(void){
        for (int tri=0; tri < trianglesNum; tri++){
            triangles[tri].normalIndex.x = triangles[tri].vertexIndex.x;
            triangles[tri].normalIndex.y = triangles[tri].vertexIndex.y;
            triangles[tri].normalIndex.z = triangles[tri].vertexIndex.z;
        }
    }
    void Mesh_IO::meshInfo()const{
        fprintf(stdout, "VertexNum: %d, PolyNum: %d \n", this->verticesNum, this->trianglesNum);
    }
    // void Mesh_IO::push_backVertex(float3 vertexPose, unsigned int id){
    //     switch (id%3){
    //     case 0:{
    //         triangles[id/3].v0.position = vertexPose;
    //     break;
    //     }case 1:{
    //         triangles[id/3].v1.position = vertexPose;
    //     break;
    //     }case 2:{
    //         triangles[id/3].v2.position = vertexPose;
    //     break;
    //     }
    //     }
    // }
    // void Mesh_IO::push_backNormal(float3 vertexNormal, unsigned int id){
    //     switch (id%3)
    //     {
    //     case 0:{
    //         triangles[id/3].v0.normal = vertexNormal;
    //     break;
    //     }case 1:{
    //         triangles[id/3].v1.normal = vertexNormal;
    //     break;
    //     }case 2:{
    //         triangles[id/3].v2.normal = vertexNormal;
    //     break;
    //     }
    //     }
    // }
    // void Mesh_IO::push_backTexture(float3 vertexTexture, unsigned int id){
    //     // switch (id%3)
    //     // {
    //     // case 0:{
    //     //     triangles[id/3].v0.normal = vertexNormal;
    //     // break;
    //     // }case 1:{
    //     //     triangles[id/3].v1.normal = vertexNormal;
    //     // break;
    //     // }case 2:{
    //     //     triangles[id/3].v2.normal = vertexNormal;
    //     // break;
    //     // }
    //     // }
    // }
    // void Mesh_IO::readPLY(const char* filename){
    //     FILE* fp = fopen(filename, "r");
    //     if (!fp) {
    //         fprintf(stdout, "unable to open %s\n", filename);
    //         return ;
    //     }
    //     fseek(fp, 0, SEEK_END);
    //     fseek(fp, 0, SEEK_SET);
    //     while (fgetc(fp) != '\n');

    //     char buffer[256]; int nVert, nFace;
    //     fscanf(fp,"%100s ", buffer);
    //     while (strcmp(buffer, "end_header") != 0){
    //         if (strcmp(buffer, "format") == 0){
    //             fscanf(fp,"%100s ", buffer);
    //             if (strcmp(buffer, "ascii") != 0){
    //                 fprintf(stdout, "PLY file format error: PLY ASCII support only.");
    //                 return;
    //             }
    //         } else if (strcmp(buffer, "element") == 0){
    //             fscanf(fp,"%100s ", buffer);
    //             if (strcmp(buffer, "vertex") == 0){
    //                 fscanf(fp,"%100s", buffer);
    //                 nVert = atoi(buffer);
    //             } else if (strcmp(buffer, "face") == 0){
    //                 fscanf(fp,"%100s", buffer);
    //                 nFace = atoi(buffer);
    //             }
    //         }
    //         fscanf(fp,"%100s ", buffer);
    //     }
    //     verticesNum = nVert;  vertexPose.reserve(verticesNum);
    //     trianglesNum = nFace;  triangles.reserve(nFace);
    //     double x, y, z;
    //     for (int i=0; i<nVert; ++i)
    //     {
    //         fgets(buffer, 255, fp);
    //         sscanf(buffer, "%lf %lf %lf", &x, &y, &z);
    //         vertexPose.push_back(Eigen::Vector3d(x, y, z));
    //     }
    //     int i[3], v_n;
    //     while (fgets(buffer, 255, fp))
    //     {
    //         if (buffer[0] == ' ' || buffer[0] == '\n') continue;
    //         sscanf(buffer, "%d %d %d %d", &v_n, i, i+1, i+2);
    //         if (v_n != 3) {
    //             fprintf(stdout, "warning: the %s is not a triangle mesh, stop reading file\n", filename);
    //             fclose(fp);
    //             return;
    //         }
    //         Polygon _poly; 
    //         _poly.vertexIndex[0] = i[0];
    //         _poly.vertexIndex[1] = i[1];
    //         _poly.vertexIndex[2] = i[2];
    //         triangles.push_back(_poly);
    //     }
    //     fclose(fp);
    //     updateNorm();
    // }
    
    // void Mesh_IO::savePLY(const char* filename)const {
    //     if (this->vertexNormal.empty()) {		
    //         fprintf(stderr, "error : make sure the normal is not empty.\n");
    //         return;
    //     }
    //     FILE* fp = fopen(filename, "w");
    //     if (!fp) {
    //         fprintf(stderr, "warning : unable to open %s when saving pcd to ply file.\n", filename);
    //         return;
    //     }
    //     fprintf(fp, "ply\n");
    //     fprintf(fp, "format ascii 1.0\n");
    //     fprintf(fp, "element vertex %d\n", (uint32_t)vertexPose.size());
    //     fprintf(fp, "property float x\n");
    //     fprintf(fp, "property float y\n");
    //     fprintf(fp, "property float z\n");
    //     fprintf(fp, "property float nx\n");
    //     fprintf(fp, "property float ny\n");
    //     fprintf(fp, "property float nz\n");
    //     fprintf(fp, "property uchar red\n");
    //     fprintf(fp, "property uchar green\n");
    //     fprintf(fp, "property uchar blue\n");
    //     fprintf(fp, "end_header\n");
        
    //     for (size_t i=0; i<vertexPose.size(); ++i) {
    //         fprintf(fp, "%f %f %f %f %f %f %d %d %d\n", 
    //             vertexPose[i][0], vertexPose[i][1], vertexPose[i][2],
    //             vertexNormal[i][0], vertexNormal[i][1], vertexNormal[i][2],
    //             0, 255, 255);
    //     }
    //     fclose(fp);
    // }
    // void Mesh_IO::downSample(size_t sampledMesh_idx, double maxDist){
    //     // pcl::PointCloud<pcl::PointNormal>::Ptr src_pcd = _trimesh2pcl_data(this);
    //     // std::vector<int> normal_sampling_indices;
    //     // std::vector<int> uniform_sampling_indices;

    //     // const int NORMAL_SAMPLE_BIN_NUMBER = 30; 
    //     // const int NORMAL_SAMPLE_NUMBER = 200;
    //     // if (1)
    //     // {
    //     //     pcl::NormalSpaceSampling<pcl::PointNormal, pcl::Normal> normal_sampler;
    //     //     pcl::PointCloud<pcl::Normal>::Ptr tri_normal(new pcl::PointCloud<pcl::Normal>);
    //     //     {
    //     //         tri_normal->resize(src_pcd->points.size());
    //     //         for (int i=0; i<src_pcd->points.size(); ++i) {
    //     //             tri_normal->points[i].normal_x = src_pcd->points[i].normal_x;
    //     //             tri_normal->points[i].normal_y = src_pcd->points[i].normal_y;
    //     //             tri_normal->points[i].normal_z = src_pcd->points[i].normal_z;
    //     //         }
    //     //     }
    //     //     normal_sampler.setSeed(time(NULL));
    //     //     normal_sampler.setInputCloud(src_pcd);
    //     //     normal_sampler.setNormals(tri_normal);
    //     //     normal_sampler.setBins(NORMAL_SAMPLE_BIN_NUMBER, NORMAL_SAMPLE_BIN_NUMBER, NORMAL_SAMPLE_BIN_NUMBER);
    //     //     normal_sampler.setSample(NORMAL_SAMPLE_NUMBER);
    //     //     normal_sampler.filter(normal_sampling_indices);
    //     // }
    // 	// // {
    // 	// // 	typedef pcl::PointXYZ PointType;
    // 	// // 	pcl::UniformSampling<PointType> uniform_sampler;
    // 	// // 	pcl::PointCloud<PointType>::Ptr tri_vertex(new pcl::PointCloud<PointType>);
    // 	// // 	{
    // 	// // 		tri_vertex->resize(src_pcd->points.size());
    // 	// // 		for (int i=0; i<src_pcd->points.size(); ++i) {
    // 	// // 			tri_vertex->points[i].x = src_pcd->points[i].x;
    // 	// // 			tri_vertex->points[i].y = src_pcd->points[i].y;
    // 	// // 			tri_vertex->points[i].z = src_pcd->points[i].z;
    // 	// // 		}
    // 	// // 	}
    // 	// // 	pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    // 	// // 	uniform_sampler.setInputCloud(tri_vertex);
    // 	// // 	uniform_sampler.setSearchMethod(tree);
    // 	// // 	uniform_sampler.setRadiusSearch(max_dist);
    // 	// // 	{
    // 	// // 		pcl::PointCloud<int> keypoints_src_idx;
    // 	// // 		uniform_sampler.compute(keypoints_src_idx);
    // 	// // 		uniform_sampling_indices.clear();
    // 	// // 		uniform_sampling_indices.resize(keypoints_src_idx.size());
    // 	// // 		std::copy(keypoints_src_idx.begin(), keypoints_src_idx.end(), uniform_sampling_indices.begin());
    // 	// // 	}
    // 	// // }

    //     // //merge the sampling result to output
    //     // sample_index.clear();
    //     // sample_index.resize(uniform_sampling_indices.size() + normal_sampling_indices.size());
    //     // std::sort(uniform_sampling_indices.begin(), uniform_sampling_indices.end());
    //     // std::sort(normal_sampling_indices.begin(), normal_sampling_indices.end());

    //     // std::merge(uniform_sampling_indices.begin(), uniform_sampling_indices.end(),
    //     //     normal_sampling_indices.begin(), normal_sampling_indices.end(), 
    //     //     sample_index.begin());
    //     // std::vector<int>::iterator last_it = std::unique(sample_index.begin(), sample_index.end());
    //     // sample_index.erase(last_it, sample_index.end());
    //     // printf("****uniform sampling count : %d\n****normal sampling count : %d\n", uniform_sampling_indices.size(),
    //     //     normal_sampling_indices.size());
    //     // printf("****sampling count : %d\n", sample_index.size());
    // }
    // void Mesh_IO::pointCloud2PolyMesh(pcl::PolygonMesh& triangles, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud){
    //     // Normal estimation*
    //     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    //     pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    //     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    //     tree->setInputCloud (cloud);
    //     n.setInputCloud (cloud);
    //     n.setSearchMethod (tree);
    //     n.setKSearch (20);
    //     n.compute (*normals);
    //     //* normals should not contain the point normals + surface curvatures

    //     // Concatenate the XYZ and normal fields*
    //     pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    //     pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //     //* cloud_with_normals = cloud + normals

    //     // Create search tree*
    //     pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    //     tree2->setInputCloud (cloud_with_normals);

    //     // Initialize objects
    //     pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    //     // Set the maximum distance between connected points (maximum edge length)
    //     gp3.setSearchRadius (0.3);

    //     // Set typical values for the parameters
    //     gp3.setMu (2.5);
    //     gp3.setMaximumNearestNeighbors (100);
    //     gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    //     gp3.setMinimumAngle(M_PI/18); // 10 degrees
    //     gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    //     gp3.setNormalConsistency(false);

    //     // Get result
    //     gp3.setInputCloud (cloud_with_normals);
    //     gp3.setSearchMethod (tree2);
    //     gp3.reconstruct (triangles);

    //     // Additional vertex information
    //     std::vector<int> parts = gp3.getPartIDs();
    //     std::vector<int> states = gp3.getPointStates();
    // }
    // void Mesh_IO::polyMesh2Mesh(Mesh& mesh){
    // }
    // void Mesh_IO::mesh2PointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr& cloud) {	
    //     for (size_t i=0; i<this->verticesNum; i++) {
    //         pcl::PointNormal v;
    //         v.x = (float)this->vertexPose[i][0];
    //         v.y = (float)this->vertexPose[i][1];
    //         v.z = (float)this->vertexPose[i][2];
    //         v.normal_x = (float)this->vertexNormal[i][0];
    //         v.normal_y = (float)this->vertexNormal[i][1];
    //         v.normal_z = (float)this->vertexNormal[i][2];
    //         cloud->points.push_back(v);
    //     }
    //     cloud->width = this->verticesNum; 
    //     cloud->height = 1; 
    //     cloud->is_dense = false;
    // }

}   //end namespace geometry
}   // namespace DynaMap
















// void Mesh::getBoundingBox(Eigen::Vector3d& Min, Eigen::Vector3d& Max){
//         const double LIMIT_MAX = std::numeric_limits<double>::max();
//         const double LIMIT_MIN = std::numeric_limits<double>::min();
//         Min = Eigen::Vector3d(LIMIT_MAX, LIMIT_MAX, LIMIT_MAX); Max = Eigen::Vector3d(LIMIT_MIN, LIMIT_MIN, LIMIT_MIN);
//         for (auto it = vertexPose.begin(); it != vertexPose.end(); ++it)
//         {
//             Eigen::Vector3d& refThis = *it;
//             if (refThis[0] < Min[0])		Min[0] = refThis[0];
//             else if (refThis[0] > Max[0])	Max[0] = refThis[0];

//             if (refThis[1] < Min[1])		Min[1] = refThis[1];
//             else if (refThis[1] > Max[1])	Max[1] = refThis[1];

//             if (refThis[2] < Min[2])		Min[2] = refThis[2];
//             else if (refThis[2] > Max[2])	Max[2] = refThis[2];
//         }
//         return;
//     }
// void Mesh::render(DisplayMode mode){
//         static bool bbox_flag = false;
//         if (!bbox_flag)
//             this->getBoundingBox(this->BBox._min, this->BBox._max);
//         Eigen::Vector3d _center;
//         _center = (this->BBox._max + this->BBox._min)*0.5;
//         float scale_factor = (float)(this->BBox._max - this->BBox._min).norm();
//         scale_factor = 1.0f / scale_factor;
//         //glScalef(scale_factor, scale_factor, scale_factor);//ע��˳��
//         //glTranslatef(-_center[0], -_center[1], -_center[2]);
//         //	drawBoundingBox(BBox._min, BBox._max);
//         //assert(vertexNormal.size() == vertexPose.size());
//         if (mode == POINT_MODE)
//         {
//             if (trianglesNum == 0)
//             {
//                 glPushMatrix();
//                 glBegin(GL_POINTS);
//                 for (int i=0; i<this->verticesNum; ++i)
//                 {
//                     glNormal3dv(&vertexNormal[i][0]);
//                     glVertex3dv(&vertexPose[i][0]);
//                 }
//                 glEnd();
//                 glPopMatrix();

//                 return;
//             } 
//             else
//                 glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
//         }else if (mode == EDGE_MODE)
//         {
//             glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
//         }
//         else if (mode == FACE_MODE) {
//             glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
//         }

//         // if (faceNormal.size() == 0) updateNorm();

//         glPushMatrix();
//         glBegin(GL_TRIANGLES);
//         for (int i=0; i<trianglesNum; ++i)
//         {
//             for (int j=0; j<=2; ++j)
//             {
//                 glNormal3dv(&vertexNormal[triangles[i].normalIndex[j]][0]);
//                 glVertex3dv(&vertexPose[triangles[i].vertexIndex[j]][0]);
//             }
//         }
//         glEnd();

//         glPopMatrix();
//     }
// struct EdgeLink{
    //     unsigned int v[2];
    //     EdgeLink(unsigned int v1, unsigned int v2) 
    //     {
    //         v[0] = v1 < v2 ? v1 : v2;
    //         v[1] = v1 < v2 ? v2 : v1;
    //     }
    //     bool operator < (const EdgeLink& ref) const 
    //     {
    //         if (v[0] != ref.v[0])  return v[0] < ref.v[0];
    //         else return v[1] < ref.v[1];
    //     }
    // };
// static pcl::PointCloud<pcl::PointNormal>::Ptr _trimesh2pcl_data(const TriMesh& trimesh) {	
    //     pcl::PointCloud<pcl::PointNormal>::Ptr out(new pcl::PointCloud<pcl::PointNormal>);
    //     out->points.reserve(trimesh.vert_num);
    //     int vi = 0;
    //     for (int i=0; i<trimesh.vert_num; ++i) {
    //         pcl::PointNormal temp_v;
    //         temp_v.x = trimesh.vertex_coord[i][0];
    //         temp_v.y = trimesh.vertex_coord[i][1];
    //         temp_v.z = trimesh.vertex_coord[i][2];
    //         temp_v.normal_x = trimesh.norm_coord[i][0];
    //         temp_v.normal_y = trimesh.norm_coord[i][1];
    //         temp_v.normal_z = trimesh.norm_coord[i][2];
    //         out->points.push_back(temp_v);
    //         ++vi;
    //     }
    //     out->width = vi; out->height = 1; out->is_dense = false;
    //     return out;
    // }

    // Geometry::Mesh  Mesh::PolyMesh2Mesh(pcl::PolygonMesh triangles){

    // }

    // Geometry::Mesh  Mesh::TriMesh2Mesh(pcl::PolygonMesh triangles){
    //    return 
    // }
    
    // static pcl::PointCloud<pcl::PointNormal>::Ptr Mesh2pcl(const TriMesh& trimesh) {	
    //     pcl::PointCloud<pcl::PointNormal>::Ptr out(new pcl::PointCloud<pcl::PointNormal>);
    //     out->points.reserve(trimesh.vert_num);
    //     int vi = 0;
    //     for (int i=0; i<trimesh.vert_num; ++i) {
    //         pcl::PointNormal temp_v;
    //         temp_v.x = trimesh.vertex_coord[i][0];
    //         temp_v.y = trimesh.vertex_coord[i][1];
    //         temp_v.z = trimesh.vertex_coord[i][2];
    //         temp_v.normal_x = trimesh.norm_coord[i][0];
    //         temp_v.normal_y = trimesh.norm_coord[i][1];
    //         temp_v.normal_z = trimesh.norm_coord[i][2];
    //         out->points.push_back(temp_v);
    //         ++vi;
    //     }
    //     out->width = vi; out->height = 1; out->is_dense = false;
    //     return out;
    // }
  
    // void util_Mesh::prepareFaceNeighbours(std::vector<std::vector<int>>& neighbours){
    //     if (!neighbours.empty()) {
    //         fprintf(stdout, "neighbours is not empty, do nothing...\n");
    //         return;
    //     }
    //     using std::multimap;
    //     multimap<EdgeLink, int> edgeFaceMap;
    //     for (int i=0; i<trianglesNum; ++i)
    //     {
    //         edgeFaceMap.insert(std::make_pair(EdgeLink(PolygonIndex[i].vertexIndex[0], PolygonIndex[i].vertexIndex[1]), i));
    //         edgeFaceMap.insert(std::make_pair(EdgeLink(PolygonIndex[i].vertexIndex[1], PolygonIndex[i].vertexIndex[2]), i));
    //         edgeFaceMap.insert(std::make_pair(EdgeLink(PolygonIndex[i].vertexIndex[2], PolygonIndex[i].vertexIndex[0]), i));
    //     }
    //     neighbours.resize(trianglesNum);

    //     for (int i=0; i<trianglesNum; ++i)
    //     {
    //         for (int j=0; j<3; ++j)
    //         {
    //             EdgeLink edgeLink(PolygonIndex[i].vertexIndex[j], PolygonIndex[i].vertexIndex[(j+1)%3]);
    //             auto lowerIter = edgeFaceMap.lower_bound(edgeLink); //it must return true
    //             assert(lowerIter != edgeFaceMap.end());
    //             if (lowerIter->second == i) ++lowerIter;
    //             neighbours[i].push_back(lowerIter->second);
    //         }
    //     }

    //     return;
    // }
    // static void drawBoundingBox(Vector3d& _min, Vector3d& _max){
    //     Vector3d box_verts[8];
    //     box_verts[0] = _min;
    //     box_verts[1] = _min; box_verts[1][0] = _max[0];
    //     box_verts[2] = _max; box_verts[2][2] = _min[2];
    //     box_verts[3] = _min; box_verts[3][1] = _max[1];
    //     box_verts[4] = _min; box_verts[4][2] = _max[2];
    //     box_verts[5] = _max; box_verts[5][1] = _min[1];
    //     box_verts[6] = _max;
    //     box_verts[7] = _max; box_verts[7][0] = _min[0];
    //     GLubyte indices[6][4] = {0, 3, 2, 1, 
    //                         4, 5, 6, 7,
    //                         2, 6, 5, 1,
    //                         4, 7, 3, 0,
    //                         0, 1, 5, 4,
    //                         2, 3, 7, 6};
    //     for (int i=0; i<6; ++i)
    //     {
    //         glBegin(GL_LINE_LOOP);
    //         for (int j=0; j<4; ++j)
    //             glVertex3dv(&box_verts[indices[i][j]][0]);
    //         glEnd();
    //     }
    // }
    // void Skeleton::read(const char* filename){
    //     pos.clear(); parents.clear();
    //     FILE* fp = NULL;
    //     fopen_s(&fp, filename, "r");
    //     if (!fp) {
    //         fprintf(stdout, "unable to open %s\n", filename);
    //         return;
    //     }
    //     Eigen::Vector3d _joint_pos; int _joint_parent_id, _joint_id;
    //     char line[1024];
    //     while (fgets(line, 1024, fp))
    //     {
    //         sscanf_s(line, "%d %lf %lf %lf %d", &_joint_id, &_joint_pos(0), 
    //             &_joint_pos(1), &_joint_pos(2), &_joint_parent_id);
    //         pos.push_back(_joint_pos);
    //         parents.push_back(_joint_parent_id);
    //     }		
    //     fclose(fp);
    // }
    // void Skeleton::render(float sphereSize /* = 0.05 */, float lineWidth /* = 3.0 */){
    //     glDisable(GL_DEPTH);

    //     static float d_color[4] = {0.980392f, 0.129412f, 0.111765f, 1.0f};
    //     glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, d_color);
    //     for (size_t i=0; i<pos.size(); ++i)
    //     {
    //         glPushMatrix();
    //         glTranslated(pos[i][0], pos[i][1], pos[i][2]);
    //         glutSolidSphere(sphereSize, 20, 20);
    //         glPopMatrix();
    //     }

    //     glDisable(GL_LIGHTING);
    //     glDisable(GL_DEPTH);
    //     glColor3d(0.1, 0.1, 0.85);
    //     glLineWidth(lineWidth);
    //     glBegin(GL_LINES);
    //     for (size_t i=0; i<pos.size(); ++i)
    //     {
    //         int p = parents[i];
    //         if (p < 0) continue;
    //         glVertex3dv(&pos[i][0]);
    //         glVertex3dv(&pos[p][0]);
    //     }
    //     glEnd();
    //     glLineWidth(1.0);
    //     glEnable(GL_LIGHTING);
    //     glEnable(GL_DEPTH);
    // }
