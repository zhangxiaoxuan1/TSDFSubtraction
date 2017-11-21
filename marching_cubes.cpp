// include required libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "marching_cubes_table.h"
#include "marching_cubes.h"
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/io/ply_io.h>

struct Cell {
    pcl::PointXYZ vert[8];
    float val[8];
    int indicator[8];
} ;
int count = 0;
float isolevel = 0;
int strangeCount = 0;
int minX = 512,maxX = -1,minY = 512,maxY = -1,minZ = 512,maxZ = -1;
int tolerance = 0;

// Calculate the intersection on an edge
pcl::PointXYZ VertexInterp(float isolevel,pcl::PointXYZ p1,pcl::PointXYZ p2,float valp1,float valp2) {
    float mu;
    pcl::PointXYZ p;
    // If jump is too big, it is suspicious
    if (std::abs(valp1-valp2) > 0.8){
        strangeCount ++;
    }

    if (std::abs(isolevel-valp1) < 0.00001){
        return(p1);
    }
    if (std::abs(isolevel-valp2) < 0.00001){
        return(p2);
    }
    if (std::abs(valp1-valp2) < 0.00001){
        return(p1);
    }
    mu = (isolevel - valp1) / (valp2 - valp1);
    p.x = p1.x + mu * (p2.x - p1.x);
    p.y = p1.y + mu * (p2.y - p1.y);
    p.z = p1.z + mu * (p2.z - p1.z);
    return(p);
}


int process_cube(Cell grid, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int index) {

    int cubeindex = 0;
    if (grid.val[0] <= isolevel) cubeindex |= 1;
    if (grid.val[1] <= isolevel) cubeindex |= 2;
    if (grid.val[2] <= isolevel) cubeindex |= 4;
    if (grid.val[3] <= isolevel) cubeindex |= 8;
    if (grid.val[4] <= isolevel) cubeindex |= 16;
    if (grid.val[5] <= isolevel) cubeindex |= 32;
    if (grid.val[6] <= isolevel) cubeindex |= 64;
    if (grid.val[7] <= isolevel) cubeindex |= 128;

    // Cube is entirely in/out of the surface
    if (edgeTable[cubeindex] == 0){
        return(0);
    }
    pcl::PointXYZ vertlist[12];

    // Find the points where the surface intersects the cube
    strangeCount = 0;
    if (edgeTable[cubeindex] & 1){
        vertlist[0] = VertexInterp(isolevel,grid.vert[0],grid.vert[1],grid.val[0],grid.val[1]);
    }
    if (edgeTable[cubeindex] & 2){
        vertlist[1] = VertexInterp(isolevel,grid.vert[1],grid.vert[2],grid.val[1],grid.val[2]);
    }
    if (edgeTable[cubeindex] & 4){
        vertlist[2] = VertexInterp(isolevel,grid.vert[2],grid.vert[3],grid.val[2],grid.val[3]);
    }
    if (edgeTable[cubeindex] & 8){
        vertlist[3] = VertexInterp(isolevel,grid.vert[3],grid.vert[0],grid.val[3],grid.val[0]);
    }
    if (edgeTable[cubeindex] & 16){
        vertlist[4] = VertexInterp(isolevel,grid.vert[4],grid.vert[5],grid.val[4],grid.val[5]);
    }
    if (edgeTable[cubeindex] & 32){
        vertlist[5] = VertexInterp(isolevel,grid.vert[5],grid.vert[6],grid.val[5],grid.val[6]);
    }
    if (edgeTable[cubeindex] & 64){
        vertlist[6] = VertexInterp(isolevel,grid.vert[6],grid.vert[7],grid.val[6],grid.val[7]);
    }
    if (edgeTable[cubeindex] & 128){
        vertlist[7] = VertexInterp(isolevel,grid.vert[7],grid.vert[4],grid.val[7],grid.val[4]);
    }
    if (edgeTable[cubeindex] & 256){
        vertlist[8] = VertexInterp(isolevel,grid.vert[0],grid.vert[4],grid.val[0],grid.val[4]);
    }
    if (edgeTable[cubeindex] & 512){
        vertlist[9] = VertexInterp(isolevel,grid.vert[1],grid.vert[5],grid.val[1],grid.val[5]);
    }
    if (edgeTable[cubeindex] & 1024){
        vertlist[10] = VertexInterp(isolevel,grid.vert[2],grid.vert[6],grid.val[2],grid.val[6]);
    }
    if (edgeTable[cubeindex] & 2048){
        vertlist[11] = VertexInterp(isolevel,grid.vert[3],grid.vert[7],grid.val[3],grid.val[7]);
    }

    if(strangeCount >=1 && (grid.val[0]==0 || grid.val[1]==0 || grid.val[2]==0 ||grid.val[3]==0 ||grid.val[4]==0 ||grid.val[5]==0 ||grid.val[6]==0 ||grid.val[7]==0)){
        return (0);
    }
    // Only recreate the largest component
    if (grid.indicator[0]!=index && grid.indicator[1]!=index && grid.indicator[2]!=index && grid.indicator[3]!=index &&
        grid.indicator[4]!=index &&grid.indicator[5]!=index && grid.indicator[6]!=index && grid.indicator[7]!=index){
        return(0);
    }

    // Create the triangle
    int triangle_count = 0;
    pcl::PointXYZ triangle[3];
    for (int i=0;triTable[cubeindex][i]!=-1;i+=3) {
        triangle[0] = vertlist[triTable[cubeindex][i  ]];
        triangle[1] = vertlist[triTable[cubeindex][i+1]];
        triangle[2] = vertlist[triTable[cubeindex][i+2]];
        triangle[0].x = triangle[0].x*float(0.606);
        triangle[1].x = triangle[1].x*float(0.606);
        triangle[2].x = triangle[2].x*float(0.606);
        triangle[0].y = triangle[0].y*float(0.505);
        triangle[1].y = triangle[1].y*float(0.505);
        triangle[2].y = triangle[2].y*float(0.505);
        cloud->push_back(triangle[0]);
        cloud->push_back(triangle[1]);
        cloud->push_back(triangle[2]);
        triangle_count++;
    }
    return(triangle_count);

}

void MarchingCubes::marchingCube(
        int matrixSize[3],
        std::vector<std::vector<std::vector<short int>>>& grid,
        std::vector<std::vector<std::vector<float>>>& tsdfGrid,
        int index,pcl::PointCloud<pcl::PointXYZRGB>::Ptr componentCloud)
{
    std::cout << "Running Marching Cubes..." << std::endl;
    std::cout.flush();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr ptrCloud(&cloud);
    pcl::PolygonMesh  triangleMesh;
    pcl::PolygonMesh::Ptr ptrMesh(&triangleMesh);
    struct Cell gridcell;
    double vert[8][3];
    float val[8];

    // Calculate gridcells
    for(int i = 0;i < matrixSize[0] - 1;i++){
        std::cout << "\rScanning layer " << i << "/" << matrixSize[0] <<"...";
        std::cout.flush();
        for(int j = 0; j < matrixSize[1] - 1; j++){
            for(int k = 0; k < matrixSize[2] - 1; k++){
                // Retrieve 8 vertices of the gridcell.
                int position_arr[8][3] = { {i,j,k+1},
                                           {i+1,j,k+1},
                                           {i+1,j,k},
                                           {i,j,k},
                                           {i,j+1,k+1},
                                           {i+1,j+1,k+1},
                                           {i+1,j+1,k},
                                           {i,j+1,k} };
                for(int l=0;l<8;l++){
                    gridcell.val[l] = tsdfGrid[position_arr[l][0]][position_arr[l][1]][position_arr[l][2]];
                    gridcell.indicator[l] = grid[position_arr[l][0]][position_arr[l][1]][position_arr[l][2]];
                    gridcell.vert[l].x = position_arr[l][0];
                    gridcell.vert[l].y = position_arr[l][1];
                    gridcell.vert[l].z = position_arr[l][2];
                }
                count+=process_cube(gridcell,ptrCloud,index);
            }
        }
    }

    std::cout << std::endl << "A total of " << count << " triangles are processed." << std::endl;

    // Process the points and convert to mesh
    pcl::PolygonMesh Mesh;
    pcl::PolygonMesh::Ptr mesh_ptr(&Mesh);
    pcl::toPCLPointCloud2 (cloud, mesh_ptr->cloud);
    for (uint32_t i = 0; i < count; i++)
    {
        pcl::Vertices v;
        v.vertices.push_back (i*3 + 0);
        v.vertices.push_back (i*3 + 1);
        v.vertices.push_back (i*3 + 2);
        mesh_ptr->polygons.push_back(v);
    }
    std::cout << "Saving PLY file..." << std::endl;
    pcl::io::savePLYFile ("subtracted_TSDF_mesh.ply", Mesh);
    std::cout << "PLY file saved." << std::endl;

    std::cout << "Visualzing point clouds..."
              << std::endl
              << "The point cloud shows all the connected components in different colors. The white one is the largest."
              << std::endl;
    // Show cloud after marching cubes
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(componentCloud);
    while (!viewer.wasStopped ())
    {

    }

}
