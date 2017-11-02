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
    int val[8];
    float tsdfVal[8];
} ;
int count = 0;
float isolevel = 0;
// Calculate the intersection on an edge
pcl::PointXYZ VertexInterp(pcl::PointXYZ p1,pcl::PointXYZ p2,float valp1,float valp2) {
    float mu;
    pcl::PointXYZ p;

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

    // Only recreate the largest component
    if (grid.val[0]!=index && grid.val[1]!=index && grid.val[2]!=index && grid.val[3]!=index &&
            grid.val[4]!=index &&grid.val[5]!=index && grid.val[6]!=index && grid.val[7]!=index){
        return(0);
    }
    if (grid.tsdfVal[0] < isolevel) cubeindex |= 1;
    if (grid.tsdfVal[1] < isolevel) cubeindex |= 2;
    if (grid.tsdfVal[2] < isolevel) cubeindex |= 4;
    if (grid.tsdfVal[3] < isolevel) cubeindex |= 8;
    if (grid.tsdfVal[4] < isolevel) cubeindex |= 16;
    if (grid.tsdfVal[5] < isolevel) cubeindex |= 32;
    if (grid.tsdfVal[6] < isolevel) cubeindex |= 64;
    if (grid.tsdfVal[7] < isolevel) cubeindex |= 128;

    // Cube is entirely in/out of the surface
    if (edgeTable[cubeindex] == 0){
        return(0);
    }
    pcl::PointXYZ vertlist[12];

    // Find the points where the surface intersects the cube
    if (edgeTable[cubeindex] & 1){
        vertlist[0] = VertexInterp(grid.vert[0],grid.vert[1],grid.tsdfVal[0],grid.tsdfVal[1]);
    }
    if (edgeTable[cubeindex] & 2){
        vertlist[1] = VertexInterp(grid.vert[1],grid.vert[2],grid.tsdfVal[1],grid.tsdfVal[2]);
    }
    if (edgeTable[cubeindex] & 4){
        vertlist[2] = VertexInterp(grid.vert[2],grid.vert[3],grid.tsdfVal[2],grid.tsdfVal[3]);
    }
    if (edgeTable[cubeindex] & 8){
        vertlist[3] = VertexInterp(grid.vert[3],grid.vert[0],grid.tsdfVal[3],grid.tsdfVal[0]);
    }
    if (edgeTable[cubeindex] & 16){
        vertlist[4] = VertexInterp(grid.vert[4],grid.vert[5],grid.tsdfVal[4],grid.tsdfVal[5]);
    }
    if (edgeTable[cubeindex] & 32){
        vertlist[5] = VertexInterp(grid.vert[5],grid.vert[6],grid.tsdfVal[5],grid.tsdfVal[6]);
    }
    if (edgeTable[cubeindex] & 64){
        vertlist[6] = VertexInterp(grid.vert[6],grid.vert[7],grid.tsdfVal[6],grid.tsdfVal[7]);
    }
    if (edgeTable[cubeindex] & 128){
        vertlist[7] = VertexInterp(grid.vert[7],grid.vert[4],grid.tsdfVal[7],grid.tsdfVal[4]);
    }
    if (edgeTable[cubeindex] & 256){
        vertlist[8] = VertexInterp(grid.vert[0],grid.vert[4],grid.tsdfVal[0],grid.tsdfVal[4]);
    }
    if (edgeTable[cubeindex] & 512){
        vertlist[9] = VertexInterp(grid.vert[1],grid.vert[5],grid.tsdfVal[1],grid.tsdfVal[5]);
    }
    if (edgeTable[cubeindex] & 1024){
        vertlist[10] = VertexInterp(grid.vert[2],grid.vert[6],grid.tsdfVal[2],grid.tsdfVal[6]);
    }
    if (edgeTable[cubeindex] & 2048){
        vertlist[11] = VertexInterp(grid.vert[3],grid.vert[7],grid.tsdfVal[3],grid.tsdfVal[7]);
    }
    for(int i=0;i<12;i++){
        if(vertlist[i].x < 0){
            return(0);
        }
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
        for(int j = 0; j < matrixSize[1] - 1; j++){
            for(int k = 0; k < matrixSize[2] - 1; k++){
                int position_arr[8][3] = { {i,j,k+1},
                                           {i+1,j,k+1},
                                           {i+1,j,k},
                                           {i,j,k},
                                           {i,j+1,k+1},
                                           {i+1,j+1,k+1},
                                           {i+1,j+1,k},
                                           {i,j+1,k} };
                for(int l=0;l<8;l++){
                    gridcell.val[l] = grid[position_arr[l][0]][position_arr[l][1]][position_arr[l][2]];
                    gridcell.tsdfVal[l] = tsdfGrid[position_arr[l][0]][position_arr[l][1]][position_arr[l][2]];
                    gridcell.vert[l].x = position_arr[l][0];
                    gridcell.vert[l].y = position_arr[l][1];
                    gridcell.vert[l].z = position_arr[l][2];
                }
                count+=process_cube(gridcell,ptrCloud,index);
            }
        }
    }

    std::cout << "A total of " << count << " triangles are processed." << std::endl;

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
