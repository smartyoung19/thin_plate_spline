#include <iostream>
#include <vector>
#include "ThinPlateSpline.hpp"

using namespace std;

/*
*  pt_vect: (2, N), N point with (x, y)
*  return: vecotr<Eigen::Vector3d>, 3d point is (x, y, 0). All z=0.
*/
PointList Vector2PointList(vector<vector<int>> pt_vect)
{
    PointList res;
    for (int i = 0; i < pt_vect[0].size(); ++i)
    {
        res.push_back(Eigen::Vector3d(pt_vect[0][i], pt_vect[1][i], 0));
    }
    return res;
}

int main()
{
    // src control points
    vector<vector<int>> pt_src;
    pt_src.push_back(vector<int>{ 10, 10, 50, 53, 100, 100 });   // x 10, 10, 50, 100, 100
    pt_src.push_back(vector<int>{ 10, 100, 50, 55, 10, 100 });  // y 10, 100, 50, 10, 100


    // matched dst control points
    vector<vector<int>> pt_dst;
    pt_dst.push_back(vector<int>{ 5, 15, 61, 50, 110, 101 });     // x 5, 15, 61, 50, 110
    pt_dst.push_back(vector<int>{ 7, 90, 45, 48, 2, 104 });  // y 7, 90, 45, 48, 2, 104


    PointList src_list = Vector2PointList(pt_src);
    PointList dst_list = Vector2PointList(pt_dst);

    ThinPlateSpline tps(src_list, dst_list);
    tps.solve();

    // test result
    cv::Mat flow = tps.getWarpFlow(120, 120);
    cv::imwrite("flow.tif", flow);

    return 0;
}