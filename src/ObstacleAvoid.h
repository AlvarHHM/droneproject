#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <numeric>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include "math.h"
#include "KeyPointHistory.h"

using namespace cv;
using namespace std;

class ObstacleAvoid {

public:
    static const int SEARCH_RES = 20;
    static constexpr double MINSIZE = 1.2f;
    //1 + np.arange(SEARCH_RES+1)/double(2*SEARCH_RES)
    static const std::array<double,(SEARCH_RES+1)> SCALE_RANGE;
    static constexpr double KEYPOINT_SCALE = (MINSIZE * SEARCH_RES) / 9;
    static const int LAST_DAY = 10;

    bool hasObstacle = false;
    int obstacleX;
    vector<KeyPoint> obstacleCluster;


    virtual ~ObstacleAvoid();

    ObstacleAvoid();

    void processFrame(Mat &frame);

    void init(Mat &frame);



private:
    BFMatcher *bfMatcher;
    SURF *surf_ui;

    int uniqueId(void);

    double diffKP_L2(KeyPoint kp0, KeyPoint kp1);

    void estimateKeypointExpansion(Mat const &currFrame, Mat const &lastFrame, vector<DMatch> const &matches,
                                   vector<KeyPoint> const &queryKPs, vector<KeyPoint> const &trainKPs,
                                   unordered_map<int, KeyPointHistory>  &kphist,
                                   vector<double>& scale_argmin, vector<DMatch>& expandingMatches ) ;
    vector<vector<KeyPoint> > clusterKeyPoints(vector<KeyPoint>);
    void trunc_coords(const Size &dims, const int &in_x, const int &in_y, int &out_x, int &out_y);
    void normalizeMAtrix(const Mat& inImg, Mat& outImg);

    Mat lastFrame;
    vector<KeyPoint> queryKP;
    Mat qdesc;
    unordered_map<int, KeyPointHistory> kphist;
    Mat roi;
    long t_last;


    double mean_ttc;

};

