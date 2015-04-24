#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <numeric>
#include <array>
#include <vector>
#include "math.h"

using namespace cv;
using namespace std;

class KeyPointHistory {
public:
    int age = -1;
    int detects = 0;
    vector<double> scalehist;
    vector<long> timehist_t0;
    vector<long> timehist_t1;
    KeyPoint keypoint;
    Mat descriptor;
    int consecutive = 0;
    Mat frame;

    virtual ~KeyPointHistory();

    void update(const KeyPoint& kp, const Mat& desc, Mat& frame, long t0, long t1, double scale);
    int downdate();
private:

};


