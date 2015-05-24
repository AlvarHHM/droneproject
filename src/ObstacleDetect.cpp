#include "ObstacleDetect.h"


const std::array<double, ObstacleDetect::SEARCH_RES+1> ObstacleDetect::SCALE_RANGE = []{
    std::array<double, ObstacleDetect::SEARCH_RES+1> arr;
    for(int i = 0; i < arr.size(); ++i){
        arr[i] = 1 + (i / (2.0 * ObstacleDetect::SEARCH_RES));
    }
    return arr;
}();

ObstacleDetect::~ObstacleDetect() {
    delete this->bfMatcher;
    delete this->surf_ui;
}

ObstacleDetect::ObstacleDetect() {
    this->bfMatcher = new BFMatcher();
    this->surf_ui = new SURF(2000, 4, 2, true, true);

    Mat roi = Mat::zeros(360, 640, CV_8U);
    int scrapY = 360 / 4;
    int scrapX = 640 / 4;
    roi(Rect(scrapX, scrapY, roi.cols - 2 * scrapX, roi.rows - 2 * scrapY)) = true;
    this->roi = roi;

}

void ObstacleDetect::processFrame(Mat& frame) {
    this->hasObstacle = false;

    timeval time;
    gettimeofday(&time, NULL);
    long t_curr = time.tv_sec * 1000 * 1000 + time.tv_usec;
    Mat& currFrame = frame;
//    Mat dispim;
//    frame.copyTo(dispim);

    vector<KeyPoint>& queryKP = this->queryKP;
    vector<KeyPoint> trainKP;
    Mat& qdesc = this->qdesc;
    Mat tdesc;
    auto& kphist = this->kphist;
    Mat& roi = this->roi;
    for(KeyPoint& kp : this->queryKP){
        if (kp.class_id == 1 or kp.class_id == -1) {
            kp.class_id = this->uniqueId();
        }
    }

    this->surf_ui->operator()(currFrame, roi, trainKP, tdesc);

    vector<vector<DMatch> > pre_matches;
    if (!(qdesc.total() == 0 or tdesc.total() == 0)) {
        this->bfMatcher->knnMatch(qdesc, tdesc, pre_matches, 2);
    }


    vector<double> matchdist;
    vector<DMatch> matches;

    for (vector<DMatch>& m : pre_matches){
        if ((m.size() ==2 and m[0].distance >= 0.8 * m[1].distance) or m[0].distance >= 0.25){
            continue;
        }
        matches.push_back(m[0]);
        KeyPoint& qkp = queryKP[m[0].queryIdx];
        KeyPoint& tkp = trainKP[m[0].trainIdx];
        tkp.class_id = qkp.class_id;
        matchdist.push_back(diffKP_L2(qkp, tkp));
    }

    // discard non-confidence matches
    if (matchdist.size() != 0){
        vector<double> tmp_matchdist(matchdist.begin(), matchdist.end() - matchdist.size() / 4);
        double mean = std::accumulate(tmp_matchdist.begin(), tmp_matchdist.end(), 0.0) / tmp_matchdist.size();
        std::vector<double> diff(tmp_matchdist.size());
        std::transform(tmp_matchdist.begin(), tmp_matchdist.end(), diff.begin(),
                       std::bind2nd(std::minus<double>(), mean));
        double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);
        double stdev = std::sqrt(sq_sum / tmp_matchdist.size());
        double threshdist = 2 * stdev + mean;
        auto itm = matches.begin();
        auto itd = matchdist.begin();
        for (;(itm != matches.end() && itd != matchdist.end());){
            if (*itd > threshdist){
                itm = matches.erase(itm);
                itd = matchdist.erase(itd);
            }else{
                ++itm;
                ++itd;
            }
        }
    }



    // Draw matches
//    drawKeypoints(dispim, queryKP, dispim, Scalar(0, 255, 0));
//    drawKeypoints(dispim, trainKP, dispim, Scalar(255, 0, 0));


    // discard keypoint that is getting smaller or equal
    for(auto i = matches.begin(); i != matches.end();){
        if(trainKP[i->trainIdx].size <= queryKP[i->queryIdx].size){
            i = matches.erase(i);
        }else{
            ++i;
        }
    }


    vector<DMatch> scaledMatches;
    vector<double> kpscales;
    this->estimateKeypointExpansion(currFrame, lastFrame, matches, queryKP,trainKP,
                              kphist, kpscales, scaledMatches);


    for(int i = 0; i < scaledMatches.size(); ++i){
        DMatch m = scaledMatches[i];
        double scale = kpscales[i];
        int clsid = trainKP[m.trainIdx].class_id;
        double t_A;
        if (kphist.count(clsid) == 0) {
            kphist[clsid] = KeyPointHistory();
            t_A = t_last;
        }else {
            t_A = kphist[clsid].timehist_t1.back();
        }
        kphist[clsid].update(trainKP[m.trainIdx], tdesc.row(m.trainIdx), currFrame, t_A, t_curr, scale);
    }

    unordered_set<int> detected;
    for (KeyPoint& kp : trainKP) {
        detected.insert(kp.class_id);
    }

    for (auto it =  kphist.begin(); it != kphist.end();){
        // age key point
        it->second.downdate();
        // delete aging key point
        if(it->second.age > 10){
            it = kphist.erase(it);
        }else{
            // push back old key point into the training set
            if (detected.count(it->first) == 0 && it->second.age > 0) {
                trainKP.push_back(it->second.keypoint);
                tdesc.push_back(it->second.descriptor);
            }
            ++it;
        }
    }


    vector<KeyPoint> expandingKPs;
    for (auto&& kp : trainKP){
        if (kphist.count(kp.class_id) != 0 && kphist[kp.class_id].age == 0){
            expandingKPs.push_back(kp);
        }
    }

    const vector<vector<KeyPoint> >& clusters = clusterKeyPoints(expandingKPs);
    const vector<KeyPoint>& cluster  = clusters[0];
    if (clusters.size() != 0) {
        vector<double> ttc_cluster;
        int sum = 0;
        for (const KeyPoint& kp : cluster){
            sum += kp.pt.x;
            double scale = kphist[kp.class_id].scalehist.back();
            double tstep = (kphist[kp.class_id].timehist_t1.back()
                            - kphist[kp.class_id].timehist_t0.back());
            ttc_cluster.push_back(tstep / scale);
            cout << scale << " " << tstep << endl;
        }
        double mean_ttc = std::accumulate(ttc_cluster.begin(), ttc_cluster.end(), 0.0)
                          / ttc_cluster.size();
        this->hasObstacle = true;
        this->mean_ttc = mean_ttc / 1000000;
        this->obstacleX = static_cast<int>(sum / cluster.size());
        this->obstacleCluster = cluster;

        ROS_INFO("mean ttc: %.2f", this->mean_ttc);
    }

//    imshow("DEBUG_OBSTACLE", dispim);

    this->lastFrame = currFrame;
    this->t_last = t_curr;
    this->queryKP = trainKP;
    this->qdesc = tdesc;
}

//void ObstacleDetect::estimateKeypointExpansion(Mat const &currFrame, Mat const &lastFrame, vector<DMatch> const &matches,
//                                              vector<KeyPoint> const &queryKPs, vector<KeyPoint> const &trainKPs,
//                                              unordered_map<int, KeyPointHistory>  &kphist,
//                                              vector<double>& scale_argmin, vector<DMatch>& expandingMatches ) {
//    const Mat& trainImg = currFrame;
//    const Mat& prevImg = lastFrame;
//
//    for(const DMatch& m : matches){
//        const KeyPoint& qkp = queryKPs[m.queryIdx];
//        const KeyPoint& tkp = trainKPs[m.trainIdx];
//
//        const Mat& queryImg = (kphist.count(qkp.class_id) == 1)?kphist[qkp.class_id].frame:prevImg;
////        const Mat& queryImg = prevImg;
//
//        int x_qkp = static_cast<int>(qkp.pt.x);
//        int y_qkp = static_cast<int>(qkp.pt.y);
//        int r_qkp = static_cast<int>(qkp.size * 1.2 / 9 * 20 / 2);
//        int x0, y0, x1, y1;
//        const Size& qsize = queryImg.size();
//        trunc_coords(qsize, (x_qkp - r_qkp), (y_qkp - r_qkp), x0, y0);
//        trunc_coords(qsize, (x_qkp + r_qkp), (y_qkp + r_qkp), x1, y1);
//        Mat querypatch;
//        queryImg(Range(min(y0, y1), max(y0, y1)), Range(min(x0, x1), max(x0, x1))).copyTo(querypatch);
//        if (querypatch.total() == 0) continue;
//        normalizeMAtrix(querypatch, querypatch);
//
//
//        std::array<double, SCALE_RANGE.size()> tm_scale;
//        tm_scale.fill(NAN);
//        for(int i = 0; i < SCALE_RANGE.size(); ++i){
//            double scale = SCALE_RANGE[i];
//
//            int x_tkp = static_cast<int>(tkp.pt.x);
//            int y_tkp = static_cast<int>(tkp.pt.y);
//            int r_tkp = static_cast<int>(tkp.size * 1.2 / 9 * 20 * scale / 2);
//            int x0, y0, x1, y1;
//            Size tsize = trainImg.size();
//            trunc_coords(tsize, (x_tkp - r_tkp), (y_tkp - r_tkp), x0, y0);
//            trunc_coords(tsize, (x_tkp + r_tkp), (y_tkp + r_tkp), x1, y1);
//            Mat scaledtrain;
//            trainImg(Range(min(y0, y1), max(y0, y1)), Range(min(x0, x1), max(x0, x1))).copyTo(scaledtrain);
//            if (scaledtrain.total() == 0) continue;
//            normalizeMAtrix(scaledtrain, scaledtrain);
//
//
//            Mat scaledquery;
//            resize(querypatch, scaledquery, scaledtrain.size(), scale, scale);
////            normalizeMAtrix(scaledquery, scaledquery);
//
//            Mat tmp;
//            pow((scaledquery - scaledtrain), 2, tmp);
//            tmp = tmp / pow(scale, 2);
//            tm_scale[i] = sum(tmp).val[0];
//        }
//
//        if (all_of(tm_scale.begin(), tm_scale.end(), [](double i){return std::isnan(i);})) continue;
//        int minIndex = 0;
//        double minValue = tm_scale[0];
//        for(int i = 0; i < tm_scale.size(); i++){
//            if(tm_scale[i] < minValue){
//                minIndex = i;
//                minValue = tm_scale[i];
//            }
//        }
//        if ((SCALE_RANGE[minIndex] > MINSIZE) and (minValue < 0.8 * tm_scale[0])){
//            scale_argmin.push_back(SCALE_RANGE[minIndex]);
//            expandingMatches.push_back(m);
//        }
//    }
//}


void ObstacleDetect::estimateKeypointExpansion(Mat const &currFrame, Mat const &lastFrame, vector<DMatch> const &matches,
                                              vector<KeyPoint> const &queryKPs, vector<KeyPoint> const &trainKPs,
                                              unordered_map<int, KeyPointHistory>  &kphist,
                                              vector<double>& scale_argmin, vector<DMatch>& expandingMatches ) {

    const Mat& trainImg = currFrame;
    const Mat& prevImg = lastFrame;

    for(const DMatch& m : matches){
        const KeyPoint& qkp = queryKPs[m.queryIdx];
        const KeyPoint& tkp = trainKPs[m.trainIdx];

        const Mat& queryImg = (kphist.count(qkp.class_id) == 1)?kphist[qkp.class_id].frame:prevImg;
//        const Mat& queryImg = prevImg;

        int x_qkp = static_cast<int>(qkp.pt.x);
        int y_qkp = static_cast<int>(qkp.pt.y);
        int qkpr = static_cast<int>(qkp.size * KEYPOINT_SCALE / 2);
        int x0, y0, x1, y1;
        trunc_coords(queryImg.size(), x_qkp - qkpr, y_qkp - qkpr, x0, y0);
        trunc_coords(queryImg.size(), x_qkp + qkpr, y_qkp + qkpr, x1, y1);
        Mat querypatch;
        queryImg(cv::Range(min(y0, y1), max(y0, y1)), cv::Range(min(x0, x1), max(x0, x1))).copyTo(querypatch);
        if (querypatch.total() == 0) continue;
        normalizeMAtrix(querypatch, querypatch);

        int x_tkp = static_cast<int>(tkp.pt.x);
        int y_tkp = static_cast<int>(tkp.pt.y);
        int tkpr = static_cast<int>(tkp.size * KEYPOINT_SCALE * SCALE_RANGE.back() / 2);
        trunc_coords(trainImg.size(), x_tkp - tkpr, y_tkp - tkpr, x0, y0);
        trunc_coords(trainImg.size(), x_tkp + tkpr, y_tkp + tkpr, x1, y1);
        Mat trainpatch;
        trainImg(cv::Range(min(y0, y1), max(y0, y1)), cv::Range(min(x0, x1), max(x0, x1))).copyTo(trainpatch);
        normalizeMAtrix(trainpatch, trainpatch);


        std::array<double, SCALE_RANGE.size()> res;
        std::fill(res.begin(), res.end(), NAN);
        x_tkp = x_tkp-x0;
        y_tkp = y_tkp-y0;

        for (int i = 0; i < SCALE_RANGE.size(); ++i){
            double scale = SCALE_RANGE[i];
            int r = static_cast<int>(qkp.size * KEYPOINT_SCALE * scale / 2);
            trunc_coords(trainpatch.size(), x_tkp - r, y_tkp - r, x0, y0);
            trunc_coords(trainpatch.size(), x_tkp + r, y_tkp + r, x1, y1);
            Mat scaledtrain = trainpatch(cv::Range(min(y0, y1), max(y0, y1)), cv::Range(min(x0, x1), max(x0, x1)));
            if (scaledtrain.total() == 0) continue;

            Mat scaledquery;
            resize(querypatch, scaledquery, scaledtrain.size(), scale, scale, cv::INTER_LINEAR);

            Mat tmp;
            pow((scaledquery - scaledtrain), 2, tmp);
            res[i] = (cv::sum(tmp)[0] / pow(scale, 2));
        }


        if (all_of(res.begin(), res.end(), [](double i){return std::isnan(i);})) continue;
        int minIndex = 0;
        double minValue = res[0];
        for(int i = 0; i < res.size(); i++){
            if(res[i] < minValue){
                minIndex = i;
                minValue = res[i];
            }
        }
        if ((SCALE_RANGE[minIndex] > MINSIZE) and (minValue < 0.8 * res[0])){
            scale_argmin.push_back(SCALE_RANGE[minIndex]);
            expandingMatches.push_back(m);
        }
    }
}



vector<vector<KeyPoint> > ObstacleDetect::clusterKeyPoints(vector<KeyPoint> keypoints) {
    vector<vector<KeyPoint> > clusters;
    if  (keypoints.size() < 2) return clusters;
    std::sort(keypoints.begin(), keypoints.end(),
        [](KeyPoint kp1, KeyPoint kp2){
            return kp1.pt.x == kp2.pt.x? kp1.pt.x < kp2.pt.x: kp1.pt.y < kp2.pt.y;
        });
    while(keypoints.size() != 0){
        vector<KeyPoint> cluster;
        cluster.push_back(keypoints[0]);
        keypoints.erase(keypoints.begin());
        KeyPoint kp = cluster[0];
        int i = 0;
        while (i < keypoints.size()){
            if ((kp.size / 2 + keypoints[i].size / 2) > diffKP_L2(kp, keypoints[i])){
                cluster.push_back(keypoints[i]);
                keypoints.erase(keypoints.begin() + i);
            }else{
                ++i;
            }
        }
        if (cluster.size() >= 3){
            clusters.push_back(cluster);
        }

    }

    std::sort(clusters.begin(), clusters.end(),
         [](vector<KeyPoint> clut1, vector<KeyPoint> clut2){return clut1.size() > clut2.size();});

    return clusters;
}

void ObstacleDetect::trunc_coords(const Size& dims, const int& in_x, const int& in_y, int& out_x, int& out_y) {
    out_x = (in_x >= 0 and in_x <= dims.width)? in_x: (in_x < 0)? 0: dims.width;
    out_y = (in_y >= 0 and in_y <= dims.height)? in_y: (in_y < 0)? 0: dims.height;
}


void ObstacleDetect::init(Mat& frame) {
    this->lastFrame = frame;
    time(&this->t_last);
    Mat roi = Mat::zeros(frame.rows, frame.cols, CV_8UC1);
    int scrapY = frame.rows / 4;
    int scrapX = frame.cols / 4;
    roi(Rect(scrapX, scrapY, roi.cols - 2 * scrapX, roi.rows - 2 * scrapY)) = true;
    this->roi = roi;
    this->qdesc = Mat();

    this->surf_ui->operator()(frame, roi, this->queryKP, this->qdesc);
    for(auto& kp : this->queryKP){
        kp.class_id = this->uniqueId();
    }
}


int ObstacleDetect::uniqueId(void) {
    static volatile int i = 2;
    return __sync_add_and_fetch(&i, 1);
}

void ObstacleDetect::normalizeMAtrix(const Mat& inImg, Mat& outImg){
    Mat tmp;
    inImg.convertTo(tmp, CV_32F);
    Scalar mean_scala;
    Scalar stddev_scala;
    meanStdDev(tmp, mean_scala, stddev_scala);
    double mean = static_cast<double>(mean_scala.val[0]);
    double std = static_cast<double>(stddev_scala.val[0]);

    outImg = (tmp - mean) / std;
}

double ObstacleDetect::diffKP_L2(KeyPoint kp0, KeyPoint kp1){
    return sqrt(pow((kp0.pt.x - kp1.pt.x), 2) + pow((kp0.pt.y - kp1.pt.y), 2));
}


void ObstacleDetect::reset() {
    this->hasObstacle = false;
    this->obstacleX = -1;
    this->obstacleCluster.clear();
    this->kphist.clear();
    this->queryKP.clear();
    this->qdesc = Mat();
    this->mean_ttc = -1;
}
