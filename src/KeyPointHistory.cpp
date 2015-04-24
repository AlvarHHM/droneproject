#include "KeyPointHistory.h"

void KeyPointHistory::update(const KeyPoint& kp, const Mat& desc, Mat& frame, long t0, long t1, double scale) {
    if (this->timehist_t1.size() > 0 and t0 == this->timehist_t1.back()){
        this->consecutive += 1;
    }else{
        this->consecutive = 1;
    }
    this->age = -1;
    this->detects += 1;
    this->scalehist.push_back(scale);
    this->timehist_t0.push_back(t0);
    this->timehist_t1.push_back(t1);
    this->keypoint = kp;
    this->frame = frame;
    desc.copyTo(this->descriptor);

}

int KeyPointHistory::downdate() {
    this->age += 1;
    return this->age;

}

KeyPointHistory::~KeyPointHistory(){
//    cout << "KeyPointHistory delete";
}