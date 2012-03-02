#ifndef BODY_POSE_H
#define BODY_POSE_H

class BodyPose{
  public:
    BodyPose(){};
    BodyPose(double x_, double y_, double z_, double th_){
      x = x_;
      y = y_;
      z = z_;
      theta = th_;
    };
    double x;
    double y;
    double z;
    double theta;
};

class BodyCell{
  public:
    BodyCell(){};
    BodyCell(int x_, int y_, int z_, int th_){
      x = x_;
      y = y_;
      z = z_;
      theta = th_;
    };
    int x;
    int y;
    int z;
    int theta;
};

#endif 
