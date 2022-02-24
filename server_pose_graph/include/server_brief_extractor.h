#pragma once

#include "../ThirdParty/DBoW/DBoW2.h"
#include "../ThirdParty/DVision/DVision.h"

#include <iostream>
#include <vector>
#include <set>
#include <string>
using namespace std;

class ServerBriefExtractor
{
public:
  virtual void operator()(const cv::Mat &mImage, vector<cv::KeyPoint> &gKeyPoints, vector<DVision::BRIEF::bitset> &gDescriptros) const;
  ServerBriefExtractor(const std::string &aPatternFilePath);

  DVision::BRIEF m_iBrief;
};