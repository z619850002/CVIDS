#include "../include/server_brief_extractor.h"

using namespace std;

//Constructor.
ServerBriefExtractor::ServerBriefExtractor(const string &aPatternFilePath)
{
  // The DVision::BRIEF extractor computes a random pattern by default when
  // the object is created.
  // We load the pattern that we used to build the vocabulary, to make
  // the descriptors compatible with the predefined vocabulary

  // loads the pattern
  cv::FileStorage iFileStorage(aPatternFilePath.c_str(), cv::FileStorage::READ);
  if(!iFileStorage.isOpened()) throw string("Could not open file ") + aPatternFilePath;

  vector<int> x1, y1, x2, y2;
  iFileStorage["x1"] >> x1;
  iFileStorage["x2"] >> x2;
  iFileStorage["y1"] >> y1;
  iFileStorage["y2"] >> y2;

  m_iBrief.importPairs(x1, y1, x2, y2);
}


void ServerBriefExtractor::operator() (const cv::Mat &mImage, vector<cv::KeyPoint> &gKeyPoints, vector<DVision::BRIEF::bitset> &gDescriptros) const
{
  m_iBrief.compute(mImage, gKeyPoints, gDescriptros);
}


