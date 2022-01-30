#ifndef BINARY_UTILITY_DOT_H
#define BINARY_UTILITY_DOT_H

#include <vector>
#include <cmath>
#include <stdexcept>

#include "numberUtility.h"

using namespace std;

namespace binUtil {

  vector<int> decToBin1Byte(int dec) {
    vector<int> v;
    for (int i = 7; i >= 0; i--) {
      if (dec - pow(2, i) >= 0) {
        v.push_back(1);
        dec -= pow(2, i);
      } else {
        v.push_back(0);
      }
    }
    return v;
  }

  int binToDec1Byte(const vector<int>& bin) {
    int d = 0;
    for (int i = 0; i < bin.size(); i++) {
      if (bin.at(i) == 1) {
        d += pow(2, 7 - i);
      }
    }
    return d;
  }

  vector<int> decToBinNoLeading0(int dec) {
    if (dec == 0) {
      return {0};
    }
    vector<int> v;
    bool hasLeading1 = false;
    for (int i = numUtil::log(dec, 2); i >= 0; i--) {
      if (dec - pow(2, i) >= 0) {
        v.push_back(1);
        dec -= pow(2, i);
        hasLeading1 = true;
      } else if (hasLeading1) {
        v.push_back(0);
      }
    }
    return v;
  }

  int minRequiredDigits(int dec) {
    return (dec == 0) ? 1 : (numUtil::log(dec, 2) + 1);
  }

  vector<int> decToBinNdigits(int dec, int n) {
    int minRequired = minRequiredDigits(dec);
    if (n < minRequired) {
      throw invalid_argument("minimum required digits = " + to_string(minRequired));
    }
    vector<int> v;
    for (int i = n - 1; i >= 0; i--) {
      if (dec - pow(2, i) >= 0) {
        v.push_back(1);
        dec -= pow(2, i);
      } else {
        v.push_back(0);
      }
    }
    return v;
  }

}

#endif