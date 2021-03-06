#include "curvefileio.h"

#include <cstddef>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>

namespace geometry {

// HELPER FUNCTIONS because GLM does not provide them (should be in another
// place, but are acceptable in .cpp (local to this TU)
std::istream &operator>>(std::istream &in, glm::vec3 &vec) {
  in >> vec.x >> vec.y >> vec.z;
  return in;
}

std::ostream &operator<<(std::ostream &out, glm::vec3 const &vec) {
  out << vec.x << ' ' << vec.y << ' ' << vec.z;
  return out;
}

void saveCurveToFile(const Curve &curve, const std::string &filePath) {
  std::ofstream file(filePath);
  for (auto const &point : curve.points()) {
    file << point << '\n';
  }
  file.close();
}

Curve loadCurveFromFile(std::string const &filePath) {
  using std::istreambuf_iterator;
  using std::string;
  using std::stringstream;

  Points points;
  std::ifstream file(filePath);

  if (!file) {
    std::cerr << "Unable to open file " << filePath << '\n';
    return Curve{};
  }

  string line;
  size_t index;
  stringstream ss(std::ios_base::in);
  size_t lineNum = 0;

  while (getline(file, line)) {
    ++lineNum;

    // remove comments
    index = line.find_first_of("#");
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    // removes leading/tailing junk
    line.erase(0, line.find_first_not_of(" \t\r\n\v\f"));
    index = line.find_last_not_of(" \t\r\n\v\f") + 1;
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    if (line.empty()) {
      continue; // empty or commented out line
    }

    ss.str(line);
    ss.clear();

    glm::vec3 v;
    ss >> v;

    if (!ss) {
      std::cerr << "Error read file: " + line +
                       " (line: " + std::to_string(lineNum) + ")";
    } else {
      points.push_back(v);
    }
  }
  file.close();

  return {points};
}

geometry::Curve loadCurveFrom_OBJ_File(std::string const &filePath) {
  using std::istreambuf_iterator;
  using std::string;
  using std::stringstream;

  Points points;
  std::ifstream objFile(filePath);

  if (!objFile) {
    std::cerr << "Unable to open file " << filePath << '\n';
    return Curve{};
  }

  string line;
  size_t index;
  stringstream ss(std::ios_base::in);
  size_t lineNum = 0;

  while (getline(objFile, line)) {
    ++lineNum;

    // remove comments
    index = line.find_first_of("#");
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    // removes leading/tailing junk
    line.erase(0, line.find_first_not_of(" \t\r\n\v\f"));
    index = line.find_last_not_of(" \t\r\n\v\f") + 1;
    if (index != string::npos) {
      line.erase(index, string::npos);
    }

    if (line.empty()) {
      continue; // empty or commented out line
    }

    if (line.front() != 'v')
      continue;

    ss.str(line);
    ss.clear();

    char isV; // stupid but meh
    ss >> isV;

    glm::vec3 v;
    ss >> v;

    if (!ss) {
      std::cerr << "Error read file: " + line +
                       " (line: " + std::to_string(lineNum) + ")";
    } else {
      points.push_back(v);
    }
  }
  objFile.close();

  return {points};
}

} // namespace geometry
