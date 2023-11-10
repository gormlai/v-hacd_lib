#define ENABLE_VHACD_IMPLEMENTATION 1
#include <VHACD.h>

#include <string>
#include <map>
#include <vector>

extern "C" {
  void printNumber(int num);
  // return handle
  uint64_t convexDecomposition(const float * vertices, const uint32_t * indices, const unsigned int numVertices, const unsigned int numIndices);
  void freeHandle(uint64_t handle);
  void getTriangles(uint64_t handle, uint convexHullIndex, uint32_t * indices);
  void getVertices(uint64_t handle, uint convexHullIndex, double * vertices);
  uint numConvexHulls(uint64_t handle);
  uint numTriangles(uint64_t handle, uint convexHullIndex);
  uint numVertices(uint64_t handle, uint convexHullIndex);
  float getVolume(uint64_t handle, uint convexHullIndex);
}

struct ConvexHullDescriptor {
  VHACD::IVHACD::ConvexHull convexHull;
  VHACD::BoundsAABB bounds;
  std::vector<int> neighbours;
};

namespace {
  typedef std::vector<ConvexHullDescriptor> ConvexHullDescVector;
  std::map<VHACD::IVHACD *, ConvexHullDescVector> g_convexHullMap;
}

void printNumber(int num) {
  const std::string numS = std::to_string(num);
  printf("num is %s\n", numS.c_str());
}

uint64_t convexDecomposition(const float * vertices, const uint32_t * indices, const unsigned int numVertices, const unsigned int numIndices) {
  VHACD::IVHACD * cDecomposer = VHACD::CreateVHACD();
  assert(cDecomposer != nullptr);

  //  double *dVertices = new double[numVertices*3];
  double dVertices[numVertices*3];
  for(unsigned int vertexIndex = 0 ; vertexIndex < (numVertices*3) ; vertexIndex++) {
    dVertices[vertexIndex] = vertices[vertexIndex];
  }
  
  VHACD::IVHACD::Parameters parameters = {};
  parameters.m_maxConvexHulls = 12;
  parameters.m_resolution = 100 * 100 * 100;
  parameters.m_minimumVolumePercentErrorAllowed = 1;
  parameters.m_maxRecursionDepth = 10;

  
  const bool decomposeResult = cDecomposer->Compute(dVertices, numVertices, indices, numIndices / 3, parameters);
  if(decomposeResult) {
    const int numConvexHulls = cDecomposer->GetNConvexHulls();

    std::vector<ConvexHullDescriptor> hulls(numConvexHulls);
    for(int convexHullIndex = 0 ; convexHullIndex < numConvexHulls ; convexHullIndex++) {
      cDecomposer->GetConvexHull(convexHullIndex, hulls[convexHullIndex].convexHull);
    }

    g_convexHullMap[cDecomposer] = hulls;

    return reinterpret_cast<uint64_t>(cDecomposer);
  }
  else {
    printf("Failed to calculate convex hull\n");
    return 0;
  }

}

void freeHandle(uint64_t handle) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return;
  }  

  g_convexHullMap.erase(it);
  cDecomposer->Release();
}

float getVolume(uint64_t handle, uint convexHullIndex) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return 0;
  }

  std::vector<ConvexHullDescriptor> & vec = it->second;
  assert(convexHullIndex < vec.size());

  VHACD::IVHACD::ConvexHull & cHull = vec[convexHullIndex].convexHull;
  return (float)cHull.m_volume;
}


void getTriangles(uint64_t handle, uint convexHullIndex, uint32_t * indices) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return;
  }

  std::vector<ConvexHullDescriptor> & vec = it->second;
  assert(convexHullIndex < vec.size());

  VHACD::IVHACD::ConvexHull & cHull = vec[convexHullIndex].convexHull;
  memcpy(indices, &cHull.m_triangles[0], sizeof(uint32_t)*cHull.m_triangles.size()*3);
}

void getVertices(uint64_t handle, uint convexHullIndex, double * vertices) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return;
  }

  std::vector<ConvexHullDescriptor> & vec = it->second;
  assert(convexHullIndex < vec.size());

  VHACD::IVHACD::ConvexHull & cHull = vec[convexHullIndex].convexHull;
  memcpy(vertices, &cHull.m_points[0], sizeof(double)*cHull.m_points.size() * 3);
}


uint numConvexHulls(uint64_t handle) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return 0;
  }  

  const uint numConvexHulls = cDecomposer->GetNConvexHulls();
  return numConvexHulls; 
  
}


uint numTriangles(uint64_t handle, uint convexHullIndex) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return 0;
  }

  std::vector<ConvexHullDescriptor> & vec = it->second;
  assert(convexHullIndex < vec.size());

  VHACD::IVHACD::ConvexHull & cHull = vec[convexHullIndex].convexHull;
  return (uint)cHull.m_triangles.size();  
}

uint numVertices(uint64_t handle, uint convexHullIndex) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return 0;
  }

  std::vector<ConvexHullDescriptor> & vec = it->second;
  assert(convexHullIndex < vec.size());

  VHACD::IVHACD::ConvexHull & cHull = vec[convexHullIndex].convexHull;
  return (uint)cHull.m_points.size();    
}

