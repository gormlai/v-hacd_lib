#define ENABLE_VHACD_IMPLEMENTATION 1
#include <VHACD.h>

#include <string>
#include <map>
#include <vector>

extern "C" {
  void printNumber(int num);
  // return handle
  int64_t convexDecomposition(const float * vertices, const uint32_t * indices, const unsigned int numVertices, const unsigned int numIndices);
  int numConvexHulls(int64_t handle);
  int numTriangles(int64_t handle, int convexHullIndex);
  int numVertices(int64_t handle, int convexHullIndex);
  void freeHandle(int64_t handle);
}


namespace {
  typedef std::vector<VHACD::IVHACD::ConvexHull> ConvexHullVector;
  std::map<VHACD::IVHACD *, ConvexHullVector> g_convexHullMap;
}

void printNumber(int num) {
  const std::string numS = std::to_string(num);
  printf("num is %s\n", numS.c_str());
}

int64_t convexDecomposition(const float * vertices, const uint32_t * indices, const unsigned int numVertices, const unsigned int numIndices) {
  VHACD::IVHACD * cDecomposer = VHACD::CreateVHACD();
  assert(cDecomposer != nullptr);

  //  double *dVertices = new double[numVertices*3];
  double dVertices[numVertices*3];
  for(unsigned int vertexIndex = 0 ; vertexIndex < (numVertices*3) ; vertexIndex++) {
    dVertices[vertexIndex] = vertices[vertexIndex];
  }
  VHACD::IVHACD::Parameters parameters;
  const bool decomposeResult = cDecomposer->Compute(dVertices, numVertices, indices, numIndices / 3, parameters);
  if(decomposeResult) {
    const int numConvexHulls = cDecomposer->GetNConvexHulls();

    for(int convexHullIndex = 0 ; convexHullIndex < numConvexHulls ; convexHullIndex++) {
      VHACD::IVHACD::ConvexHull convexHull;
      cDecomposer->GetConvexHull(convexHullIndex, convexHull);
    }

    return reinterpret_cast<int64_t>(cDecomposer);
    //   return numConvexHulls;    
  }
  else {
    printf("Failed to calculate convex hull\n");
    return 0;
  }

}

void freeHandle(int64_t handle) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return;
  }  

  cDecomposer->Release();
}

int numConvexHulls(int64_t handle) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return 0;
  }  

  const int numConvexHulls = cDecomposer->GetNConvexHulls();
  return numConvexHulls; 
  
}


int numTriangles(int64_t handle, int convexHullIndex) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return 0;
  }

  std::vector<VHACD::IVHACD::ConvexHull> & vec = it->second;
  assert(convexHullIndex < vec.size());

  VHACD::IVHACD::ConvexHull & cHull = vec[convexHullIndex];
  return (int)cHull.m_triangles.size();  
}

int numVertices(int64_t handle, int convexHullIndex) {
  VHACD::IVHACD * cDecomposer = reinterpret_cast<VHACD::IVHACD *>(handle);
  auto it = g_convexHullMap.find(cDecomposer);
  if(it == g_convexHullMap.end()) {
    return 0;
  }

  std::vector<VHACD::IVHACD::ConvexHull> & vec = it->second;
  assert(convexHullIndex < vec.size());

  VHACD::IVHACD::ConvexHull & cHull = vec[convexHullIndex];
  return (int)cHull.m_points.size();    
}
