#define ENABLE_VHACD_IMPLEMENTATION 1
#include <VHACD.h>

#include <string>

extern "C" {
  void printNumber(int num);
  int convexDecomposition(const float * vertices, const uint32_t * indices, const unsigned int numVertices, const unsigned int numIndices);
}

void printNumber(int num) {
  const std::string numS = std::to_string(num);
  printf("num is %s\n", numS.c_str());
}

int convexDecomposition(const float * vertices, const uint32_t * indices, const unsigned int numVertices, const unsigned int numIndices) {
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

    return numConvexHulls;    
  }
  else {
    printf("Failed to calculate convex hull\n");
    return 0;
  }

}
