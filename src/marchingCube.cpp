#include "marchingCube.h"

// constructor
MarchingCube::MarchingCube(double density, double pmass, Real nserach_radius, const vector<array<Real, 3>> &particles,
                           const Vector3D &unitGrid, const Vector3D &minBox, const Vector3D &maxBox) {
    init(density, pmass, nserach_radius, particles, unitGrid, minBox, maxBox);
}

// destructor
MarchingCube::~MarchingCube() {
    destroy();
}

// initialize private members
void MarchingCube::init(double density, double pmass, Real nserach_radius, const vector<array<Real, 3>> &particles,
                   const Vector3D &unitGrid, const Vector3D &minBox, const Vector3D &maxBox) {
    // init variables
    _density = density;
    _pmass = pmass;
    _minBox = minBox;
    _maxBox = maxBox;
    _unitGrid = unitGrid;
    _totTriangles = 0;

    // get number of grids on x, y, z coordinates (use ceiling value)
    _numGrids[0] = ceil((maxBox.x - minBox.x) / unitGrid.x);
    _numGrids[1] = ceil((maxBox.y - minBox.y) / unitGrid.y);
    _numGrids[2] = ceil((maxBox.z - minBox.z) / unitGrid.z);

    // build arrays for nsearch and triangles
    _particles = particles;
    _neighbors = new NeighborhoodSearch(nserach_radius, true);
    _neighbors->add_point_set(particles.front().data(), particles.size(), true, true);
    _neighbors->find_neighbors();
    _triangles = new vector<MarchingTriangle>();
}

// delete allocated memories
void MarchingCube::destroy() {
    delete _neighbors;
    delete _triangles;
}

// reset private members
void MarchingCube::reset(double density, double pmass, Real nserach_radius, const vector<array<Real, 3>> &particles,
           const Vector3D &unitGrid, const Vector3D &minBox, const Vector3D &maxBox) {
    destroy();
    init(density, pmass, nserach_radius, particles, unitGrid, minBox, maxBox);
}

// get triangles using marching cube algorithm
void MarchingCube::calculateTriangles(double coefficient, double isolevel, bool force) {
    for (int k = 0; k < _numGrids[2] - 1; ++k) {
        for (int j = 0; j < _numGrids[1] - 1; ++j) {
            for (int i = 0; i < _numGrids[0] - 1; ++i) {
                Vector3D index(i, j, k);
                MarchingGrid grid;
                getMarchingGrid(grid, coefficient, index, force);
                _totTriangles += Polygonise(grid, isolevel);
            }
        }
    }
}

// set actual positions and corresponding isovalues for each vertex of the unit cube
void MarchingCube::getMarchingGrid(MarchingGrid &grid, double coefficient, Vector3D &index, bool force) {
    for (int k = 0; k < 2; ++k) {
        for (int j = 0; j < 2; ++j) {
            for (int i = 0; i < 2; ++i) {
                Vector3D pos = Vector3D((index.x + i) * _unitGrid.x + _minBox.x, (index.y + j) * _unitGrid.y + _minBox.y, (index.z + k) * _unitGrid.z + _minBox.z);
                grid.p[4 * k + 2 * j + i] = pos;
                grid.val[4 * k + 2 * j + i] = getIsoValue(pos, coefficient, force);
            }
        }
    }
}

// get isovalue for each vertex
double MarchingCube::getIsoValue(Vector3D &pos, double coefficient, bool force) {
    Real rpos[3] = {pos.x, pos.y, pos.z};
    double isovalue = 0.0;
    double h = coefficient * 1.0 / pow(_density / _pmass, 1.0 / 3.0);
    if (force) h = coefficient;
    double const_part = 315.0 / (64.0 * PI * pow(h, 9.0));

    vector<vector<unsigned int>> neighbors_index;
    _neighbors->find_neighbors(rpos, neighbors_index);
    for (unsigned int i : neighbors_index[0] ) {
        Real distance[3] = {rpos[0] - _particles[i][0], rpos[1] - _particles[i][1], rpos[2] - _particles[i][2]};
        double r2 = pow(distance[0], 2.0) + pow(distance[1], 2.0) + pow(distance[2], 2.0);
        double r = sqrt(r2);
        double w = 0;
        if ( r >= 0.0 && r <= h ) {
            w = const_part * pow(pow(h, 2.0) - r2, 3.0);
        }
        isovalue += _pmass * w;
    }
    return isovalue;
}

/* Given a grid cell and an isolevel, calculate the triangular facets required to represent the isosurface through the cell.
 * Return the number of triangular facets, the array "triangles" will be loaded up with the vertices at most 5 triangular facets.
 * 0 will be returned if the grid cell is either totally above of totally below the isolevel. */
int MarchingCube::Polygonise(MarchingGrid &grid, double isolevel) {
    int ntriang = 0;
    int cubeindex = 0;
    Vector3D vertlist[12];

    /* Determine the index into the edge table which tells us which vertices are inside of the surface */
    if (grid.val[0] < isolevel) cubeindex |= 1;
    if (grid.val[1] < isolevel) cubeindex |= 2;
    if (grid.val[2] < isolevel) cubeindex |= 4;
    if (grid.val[3] < isolevel) cubeindex |= 8;
    if (grid.val[4] < isolevel) cubeindex |= 16;
    if (grid.val[5] < isolevel) cubeindex |= 32;
    if (grid.val[6] < isolevel) cubeindex |= 64;
    if (grid.val[7] < isolevel) cubeindex |= 128;

    /* Cube is entirely in/out of the surface */
    if (_edgeTable[cubeindex] == 0)
        return(0);
    /* Find the vertices where the surface intersects the cube */
    if (_edgeTable[cubeindex] & 1)
        vertlist[0] = VertexInterp(isolevel,grid.p[0],grid.p[1],grid.val[0],grid.val[1]);
    if (_edgeTable[cubeindex] & 2)
        vertlist[1] = VertexInterp(isolevel,grid.p[1],grid.p[2],grid.val[1],grid.val[2]);
    if (_edgeTable[cubeindex] & 4)
        vertlist[2] = VertexInterp(isolevel,grid.p[2],grid.p[3],grid.val[2],grid.val[3]);
    if (_edgeTable[cubeindex] & 8)
        vertlist[3] = VertexInterp(isolevel,grid.p[3],grid.p[0],grid.val[3],grid.val[0]);
    if (_edgeTable[cubeindex] & 16)
        vertlist[4] = VertexInterp(isolevel,grid.p[4],grid.p[5],grid.val[4],grid.val[5]);
    if (_edgeTable[cubeindex] & 32)
        vertlist[5] = VertexInterp(isolevel,grid.p[5],grid.p[6],grid.val[5],grid.val[6]);
    if (_edgeTable[cubeindex] & 64)
        vertlist[6] = VertexInterp(isolevel,grid.p[6],grid.p[7],grid.val[6],grid.val[7]);
    if (_edgeTable[cubeindex] & 128)
        vertlist[7] = VertexInterp(isolevel,grid.p[7],grid.p[4],grid.val[7],grid.val[4]);
    if (_edgeTable[cubeindex] & 256)
        vertlist[8] = VertexInterp(isolevel,grid.p[0],grid.p[4],grid.val[0],grid.val[4]);
    if (_edgeTable[cubeindex] & 512)
        vertlist[9] = VertexInterp(isolevel,grid.p[1],grid.p[5],grid.val[1],grid.val[5]);
    if (_edgeTable[cubeindex] & 1024)
        vertlist[10] = VertexInterp(isolevel,grid.p[2],grid.p[6],grid.val[2],grid.val[6]);
    if (_edgeTable[cubeindex] & 2048)
        vertlist[11] = VertexInterp(isolevel,grid.p[3],grid.p[7],grid.val[3],grid.val[7]);

    /* Create the triangle */
    for (int i = 0; _triTable[cubeindex][i] != -1; i += 3) {
        MarchingTriangle tri;
        tri.p[0] = vertlist[_triTable[cubeindex][i]];
        tri.p[1] = vertlist[_triTable[cubeindex][i+1]];
        tri.p[2] = vertlist[_triTable[cubeindex][i+2]];
        _triangles->push_back(tri);
        ntriang++;
    }
    return(ntriang);
}

/* Linearly interpolate the position where an isosurface cuts an edge between two vertices, each with their own scalar value */
Vector3D MarchingCube::VertexInterp(double isolevel, Vector3D a, Vector3D b, double val_a, double val_b) {
    double mu;
    Vector3D p;

    if (abs(isolevel - val_a) < 0.00001)
        return(a);
    if (abs(isolevel - val_b) < 0.00001)
        return(b);
    if (abs(val_a - val_b) < 0.00001)
        return(a);
    mu = (isolevel - val_a) / (val_b - val_a);
    p.x = a.x + mu * (b.x - a.x);
    p.y = a.y + mu * (b.y - a.y);
    p.z = a.z + mu * (b.z - a.z);

    return(p);
}

// given filepath, store all triangles information into obj file
void MarchingCube::writeTrianglesIntoObjs(string filepath) {
    ofstream file;
    file.open(filepath + ".obj");

    unordered_map<string, int> map;
    vector<string> face;
    int idx = 1;
    for (MarchingTriangle &tri: *_triangles ) {
        int onetri_idx[3];
        for (int i = 0; i < 3; ++i) {
            string str = "v " + to_string(tri.p[i].x) + " " + to_string(tri.p[i].y) + " " + to_string(tri.p[i].z);
            unordered_map<string, int>::const_iterator got = map.find(str);
            if (got == map.end()) {
                map.insert({str, idx});
                onetri_idx[i] = idx;
                file << str + "\n";
                ++idx;
            } else {
                onetri_idx[i] = got->second;
            }
        }
        string str2 = "f " + to_string(onetri_idx[0]) + " " + to_string(onetri_idx[1]) + " " + to_string(onetri_idx[2]);
        face.push_back(str2);
    }

    for (string &str : face) {
        file << str + "\n";
    }

    file.close();
}