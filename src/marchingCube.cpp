#include "marchingCube.h"

// constructor
MarchingCube::MarchingCube(double density, double pmass, Real nserach_radius, const vector<array<Real, 3>> &particles,
                           const Vector3R &unitGrid, const Vector3R &minBox, const Vector3R &maxBox, NeighborhoodSearch *neighbors) {
    init(density, pmass, nserach_radius, particles, unitGrid, minBox, maxBox, neighbors);
}

// destructor
MarchingCube::~MarchingCube() {
    destroy();
}

// initialize private members
void MarchingCube::init(double density, double pmass, Real nserach_radius, const vector<array<Real, 3>> &particles,
                   const Vector3R &unitGrid, const Vector3R &minBox, const Vector3R &maxBox, NeighborhoodSearch *neighbors) {
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
    if (!neighbors) {
        // no external neighbor pointer, build one
        _neighbors = new NeighborhoodSearch(nserach_radius, true);
        _neighbors->add_point_set(particles.front().data(), particles.size(), true, true);
        _neighbors->find_neighbors();
        _useExternalNeighbors = false;
    } else {
        // else, use provided neighbor pointer
        _neighbors = neighbors;
        _useExternalNeighbors = true;
    }

    _triangles = new vector<MarchingTriangle>();
}

// delete allocated memories
void MarchingCube::destroy() {
    if (!_useExternalNeighbors) {
        delete _neighbors;
    }
    delete _triangles;
}

// reset private members
void MarchingCube::reset(double density, double pmass, Real nserach_radius, const vector<array<Real, 3>> &particles,
           const Vector3R &unitGrid, const Vector3R &minBox, const Vector3R &maxBox, NeighborhoodSearch *neighbors) {
    destroy();
    init(density, pmass, nserach_radius, particles, unitGrid, minBox, maxBox, neighbors);
}

// get triangles using marching cube algorithm
void MarchingCube::calculateTriangles(double coefficient, double isolevel) {
    for (int i = 0; i < _numGrids[0] - 1; ++i) {
        for (int j = 0; j < _numGrids[1] - 1; ++j) {
            for (int k = 0; k < _numGrids[2] - 1; ++k) {
                Vector3R index(i, j, k);
                MarchingGrid grid;
                getMarchingGrid(grid, coefficient, index);
                _totTriangles += Polygonise(grid, isolevel);
            }
        }
    }
    cout << "tot = " << _totTriangles << endl;
}

// set actual positions and corresponding isovalues for each vertex of the unit cube
void MarchingCube::getMarchingGrid(MarchingGrid &grid, double coefficient, Vector3R &index) {
    // 000, 100, 101, 001, 010, 110, 111, 011
    int order[8][3];
    order[0][0] = 0; order[0][1] = 0; order[0][2] = 0;
    order[1][0] = 1; order[1][1] = 0; order[1][2] = 0;
    order[2][0] = 1; order[2][1] = 0; order[2][2] = 1;
    order[3][0] = 0; order[3][1] = 0; order[3][2] = 1;
    order[4][0] = 0; order[4][1] = 1; order[4][2] = 0;
    order[5][0] = 1; order[5][1] = 1; order[5][2] = 0;
    order[6][0] = 1; order[6][1] = 1; order[6][2] = 1;
    order[7][0] = 0; order[7][1] = 1; order[7][2] = 1;

    for (int i = 0; i < 8; ++i) {
        Vector3R pos = Vector3R((index.x + order[i][0]) * _unitGrid.x + _minBox.x, (index.y + + order[i][1]) * _unitGrid.y + _minBox.y, (index.z + + order[i][2]) * _unitGrid.z + _minBox.z);
        Vector3R norm = getNormal(pos, coefficient);
        grid.v[i].p = pos;
        grid.v[i].n = norm;
        grid.val[i] = getIsoValue(pos, coefficient);
    }
}

// get isovalue for each vertex
double MarchingCube::getIsoValue(Vector3R &pos, double coefficient) {
    Real rpos[3] = {pos.x, pos.y, pos.z};
    double isovalue = 0.0;
    double h = coefficient;
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

Vector3R MarchingCube::getNormal(Vector3R &pos, double coefficient) {
    double step_x = _unitGrid.x, step_y = _unitGrid.y, step_z = _unitGrid.z;
    Vector3R x_plus = Vector3R(pos.x + step_x, pos.y, pos.z), x_minus = Vector3R(pos.x - step_x, pos.y, pos.z);
    Vector3R y_plus = Vector3R(pos.x, pos.y + step_y, pos.z), y_minus = Vector3R(pos.x, pos.y - step_y, pos.z);
    Vector3R z_plus = Vector3R(pos.x, pos.y, pos.z + step_z), z_minus = Vector3R(pos.x, pos.y, pos.z - step_z);
    Vector3R normal = Vector3R(getIsoValue(x_plus, coefficient) - getIsoValue(x_minus, coefficient),
            getIsoValue(y_plus, coefficient) - getIsoValue(y_minus, coefficient),
            getIsoValue(z_plus, coefficient) - getIsoValue(z_minus, coefficient));
    normal.normalize();
    return normal;
}

/* Given a grid cell and an isolevel, calculate the triangular facets required to represent the isosurface through the cell.
 * Return the number of triangular facets, the array "triangles" will be loaded up with the vertices at most 5 triangular facets.
 * 0 will be returned if the grid cell is either totally above of totally below the isolevel. */
int MarchingCube::Polygonise(MarchingGrid &grid, double isolevel) {
    int ntriang = 0;
    int cubeindex = 0;
    Vertex vertlist[12];

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
        vertlist[0] = VertexInterp(isolevel,grid.v[0],grid.v[1],grid.val[0],grid.val[1]);
    if (_edgeTable[cubeindex] & 2)
        vertlist[1] = VertexInterp(isolevel,grid.v[1],grid.v[2],grid.val[1],grid.val[2]);
    if (_edgeTable[cubeindex] & 4)
        vertlist[2] = VertexInterp(isolevel,grid.v[2],grid.v[3],grid.val[2],grid.val[3]);
    if (_edgeTable[cubeindex] & 8)
        vertlist[3] = VertexInterp(isolevel,grid.v[3],grid.v[0],grid.val[3],grid.val[0]);
    if (_edgeTable[cubeindex] & 16)
        vertlist[4] = VertexInterp(isolevel,grid.v[4],grid.v[5],grid.val[4],grid.val[5]);
    if (_edgeTable[cubeindex] & 32)
        vertlist[5] = VertexInterp(isolevel,grid.v[5],grid.v[6],grid.val[5],grid.val[6]);
    if (_edgeTable[cubeindex] & 64)
        vertlist[6] = VertexInterp(isolevel,grid.v[6],grid.v[7],grid.val[6],grid.val[7]);
    if (_edgeTable[cubeindex] & 128)
        vertlist[7] = VertexInterp(isolevel,grid.v[7],grid.v[4],grid.val[7],grid.val[4]);
    if (_edgeTable[cubeindex] & 256)
        vertlist[8] = VertexInterp(isolevel,grid.v[0],grid.v[4],grid.val[0],grid.val[4]);
    if (_edgeTable[cubeindex] & 512)
        vertlist[9] = VertexInterp(isolevel,grid.v[1],grid.v[5],grid.val[1],grid.val[5]);
    if (_edgeTable[cubeindex] & 1024)
        vertlist[10] = VertexInterp(isolevel,grid.v[2],grid.v[6],grid.val[2],grid.val[6]);
    if (_edgeTable[cubeindex] & 2048)
        vertlist[11] = VertexInterp(isolevel,grid.v[3],grid.v[7],grid.val[3],grid.val[7]);

    /* Create the triangle */
    for (int i = 0; _triTable[cubeindex][i] != -1; i += 3) {
        MarchingTriangle tri;
        tri.v[0] = vertlist[_triTable[cubeindex][i]];
        tri.v[1] = vertlist[_triTable[cubeindex][i+1]];
        tri.v[2] = vertlist[_triTable[cubeindex][i+2]];
        _triangles->push_back(tri);
        ntriang++;
    }
    return(ntriang);
}

/* Linearly interpolate the position where an isosurface cuts an edge between two vertices, each with their own scalar value */
Vertex MarchingCube::VertexInterp(double isolevel, Vertex &a, Vertex &b, double val_a, double val_b) {
    double mu;
    Vertex v;

    if (abs(isolevel - val_a) < 0.00001)
        return(a);
    if (abs(isolevel - val_b) < 0.00001)
        return(b);
    if (abs(val_a - val_b) < 0.00001)
        return(a);
    mu = (isolevel - val_a) / (val_b - val_a);
    v.p.x = a.p.x + mu * (b.p.x - a.p.x);
    v.p.y = a.p.y + mu * (b.p.y - a.p.y);
    v.p.z = a.p.z + mu * (b.p.z - a.p.z);

    v.n.x = a.n.x + mu * (b.n.x - a.n.x);
    v.n.y = a.n.y + mu * (b.n.y - a.n.y);
    v.n.z = a.n.z + mu * (b.n.z - a.n.z);

    return(v);
}

// given filepath, store all triangles information into obj file
void MarchingCube::writeTrianglesIntoObjs(string filepath) {
    ofstream file;
    file.open(filepath + ".obj");

    unordered_map<string, int> map;
    vector<string> face;
    int idx = 1;
    for (MarchingTriangle &tri: *_triangles) {
        int onetri_idx[3];
        for (int i = 0; i < 3; ++i) {
            string pos = "v " + to_string(tri.v[i].p.x) + " " + to_string(tri.v[i].p.y) + " " + to_string(tri.v[i].p.z);
            string norm = "vn " + to_string(tri.v[i].n.x) + " " + to_string(tri.v[i].n.y) + " " + to_string(tri.v[i].n.z);
            unordered_map<string, int>::const_iterator got = map.find(pos);
            if (got == map.end()) {
                map.insert({pos, idx});
                onetri_idx[i] = idx;
                file << pos + "\n";
                file << norm + "\n";
                ++idx;
            } else {
                onetri_idx[i] = got->second;
            }
        }
        string face = "f " + to_string(onetri_idx[0]) + "//" + to_string(onetri_idx[0]) + " " + to_string(onetri_idx[1]) + "//" + to_string(onetri_idx[1]) + " " + to_string(onetri_idx[2]) + "//" + to_string(onetri_idx[2]);
        file << face << "\n";
    }

    file.close();
}
