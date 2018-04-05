#include "polygonetool.hpp"
#include <stdio.h>
using namespace std;
using namespace glm;

typedef struct  //line segment : vertex1 --> vertex2
{
	vec2 vertex1;
	vec2 vertex2;
} edge;

inline double cross2d(vec2 & p1, vec2 & p2)
{
	return p1[0]*p2[1] - p1[1]*p2[0];
}

/* judgement whether point lies on the left sided of lineseg
 * 1 if left, -1 if right 0 if colinear
 */
int left_of(edge & lineseg, vec2 & point)
{
	vec2 tmp1 = lineseg.vertex2 - lineseg.vertex1;
	vec2 tmp2 = point - lineseg.vertex2;

	double crossproduct = cross2d(tmp1,tmp2);
	return (int)sign(crossproduct);
}

int poly_winding(vector<vec2> & polygone)
{
	edge firstedge;
	firstedge.vertex1 = polygone[0];
	firstedge.vertex2 = polygone[1];
	return left_of(firstedge, polygone[2]);
}

bool line_sect(edge & edge1, edge & edge2, vec2 & sectionpoint)
{
	vec2 edge1dir = edge1.vertex2 - edge1.vertex1;
	vec2 edge2dir = edge2.vertex2 - edge2.vertex1;
	vec2 d = edge1.vertex1 - edge2.vertex1;

	float dyx = cross2d(edge2dir,edge1dir);
	if(fabs(dyx) < 1e-8) return false;
	dyx = cross2d(d,edge1dir)/dyx;

	if(dyx <= 0 || dyx >= 1) return false;

	sectionpoint[0] = edge2.vertex1[0] + dyx * edge2dir[0];
	sectionpoint[1] = edge2.vertex1[1] + dyx * edge2dir[1];
	return true;
}

vector<vec2> edgeclip(vector<vec2> subpoly, edge clipline, int dir)
{
	vector<vec2> res;
	int side0, side1;
	vec2 subvec0, subvec1,sectvec;
	edge currentedge;

	subvec0 = subpoly.back();
	side0 = left_of(clipline, subvec0);
	if(side0 != -dir ) res.push_back(subvec0);

	for(int i = 0; i < subpoly.size();  i++)
	{
		subvec1 = subpoly[i];
		
		currentedge.vertex1 = subvec0;
		currentedge.vertex2 = subvec1;

		side1 = left_of(clipline, subvec1);
		if(side1 + side0 == 0 && side0 != 0)
		{
			if(line_sect(clipline,currentedge,sectvec))
				res.push_back(sectvec);
		}

		if(i == subpoly.size() - 1) break;

		if(side1 != -dir) res.push_back(subvec1);

		subvec0 = subvec1;
		side0 = side1;
	}

	return std::move(res);
}


vector<vec2> polyclip(vector<vec2> & clippoly, vector<vec2> & subjpoly)
{
	int clipdir = poly_winding(clippoly);

    edge clippolyedge;

	vector<vec2> afterclipping = subjpoly;

	for(int i = 0; i < clippoly.size(); i++)
	{
		clippolyedge.vertex1 = clippoly[i];
		clippolyedge.vertex2 = clippoly[(i+1)%clippoly.size()];
		
		if(afterclipping.size() == 0) 
			break ;

		afterclipping =  edgeclip(afterclipping,clippolyedge,clipdir);
	}

	return std::move(afterclipping);
}


void drawineps(const char * filename,
			   std::vector<std::vector<glm::vec2>> subpolys, 
			   std::vector<glm::vec2> clippoly, std::vector<std::vector<glm::vec2>> respolys)
{
	int i;

	FILE * eps = fopen(filename, "w");
	fprintf(eps, "%%!PS-Adobe-3.0\n%%%%BoundingBox: 0 0 800 600\n"
		"/l {lineto} def /m{moveto} def /s{setrgbcolor} def"
		"/c {closepath} def /gs {fill grestore stroke} def\n");

	fprintf(eps, "0 setlinewidth %g %g m ", clippoly[0][0], clippoly[0][1]);


	for (i = 1; i < clippoly.size(); i++)
		fprintf(eps, "%g %g l ", clippoly[i][0], clippoly[i][1]);
	fprintf(eps, "c .5 0 0 s gsave 1 .7 .7 s gs\n");
 
	for(auto subpoly:subpolys)
	{
		if(subpoly.empty()) continue;
		fprintf(eps, "%g %g m ", subpoly[0][0], subpoly[0][1]);
		for (i = 1; i < subpoly.size(); i++)
			fprintf(eps, "%g %g l ", subpoly[i][0], subpoly[i][1]);
		fprintf(eps, "c 0 .2 .5 s gsave .4 .7 1 s gs\n");
	}
 
	for(auto respoly:respolys)
	{
		if(respoly.empty()) continue;
		fprintf(eps, "2 setlinewidth [10 8] 0 setdash %g %g m ",
			respoly[0][0], respoly[0][1]);
		for (i = 1; i < respoly.size(); i++)
			fprintf(eps, "%g %g l ", respoly[i][0], respoly[i][1]);
		fprintf(eps, "c .5 0 .5 s gsave .7 .3 .8 s gs\n");
	}

	fprintf(eps, "%%%%EOF");
	fclose(eps);
	printf("eps written\n");

}

float polyarea(std::vector<glm::vec2> polygon)
{
	//compuate the area using Green threom
	//the polygon is assumed to be convex, not self-intersected and do not have internal hole
	float area = 0;

	for(int i = 0; i < polygon.size(); i++)
	{
		glm::vec2 vertex1 = polygon[i];
		glm::vec2 vertex2 = polygon[(i+1)%polygon.size()];

		
		float outdistance = glm::distance(vertex1,vertex2);


		area += cross2d(vertex1,vertex2);
	}



	return fabs(area)/2;


}
