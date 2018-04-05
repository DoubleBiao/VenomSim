#include <vector>
#include <glm/glm.hpp>

std::vector<glm::vec2> polyclip(std::vector<glm::vec2> & clippoly, std::vector<glm::vec2> & subjpoly);
void drawineps(const char * filename,
			   std::vector<std::vector<glm::vec2>> subpoly, 
			   std::vector<glm::vec2> clippoly, 
			   std::vector<std::vector<glm::vec2>> respoly);

float polyarea(std::vector<glm::vec2> polygon);
