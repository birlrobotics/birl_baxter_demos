/*
This file is part of Baxter Pick and Learn.

Baxter Pick and Learn is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Nagen is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program. If not, see <http://www.gnu.org/licenses/>.

Copyright 2014 Charles Hubain <charles.hubain@haxelion.eu>
*/

#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <string>
#include <sstream>

class Piece
{
public:
    Piece(std::vector<cv::Point> shape, float picking_height, float picking_orientation[4], std::string name);
    void setDropPosition(float position[], float orientation[]);
    double match(std::vector<cv::Point> &shape);
    float getPickingHeight();
    void getPickingOrientation(float picking_orientation[]);
    void getDropPosition(float drop_position[]);
    void getDropOrientation(float drop_orientation[]);
    cv::Moments getMoments();
    std::string getName();
    std::string serialize();
    void deserialize(std::string s);

private:
    std::vector<cv::Point> shape;
    std::string name;
    float picking_height, picking_orientation[4], drop_position[3], drop_orientation[4];
};

int closestMatch(std::vector<Piece> &pieces, std::vector<cv::Point> &shape, double threshold);
void closestMatch(std::vector<Piece> &pieces, std::vector<std::vector<cv::Point> > &shapes, double threshold, int &idxp, int &idxs);
