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

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include "piece.h"

using namespace std;

int main(int argc, char **argv)
{
    vector<cv::Point> shape;
    shape.push_back(cv::Point(0,0));
    shape.push_back(cv::Point(1,0));
    shape.push_back(cv::Point(1,1));
    shape.push_back(cv::Point(0,1));
    float height = 0.8, drop_position[3], drop_orientation[4], picking_orientation[4];
    std::string name("Serialization test piece");
    for(int i = 0; i<3; i++)
        drop_position[i] = 0.2*(i+1);
    for(int i = 0; i<4; i++)
    {
        drop_orientation[i] = 0.3*(i+1);
        picking_orientation[i] = 0.5*(i+1);
    }
    Piece piece(shape, height, picking_orientation, name);
    piece.setDropPosition(drop_position, drop_orientation);
    cout << piece.serialize();
}
