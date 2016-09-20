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

#include "piece.h"

Piece::Piece(std::vector<cv::Point> shape, float picking_height, float picking_orientation[4], std::string name)
{
    this->shape = shape;
    this->picking_height = picking_height;
    this->picking_orientation[0] = picking_orientation[0];
    this->picking_orientation[1] = picking_orientation[1];
    this->picking_orientation[2] = picking_orientation[2];
    this->picking_orientation[3] = picking_orientation[3];
    this->name = name;
    for(int i = 0; i<3; i++)
        drop_position[i] = 0;
    for(int i = 0; i<4; i++)
        drop_orientation[i] = 0;
}

void Piece::setDropPosition(float position[], float orientation[])
{
    drop_position[0] = position[0];
    drop_position[1] = position[1];
    drop_position[2] = position[2];
    drop_orientation[0] = orientation[0];
    drop_orientation[1] = orientation[1];
    drop_orientation[2] = orientation[2];
    drop_orientation[3] = orientation[3];
}

std::string Piece::getName()
{
    return name;
}

std::string Piece::serialize()
{
    std::stringstream s;
    s << "name: " << name << std::endl; 
    s << "shape: [(" << shape[0].x << ", " << shape[0].y << ")";
    for(int i = 1; i < shape.size(); i++)
        s << ", (" << shape[i].x <<", "<< shape[i].y << ")";
    s << "]" << std::endl;
    s << "picking_height: " << picking_height << std::endl;
    s << "picking_orientation: [" << picking_orientation[0];
    for(int i = 1; i < 4; i++)
        s << ", " << picking_orientation[i];
    s << "]" << std::endl;
    s << "drop_position: [" << drop_position[0];
    for(int i = 1; i < 3; i++)
        s << ", " << drop_position[i];
    s << "]" << std::endl;
    s << "drop_orientation: [" << drop_position[0];
    for(int i = 1; i < 4; i++)
        s << ", " << drop_position[i];
    s << "]" << std::endl;
    return s.str();
}

void Piece::deserialize(std::string s)
{

}

double Piece::match(std::vector<cv::Point> &shape)
{
    return cv::matchShapes(this->shape, shape, CV_CONTOURS_MATCH_I3, 0);
}

float Piece::getPickingHeight()
{
    return picking_height;
}

void Piece::getPickingOrientation(float picking_orientation[])
{
    picking_orientation[0] = this->picking_orientation[0];
    picking_orientation[1] = this->picking_orientation[1];
    picking_orientation[2] = this->picking_orientation[2];
    picking_orientation[3] = this->picking_orientation[3];
}

void Piece::getDropPosition(float drop_position[])
{
    drop_position[0] = this->drop_position[0];
    drop_position[1] = this->drop_position[1];
    drop_position[2] = this->drop_position[2];
}

void Piece::getDropOrientation(float drop_orientation[])
{
    drop_orientation[0] = this->drop_orientation[0];
    drop_orientation[1] = this->drop_orientation[1];
    drop_orientation[2] = this->drop_orientation[2];
    drop_orientation[3] = this->drop_orientation[3];
}

int closestMatch(std::vector<Piece> &pieces, std::vector<cv::Point> &shape, double threshold)
{
    int match = -1;
    double closest = threshold;
    for(int j = 0; j<pieces.size(); j++)
    {
        double score = pieces[j].match(shape);
        if( score < closest)
        {
            match = j;
            closest = score;
        }
    }
    return match;
}

void closestMatch(std::vector<Piece> &pieces, std::vector<std::vector<cv::Point> > &shapes, double threshold, int &idxp, int &idxs)
{
    idxs = -1;
    idxp = -1;
    double closest = threshold;
    for(int i = 0; i<shapes.size(); i++)
    {
        for(int j = 0; j<pieces.size(); j++)
        {
            double score = pieces[j].match(shapes[i]);
            if( score < closest)
            {
                idxs = i;
                idxp = j;
                closest = score;
            }
        }
    }
}

cv::Moments Piece::getMoments()
{
    return cv::moments(shape);
}
