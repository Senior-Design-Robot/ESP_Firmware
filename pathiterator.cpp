#include "pathiterator.h"
#include <cmath>

CirclePathIterator::CirclePathIterator( float centerX, float centerY, float radius ) :
    cX(centerX), cY(centerY), r(radius)
{
    theta = 0.0;
    dtheta = MAX_DELTA / r;
}

PathElement CirclePathIterator::moveNext()
{
    float curX = cX + r * cosf(theta);
    float curY = cY + r * sinf(theta);
    theta += dtheta;
    if( theta > (2 * M_PI) )
    {
        theta -= (2 * M_PI);
    }

    return PathElement(PATH_MOVE, curX, curY);
}


SquarePathIterator::SquarePathIterator( float centerX, float centerY, float radius ) :
    cX(centerX), cY(centerY), r(radius)
{
    theta = 0.0;
    dtheta = atanf(r / (r - MAX_DELTA)) - M_PI_4;
    diag = r * M_SQRT2;
}

PathElement SquarePathIterator::moveNext()
{
    PathElement nextElem(PATH_MOVE, 0, 0);

    if( theta < M_PI_4 || theta > (7 * M_PI_4) )
    {
        // right
        nextElem.x = cX + r;
        nextElem.y = cY - diag * sin(theta);
    }
    else if( theta < (3 * M_PI_4) )
    {
        // top
        nextElem.x = cX + diag * cos(theta);
        nextElem.y = cY - r;
    }
    else if( theta < (5 * M_PI_4) )
    {
        // left
        nextElem.x = cX - r;
        nextElem.y = cY - diag * sin(theta);
    }
    else
    {
        // bottom
        nextElem.x = cX + diag * cos(theta);
        nextElem.y = cY + r;
    }

    theta += dtheta;
    if( theta > (2 * M_PI) )
    {
        theta -= (2 * M_PI);
    }

    return nextElem;
}


//------------------------------------------------------------------
// PathQueueIterator
PathQueueIterator::PathQueueIterator()
{

}

PathElement PathQueueIterator::getPathEndLoc()
{
    PathElement lastPoint;
    if( pathQueue.peekTail(lastPoint) ) return lastPoint;
    else return lastMove;
}

void PathQueueIterator::addSubdivisions( float startX, float startY, float endX, float endY )
{
    float dx = endX - startX;
    float dy = endY - startY;
    float dist = sqrtf(dx * dx + dy * dy);

    // check if need to split up the path into segments
    if( dist > MAX_DELTA )
    {
        int nDiv = (int)ceilf(dist / MAX_DELTA);
        dx = dx / nDiv;
        dy = dy / nDiv;

        float lastX = startX;
        float lastY = startY;
        float nextX;
        float nextY;

        for( int i = 0; i < (nDiv - 1); i++ )
        {
            nextX = lastX + dx;
            nextY = lastY + dy;

            pathQueue.push_back(PathElement(PATH_MOVE, nextX, nextY));

            lastX = nextX;
            lastY = nextY;
        }
    }

    pathQueue.push_back(PathElement(PATH_MOVE, endX, endY));
}

void PathQueueIterator::addMove( float x, float y, bool direct )
{
    if( direct )
    {
        pathQueue.push_back(PathElement(PATH_MOVE, x, y));
        return;
    }

    PathElement lastLoc = getPathEndLoc();
    addSubdivisions(lastLoc.x, lastLoc.y, x, y);
}

void PathQueueIterator::addPenMove( bool down )
{
    PathElementType type = down ? PATH_PEN_DOWN : PATH_PEN_UP;
    PathElement lastLoc = getPathEndLoc();

    pathQueue.push_back(PathElement(type, lastLoc.x, lastLoc.y));
}

void PathQueueIterator::clear()
{
    pathQueue.clear();
}

PathElement PathQueueIterator::moveNext()
{
    PathElement step;
    if( pathQueue.pop(step) )
    {
        lastMove = step;
        return step;
    }
    else return PathElement();
}
