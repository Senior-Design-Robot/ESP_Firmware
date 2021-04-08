#include "pathiterator.h"
#include <math.h>

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

PathElement PathQueueIterator::interpolate( const PathElement &target, const PathElement &current )
{
    float dx = target.x - current.x;
    float dy = target.y - current.y;
    float dist = sqrtf(dx * dx + dy * dy);

    // check if need to split up the path into segments
    if( dist > MAX_DELTA )
    {
        // next = goal vector / magnitude * MAX_DELTA
        float nextX = current.x + (dx * MAX_DELTA / dist);
        float nextY = current.y + (dy * MAX_DELTA / dist);
        return PathElement(PATH_MOVE, nextX, nextY);
    }
    return target;
}

int PathQueueIterator::remaining()
{
    return pathQueue.size();
}

void PathQueueIterator::addMove( float x, float y )
{
    pathQueue.push_back(PathElement(PATH_MOVE, x, y));
}

void PathQueueIterator::addPenMove( bool down )
{
    PathElementType type = down ? PATH_PEN_DOWN : PATH_PEN_UP;
    PathElement lastLoc = getPathEndLoc();

    pathQueue.push_back(PathElement(type, lastLoc.x, lastLoc.y));
}

void PathQueueIterator::addElement( const PathElement &new_elem )
{
    pathQueue.push_back(new_elem);
}

void PathQueueIterator::clear()
{
    pathQueue.clear();
}

PathElement PathQueueIterator::moveNext()
{
    PathElement target;
    if( pathQueue.peek(target) )
    {
        if( target.type != PATH_MOVE )
        {
            // pen up is always immediate
            pathQueue.pop(lastMove);
            return lastMove;
        }
        else
        {
            // other moves are interpolated
            if( (lastMove.x == target.x) && (lastMove.y == target.y) )
            {
                // finished previous move, start new one
                pathQueue.pop(target);
                if( !pathQueue.peek(target) ) return PathElement();
            }

            // interpolate to current target
            lastMove = interpolate(target, lastMove);
            return lastMove;
        }
    }
    
    return PathElement();
}
