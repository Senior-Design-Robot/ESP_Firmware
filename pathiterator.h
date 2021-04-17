#pragma once

#include "kqueue.h"

#define PEN_UP false
#define PEN_DOWN true

#define MIN_PATH_FILL 15

enum PathElementType
{
    PATH_NONE = 0,
    PATH_MOVE = 1,
    PATH_PEN_UP = 2,
    PATH_PEN_DOWN = 3,
    PATH_END = 4
};

typedef struct PathElement
{
    PathElementType type;
    float x;
    float y;

    PathElement() : type(PATH_NONE), x(0), y(0) {}
    PathElement( PathElementType type, float x, float y ) : type(type), x(x), y(y) {}
} PathElement;

class IPathIterator
{
public:
    IPathIterator() {}
    virtual ~IPathIterator() {}

    virtual PathElement moveNext() = 0;

protected:
    static constexpr float MAX_DELTA = 0.3;
};

class CirclePathIterator : public IPathIterator
{
private:
    float cX;
    float cY;
    float r;
    float theta;
    float dtheta;

public:
    CirclePathIterator( float centerX, float centerY, float radius );

    PathElement moveNext();
};


class SquarePathIterator : public IPathIterator
{
private:
    float cX;
    float cY;
    float r;
    float diag;
    float theta;
    float dtheta;

public:
    SquarePathIterator( float centerX, float centerY, float radius );

    PathElement moveNext();
};


class PathQueueIterator : public IPathIterator
{
private:
    KQueue<PathElement> pathQueue;
    PathElement lastMove;

    PathElement getPathEndLoc();
    void addSubdivisions( float startX, float startY, float endX, float endY );

    static PathElement interpolate( const PathElement &target, const PathElement &current );

public:
    PathQueueIterator();

    int remaining();

    void addMove( float x, float y );
    void addPenMove( bool down );
    void addElement( const PathElement &new_elem );
    void clear();

    PathElement moveNext();
};
