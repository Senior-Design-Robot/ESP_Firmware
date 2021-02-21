#pragma once

#include "kqueue.h"

enum PathElementType
{
    PATH_ZERO = 0,
    PATH_MOVE,
    PATH_PEN_UP,
    PATH_PEN_DOWN
};

typedef struct PathElement
{
    PathElementType type;
    float x;
    float y;

    PathElement() : type(PATH_ZERO), x(0), y(0) {}
    PathElement( PathElementType type, float x, float y ) : type(type), x(x), y(y) {}
} PathElement;

class IPathIterator
{
public:
    IPathIterator() {}
    virtual ~IPathIterator() {}

    virtual PathElement moveNext() = 0;

protected:
    static constexpr float MAX_DELTA = 0.1;
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

public:
    PathQueueIterator();

    void addMove( float x, float y, bool direct = false );
    void addPenMove( bool down );
    void clear();

    PathElement moveNext();
};

