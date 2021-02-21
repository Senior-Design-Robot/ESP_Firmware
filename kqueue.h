#pragma once

template <typename T>
class KQueue
{
protected:
    static constexpr int INIT_CAP = 32;

    T *arr;
    int capacity;
    int headIdx;
    int count;

    int advance_idx( int cur_idx )
    {
        cur_idx += 1;
        if( cur_idx >= capacity )
        {
            // loop back to start of array
            cur_idx = 0;
        }

        return cur_idx;
    }

    int insert_idx()
    {
        int idx = headIdx + count;
        if( idx >= capacity )
        {
            // looped around
            idx -= capacity;
        }
        return idx;
    }

    void increase_cap()
    {
        int newCap = (capacity * 3) / 2; // growth factor 1.5
        T *newArr = new T[newCap];

        int readIdx = headIdx;
        // foreach element in previous array, copy, shifting head to front of array
        for( int i = 0; i < capacity; i++ )
        {
            newArr[i] = arr[readIdx];
            readIdx = advance_idx(readIdx);
        }

        delete arr;
        arr = newArr;
        headIdx = 0;
        capacity = newCap;
    }

public:
    KQueue() : arr(new T[INIT_CAP]), capacity(INIT_CAP), headIdx(0), count(0) {}

    ~KQueue()
    {
        delete arr;
    }

    int size()
    {
        return count;
    }

    void push_back( const T& newElem )
    {
        if( count == 0 )
        {
            headIdx = 0; // if empty make sure we refill from beginning
        }

        if( count == capacity )
        {
            increase_cap();
        }

        arr[insert_idx()] = newElem;
        count++;
    }

    bool pop( T& val )
    {
        if( count > 0 )
        {
            val = arr[headIdx];
            headIdx = advance_idx(headIdx);
            count--;
            return true;
        }
        return false;
    }

    bool peek( T& val )
    {
        if( count > 0 )
        {
            val = arr[headIdx];
            return true;
        }
        return false;
    }

    bool peekTail( T& val )
    {
        if( count > 0 )
        {
            int tailIdx = insert_idx() - 1;
            if( tailIdx < 0 ) tailIdx = capacity - 1;
            val = arr[tailIdx];
            return true;
        }
        return false;
    }

    void clear()
    {
        headIdx = 0;
        count = 0;
    }
};

