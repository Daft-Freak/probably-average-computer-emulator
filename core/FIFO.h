#pragma once
// FIFO helper using circular buffer

template<class T, int size>
class FIFO final
{
public:
    void push(T val)
    {
        if(fullFlag)
            return;

        data[writeOff++] = val;

        if(writeOff == size)
            writeOff = 0;

        fullFlag = readOff == writeOff;
    }

    T pop()
    {
        if(empty())
            return T(0);

        auto ret = data[readOff++];

        if(readOff == size)
            readOff = 0;

        fullFlag = false;

        return ret;
    }

    bool empty() const
    {
        return !fullFlag && readOff == writeOff;
    }

    bool full() const {return fullFlag;}

    int getCount() const
    {
        if(fullFlag)
            return size;

        if(writeOff >= readOff)
            return writeOff - readOff;

        return writeOff + size - readOff;
    }

private:
    T data[size];

    int readOff = 0, writeOff = 0;
    bool fullFlag = false;
};