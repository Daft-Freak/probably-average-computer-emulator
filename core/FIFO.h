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

        data[writeOff] = val;

        writeOff = (writeOff + 1) % size;

        fullFlag = readOff == writeOff;
    }

    T pop()
    {
        if(empty())
            return T(0);

        auto ret = data[readOff];

        readOff = (readOff + 1) % size;

        fullFlag = false;

        return ret;
    }

    T peek()
    {
        if(empty())
            return T(0);

        return data[readOff];
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