#ifndef VECTOR_CLASS
#define VECTOR_CLASS

class Vector {
    public:
        int xComponent, yComponent;
        Vector();
        Vector(int xComponentIn, int yComponentIn);
        int getLength();
};

#endif