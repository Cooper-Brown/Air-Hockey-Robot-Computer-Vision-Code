#ifndef VECTOR_CLASS
#define VECTOR_CLASS

class Vector {
    public:
        float xComponent, yComponent;
        Vector();
        Vector(float xComponentIn, float yComponentIn);
        float getLength();
        Vector getUnitVector();
};

#endif