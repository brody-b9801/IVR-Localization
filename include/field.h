#pragma once
#include <iostream>
#include <vector>

using namespace std;

class field {
    public: 
        array<double, 3> simulateCast(array<double, 3> castPosition, array<double, 3> castDirection);

    private:
        struct object {
            string type;
            double min[3];
            double max[3];
            bool removed;
        };

        double hitDistance(object); //return -1 if not hit
        
        vector<object> objects;
};