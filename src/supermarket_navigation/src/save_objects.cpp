#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <map>
#include <utility>

// Define a struct to store object information
struct ObjectInfo {
    std::string name;
    int quantity;
    std::pair<double, double> position;
};

// Function to save data to a txt file
void saveToTxt(const std::map<std::string, ObjectInfo>& objects, const std::string& filename) {
    // Open the file (if it doesn't exist, it will be created; if it exists, it will be overwritten)
    std::ofstream outfile(filename);

    // Check if the file was successfully opened
    if (!outfile) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return;
    }

    // Write the header row
    outfile << "Object\tQuantity\tPosition (x, y)\n";

    // Iterate over the objects and write each object's information to the file
    for (const auto& [name, obj] : objects) {
        outfile << obj.name << "\t" << obj.quantity << "\t"
                << "(" << obj.position.first << ", " << obj.position.second << ")\n";
    }

    // Close the file stream
    outfile.close();
    std::cout << "Data saved to " << filename << std::endl;
}

int main() {
    // Create and initialize object information
    std::map<std::string, ObjectInfo> objects;
    objects["apple"] = {"apple", 2, {1.06, 1.06}};
    objects["banana"] = {"banana", 1, {0.00, 2.00}};
    objects["milk"] = {"milk", 1, {-1.00, 0.00}};

    // Call the function to save data to a txt file
    saveToTxt(objects, "object_positions.txt");

    return 0;
}
