#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <utility>

// Define a struct for storing object information
struct ObjectInfo {
    std::string name;        // Name of the object (shape)
    int quantity;            // Quantity of the object
    std::string color;       // Color of the object
};

// Function to read object information from a file
std::vector<ObjectInfo> readObjectPositions(const std::string& filename) {
    std::vector<ObjectInfo> objects;
    std::ifstream infile(filename);

    if (!infile) {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return objects;
    }

    std::string line;

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        ObjectInfo obj;
        // Read name, quantity, and color from the line
        iss >> obj.name >> obj.quantity >> obj.color;

        // Check if the line has the expected number of elements
        if (iss.fail()) {
            std::cerr << "Error: Malformed line: " << line << std::endl;
            continue;  // Skip to the next line
        }

        objects.push_back(obj);
    }

    infile.close();
    return objects;
}

// Function to search for and display all instances of a specific object
void searchAndDisplayObject(const std::vector<ObjectInfo>& objects, const std::string& target_name) {
    bool found = false;
    for (const auto& obj : objects) {
        if (obj.name == target_name) {
            if (!found) {
                std::cout << "Objects found:" << std::endl;
                found = true;
            }
            std::cout << "Name: " << obj.name << ", Quantity: " << obj.quantity
                      << ", Color: " << obj.color << std::endl;
        }
    }
    if (!found) {
        std::cout << "No object named '" << target_name << "' found." << std::endl;
    }
}

int main() {
    std::string filename = "detected_shapes.txt";
    std::vector<ObjectInfo> objects = readObjectPositions(filename);

    if (objects.empty()) {
        std::cerr << "No valid object data found in the file." << std::endl;
        return 1;
    }

    std::string target_name;
    std::cout << "Enter the name of the object to search for: ";
    std::cin >> target_name;

    searchAndDisplayObject(objects, target_name);

    return 0;
}
