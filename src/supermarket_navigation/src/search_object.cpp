#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <utility>
#include <algorithm>

// Define a struct for storing object information
struct ObjectInfo {
    std::string name;
    int quantity;
    std::pair<double, double> position;
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
    std::getline(infile, line);  // Skip the first header line

    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        ObjectInfo obj;
        char ignore;  // Used to ignore parentheses or commas
        iss >> obj.name >> obj.quantity >> ignore >> obj.position.first >> ignore >> obj.position.second >> ignore;
        objects.push_back(obj);
    }

    infile.close();
    return objects;
}

// Function to search for and display a specific object
void searchAndDisplayObject(const std::vector<ObjectInfo>& objects, const std::string& target_name) {
    bool found = false;
    for (const auto& obj : objects) {
        if (obj.name == target_name) {
            std::cout << "Object found:" << std::endl;
            std::cout << "Name: " << obj.name << ", Quantity: " << obj.quantity
                      << ", Position: (" << obj.position.first << ", " << obj.position.second << ")" << std::endl;
            found = true;
            break;
        }
    }
    if (!found) {
        std::cout << "No object named '" << target_name << "' found." << std::endl;
    }
}

int main() {
    std::string filename = "object_positions.txt";
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
