#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>

//-- Object Information Structure ----------------------------------------------------------
struct ObjectInfo {
    std::string name;        // Name of the object (e.g., shape)
    int quantity;            // Quantity of the object
    std::string color;       // Color of the object
};

//-- Function to Read Object Information from a File ---------------------------------------
std::vector<ObjectInfo> readObjectPositions(const std::string& filename) {
    std::vector<ObjectInfo> objects;
    std::ifstream infile(filename);

    if (!infile) {
        // Handle unopenable file error
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        return objects;
    }

    std::string line;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        ObjectInfo obj;

        // Parse line elements into the struct, checking input format
        if (!(iss >> obj.name >> obj.quantity >> obj.color)) {
            std::cerr << "Error: Malformed line: " << line << std::endl;
            continue;  // Skip invalid lines
        }

        objects.push_back(obj);
    }

    infile.close();  // Close file once data is read
    return objects;
}

//-- Function to Search for and Display Specific Objects ------------------------------------
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

    // Handle the case for an object not found
    if (!found) {
        std::cout << "No object named '" << target_name << "' found." << std::endl;
    }
}

//-- Main Function --------------------------------------------------------------------------
int main() {
    const std::string filename = "detected_shapes.txt";
    std::vector<ObjectInfo> objects = readObjectPositions(filename);

    // If there is nothing inside the file to be scanned
    if (objects.empty()) {
        std::cerr << "No valid object data found in the file." << std::endl;
        return 1;
    }

    // Object Search user input
    std::string target_name;
    std::cout << "Enter the name of the object to search for: ";
    std::cin >> target_name;

    searchAndDisplayObject(objects, target_name);

    return 0;
}
