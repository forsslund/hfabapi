#include <iostream>
#include <string>
#include <windows.h>

const wchar_t* registryPath = L"SOFTWARE\\Haptikfabriken\\HfabAPI";
const wchar_t* valueName = L"ComPort";
const wchar_t* defaultValue = L"COM1";

// Function to read the value from the Registry
std::wstring ReadRegistryValue() {
    HKEY hKey;
    std::wstring value;

    // Open the registry key
    if (RegOpenKeyExW(HKEY_CURRENT_USER, registryPath, 0, KEY_READ, &hKey) == ERROR_SUCCESS) {
        // Query the value
        DWORD dataSize;
        DWORD dataType;

        if (RegQueryValueExW(hKey, valueName, nullptr, &dataType, nullptr, &dataSize) == ERROR_SUCCESS) {
            if (dataType == REG_SZ) {
                wchar_t* buffer = new wchar_t[dataSize / sizeof(wchar_t)];

                if (RegQueryValueExW(hKey, valueName, nullptr, nullptr, reinterpret_cast<LPBYTE>(buffer), &dataSize) == ERROR_SUCCESS) {
                    value = buffer;
                }

                delete[] buffer;
            }
            // Add more branches for different data types if needed

        }
        else {
            std::cerr << "Error querying Registry value." << std::endl;
        }

        // Close the registry key
        RegCloseKey(hKey);

    }
    else {
        std::cerr << "Error opening Registry key." << std::endl;
    }

    return value;
}

// Function to set the value in the Registry
void SetRegistryValue(const std::wstring& value) {
    HKEY hKey;

    // Create or open the registry key
    if (RegCreateKeyExW(HKEY_CURRENT_USER, registryPath, 0, nullptr, 0, KEY_SET_VALUE, nullptr, &hKey, nullptr) == ERROR_SUCCESS) {
        // Set the value
        if (RegSetValueExW(hKey, valueName, 0, REG_SZ, reinterpret_cast<const BYTE*>(value.c_str()), static_cast<DWORD>(value.size() + 1) * sizeof(wchar_t)) != ERROR_SUCCESS) {
            std::cerr << "Error setting Registry value." << std::endl;
        }

        // Close the registry key
        RegCloseKey(hKey);

    }
    else {
        std::cerr << "Error creating/opening Registry key." << std::endl;
    }
}

int main() {
    // Read the value from the Registry
    std::wstring comPort = ReadRegistryValue();
    std::wcout << L"ComPort: " << comPort << std::endl;

    // If the value is not set, prompt the user to enter the value
    //if (comPort.empty()) {
        std::wcout << L"Which com port is the haptic device attached to? (Default: COM1, include the word COM and the number.): ";

        // Use std::getline instead of getline
        std::getline(std::wcin, comPort);

        // Use the default value if the user didn't enter anything
        if (comPort.empty()) {
            comPort = defaultValue;
        }

        // Set the value in the Registry
        SetRegistryValue(comPort);
    //}

    std::wcout << L"ComPort: " << comPort << std::endl;

    return 0;
}
