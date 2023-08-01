//============================================================================
// Name        : drv-cpp.cpp
// Author      : Miguel A. Calveiro
// Version     : 1.0
// Copyright   : UTN
// Description : Driver para controlar el puerto COM y publicar datos recebidos.
//============================================================================

#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <thread>

#ifdef _WIN32
#include <conio.h>
#include <Windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#endif

// Interfaz para el puerto serie
class ISerialPort {
public:
    virtual ~ISerialPort() {}
    virtual bool Open() = 0;
    virtual void Close() = 0;
    virtual bool Read(std::vector<char>& data) = 0;
};

#ifdef _WIN32
// Implementación de la clase para el puerto serie en Windows
class SerialPortWindows : public ISerialPort {
public:
    SerialPortWindows(const std::wstring& portName)
        : hComm(INVALID_HANDLE_VALUE), portName(portName) {}

    ~SerialPortWindows() override {
        Close();
    }

    bool Open() override {
        hComm = CreateFileW(portName.c_str(), GENERIC_READ, 0, NULL, OPEN_EXISTING, 0, NULL);
        if (hComm == INVALID_HANDLE_VALUE) {
            std::wcout << L"Error al abrir el puerto COM." << std::endl;
            return false;
        }

        DCB dcbSerialParams = { 0 };
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hComm, &dcbSerialParams)) {
            std::wcout << L"Error al obtener la configuración del puerto COM." << std::endl;
            Close();
            return false;
        }

        dcbSerialParams.BaudRate = CBR_9600;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hComm, &dcbSerialParams)) {
            std::wcout << L"Error al configurar el puerto COM." << std::endl;
            Close();
            return false;
        }

        return true;
    }

    void Close() override {
        if (hComm != INVALID_HANDLE_VALUE) {
            CloseHandle(hComm);
            hComm = INVALID_HANDLE_VALUE;
        }
    }

    bool Read(std::vector<char>& data) override {
        char incomingData[256];
        DWORD bytesRead = 0;

        if (ReadFile(hComm, incomingData, sizeof(incomingData), &bytesRead, NULL)) {
            // Copiar los datos leídos al vector
            data.assign(incomingData, incomingData + bytesRead);
            return true;
        }

        return false;
    }

private:
    HANDLE hComm = INVALID_HANDLE_VALUE;
    std::wstring portName;
};
#else
// Implementación de la clase para el puerto serie en sistemas Unix (Linux y macOS)
class SerialPortUnix : public ISerialPort {
public:
    SerialPortUnix(const std::string& portName)
        : fd(-1), portName(portName) {}

    ~SerialPortUnix() override {
        Close();
    }

    bool Open() override {
        fd = open(portName.c_str(), O_RDWR | O_NOCTTY);
        if (fd == -1) {
            std::cout << "Error al abrir el puerto serie." << std::endl;
            return false;
        }

        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            std::cout << "Error al obtener la configuración del puerto serie." << std::endl;
            Close();
            return false;
        }

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;
        tty.c_lflag &= ~ICANON;
        tty.c_lflag &= ~ECHO;
        tty.c_lflag &= ~ECHOE;
        tty.c_lflag &= ~ECHONL;
        tty.c_lflag &= ~ISIG;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
        tty.c_oflag &= ~OPOST;
        tty.c_oflag &= ~ONLCR;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 10;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            std::cout << "Error al configurar el puerto serie." << std::endl;
            Close();
            return false;
        }

        return true;
    }

    void Close() override {
        if (fd != -1) {
            close(fd);
            fd = -1;
        }
    }

    bool Read(std::vector<char>& data) override {
        char incomingData[256];
        ssize_t bytesRead = read(fd, incomingData, sizeof(incomingData));
        if (bytesRead > 0) {
            data.assign(incomingData, incomingData + bytesRead);
            return true;
        }
        return false;
    }

private:
    int fd;
    std::string portName;
};
#endif

bool isSpaceKeyPressed() {
#ifdef _WIN32
    // Utilizamos GetAsyncKeyState para verificar si la barra espaciadora ha sido presionada
    return (GetAsyncKeyState(VK_SPACE) & 0x8000) != 0;
#else
    // Otras plataformas (Linux, macOS, etc.) pueden usar una implementación diferente
    // de la función para detectar si una tecla ha sido presionada
    return false;
#endif
}

int main() {
    // Cambia "COM6" por el puerto COM que desees utilizar en Windows,
    // o "/dev/ttyS0" por el puerto que desees utilizar en sistemas Unix (Linux y macOS).
#ifdef _WIN32
    std::wstring portName = L"COM6";
#else
    std::string portName = "/dev/ttyS0";
#endif

    // Crear el objeto del puerto serie según el sistema operativo
#ifdef _WIN32
    SerialPortWindows serialPort(portName);
#else
    SerialPortUnix serialPort(portName);
#endif

    if (!serialPort.Open()) {
        std::wcout << L"No se pudo abrir el puerto serie." << std::endl;
        return 1;
    }

    std::cout << "Esperando datos..." << std::endl;

    while (true) {
        if (isSpaceKeyPressed()) { // Verificar si se ha presionado la barra espaciadora
            break; // Salir del bucle si se ha presionado la barra espaciadora
        }

        std::vector<char> receivedData;
        if (serialPort.Read(receivedData)) {
            for (char c : receivedData) {
                std::cout << c;
            }
            std::cout << std::endl;
        }

        // Añadimos una pausa de 100 ms para no sobrecargar el procesador
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

#ifdef _WIN32
    // Para asegurar que la consola no se cierre inmediatamente en Windows
    std::cout << "\nPresione Enter para salir...";
    std::cin.ignore();
#endif

    std::cout << "\nPrograma detenido por la barra espaciadora." << std::endl;

    return 0;
}

