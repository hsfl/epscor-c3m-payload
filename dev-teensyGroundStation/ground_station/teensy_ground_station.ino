/*
 * Ground Station Command Interpreter
 * A comprehensive command interpreter system for Arduino with serial communication
 *
 * Features:
 * - Serial command parsing
 * - Command history
 * - Help system
 * - Error handling
 * - Extensible command structure
 * - Version management
 *
 * Version: 1.0.0
 * Build Date: 2024-01-15
 * Author: Artemis CubeSat Team
 */

#include <Arduino.h>

// Version information
#define VERSION_MAJOR 1
#define VERSION_MINOR 0
#define VERSION_PATCH 0
#define VERSION_BUILD "2025-08-05"
#define VERSION_STRING "1.0.0"
#define BUILD_INFO "Arduino Ground Station Command Interpreter"

// Command structure definition
struct Command
{
    const char *name;
    const char *description;
    void (*function)(const char *args);
};

// Global variables
String inputBuffer = "";
String commandHistory[10];
int historyIndex = 0;
bool commandComplete = false;
bool serialConnected = false;
const int RASPBERRY_PI_GPIO_PIN = 36;

// Forward declarations
void parseCommand(const String &input);
void executeCommand(const char *cmd, const char *args);
void printPrompt();
void addToHistory(const String &command);
void showHistory();
void clearHistory();

// Command function prototypes
void cmdHelp(const char *args);
void cmdVersion(const char *args);
void cmdStatus(const char *args);
void cmdPing(const char *args);
void cmdEcho(const char *args);
void cmdLed(const char *args);
void cmdAnalog(const char *args);
void cmdDigital(const char *args);
void cmdTime(const char *args);
void cmdReset(const char *args);
void cmdHistory(const char *args);
void cmdClear(const char *args);
void cmdRPIControl(const char *args);

// Command table - easily extensible
const Command commands[] = {
    {"help", "Show available commands", cmdHelp},
    {"version", "Show version information", cmdVersion},
    {"status", "Show system status", cmdStatus},
    {"ping", "Test communication", cmdPing},
    {"echo", "Echo back the arguments", cmdEcho},
    {"led", "Control LED (on/off/toggle)", cmdLed},
    {"analog", "Read analog pin (analog <pin>)", cmdAnalog},
    {"digital", "Read/write digital pin (digital <pin> [value])", cmdDigital},
    {"time", "Show uptime", cmdTime},
    {"reset", "Reset the system", cmdReset},
    {"history", "Show command history", cmdHistory},
    {"clear", "Clear screen", cmdClear},
    {"rpi", "Control Raspberry Pi (rpi <on|off|status>)", cmdRPIControl}};

const int numCommands = sizeof(commands) / sizeof(commands[0]);

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);

    // Wait for serial port to connect (optional for some boards)
    while (!Serial)
    {
        delay(10);
    }
    
    // Mark serial as connected and clear any existing buffer
    serialConnected = true;
    inputBuffer = "";

    // Initialize built-in LED
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    // Configure the Raspberry Pi control pin
    pinMode(RASPBERRY_PI_GPIO_PIN, OUTPUT);
    // Set Raspberry Pi to off state initially
    digitalWrite(RASPBERRY_PI_GPIO_PIN, LOW);

    // Clear command history
    clearHistory();

    // Welcome message with version info
    Serial.println("\n================================================");
    Serial.println("    Artemis Ground Station Command Interpreter");
    Serial.println("================================================");
    Serial.print("Version: ");
    Serial.println(VERSION_STRING);
    Serial.print("Build Date: ");
    Serial.println(VERSION_BUILD);
    Serial.print("Build Info: ");
    Serial.println(BUILD_INFO);
    Serial.println("================================================");
    Serial.println("Type 'help' for available commands");
    Serial.println("Type 'version' for detailed version info");
    Serial.println("Type 'clear' to clear the screen");
    Serial.println("================================================\n");

    printPrompt();
}

void loop()
{
    // Check for serial connection state changes
    bool currentlyConnected = Serial;
    
    // Handle serial reconnection
    if (!serialConnected && currentlyConnected)
    {
        Serial.println("\nSerial connection restored.");
        printPrompt();
    }
    // Handle serial disconnection
    else if (serialConnected && !currentlyConnected)
    {
        // Serial disconnected - no action needed
    }
    
    serialConnected = currentlyConnected;
    
    // Read entire line when available
    if (Serial.available())
    {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove whitespace and newlines
        
        if (input.length() > 0)
        {
            parseCommand(input);
            addToHistory(input);
            printPrompt();
        }
    }
}

void parseCommand(const String &input)
{
    // Trim whitespace
    String trimmed = input;
    trimmed.trim();

    if (trimmed.length() == 0)
    {
        return;
    }

    // Find space to separate command and arguments
    int spaceIndex = trimmed.indexOf(' ');
    String command;
    String args;

    if (spaceIndex == -1)
    {
        command = trimmed;
        args = "";
    }
    else
    {
        command = trimmed.substring(0, spaceIndex);
        args = trimmed.substring(spaceIndex + 1);
    }

    // Convert to lowercase for case-insensitive matching
    command.toLowerCase();

    // Execute the command
    executeCommand(command.c_str(), args.c_str());
}

void executeCommand(const char *cmd, const char *args)
{
    // Search for command in command table
    for (int i = 0; i < numCommands; i++)
    {
        if (strcmp(cmd, commands[i].name) == 0)
        {
            commands[i].function(args);
            return;
        }
    }

    // Command not found
    Serial.print("Error: Unknown command '");
    Serial.print(cmd);
    Serial.println("'");
    Serial.println("Type 'help' for available commands");
}

void printPrompt()
{
    Serial.print("GS> ");
}

void addToHistory(const String &command)
{
    // Shift history array
    for (int i = 9; i > 0; i--)
    {
        commandHistory[i] = commandHistory[i - 1];
    }
    commandHistory[0] = command;

    if (historyIndex < 10)
    {
        historyIndex++;
    }
}

void clearHistory()
{
    for (int i = 0; i < 10; i++)
    {
        commandHistory[i] = "";
    }
    historyIndex = 0;
}

// Command implementations
void cmdHelp(const char *args)
{
    Serial.println("\nAvailable Commands:");
    Serial.println("==================");

    for (int i = 0; i < numCommands; i++)
    {
        Serial.print("  ");
        Serial.print(commands[i].name);
        Serial.print(" - ");
        Serial.println(commands[i].description);
    }

    Serial.println("\nUsage: command [arguments]");
    Serial.println("Example: led on, analog 0, digital 13 1");
}

void cmdVersion(const char *args)
{
    Serial.println("\n=== Version Information ===");
    Serial.print("Software: ");
    Serial.println(BUILD_INFO);
    Serial.print("Version: ");
    Serial.print(VERSION_MAJOR);
    Serial.print(".");
    Serial.print(VERSION_MINOR);
    Serial.print(".");
    Serial.println(VERSION_PATCH);
    Serial.print("Build Date: ");
    Serial.println(VERSION_BUILD);
    Serial.print("Arduino Version: ");
    Serial.println(ARDUINO);
    Serial.print("Board: ");
    Serial.println(F_CPU);
    Serial.println("========================");
}

void cmdStatus(const char *args)
{
    Serial.println("\n=== System Status ===");
    cmdTime(args);

    Serial.print("LED Status: ");
    Serial.println(digitalRead(LED_BUILTIN) ? "ON" : "OFF");

    Serial.print("Commands in history: ");
    Serial.println(historyIndex);
}

void cmdPing(const char *args)
{
    Serial.println("Teensy 4.1: PONG");
}

void cmdEcho(const char *args)
{
    if (strlen(args) > 0)
    {
        Serial.print("Echo: ");
        Serial.println(args);
    }
    else
    {
        Serial.println("Usage: echo <message>");
    }
}

void cmdLed(const char *args)
{
    String argStr = String(args);
    argStr.toLowerCase();

    if (argStr == "on")
    {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("LED turned ON");
    }
    else if (argStr == "off")
    {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("LED turned OFF");
    }
    else if (argStr == "toggle")
    {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        Serial.print("LED toggled to ");
        Serial.println(digitalRead(LED_BUILTIN) ? "ON" : "OFF");
    }
    else
    {
        Serial.println("Usage: led [on|off|toggle]");
    }
}

void cmdAnalog(const char *args)
{
    if (strlen(args) == 0)
    {
        Serial.println("Usage: analog <pin>");
        return;
    }

    int pin = atoi(args);
    if (pin < 0 || pin > 15)
    { // Adjust based on your board
        Serial.println("Error: Invalid pin number (0-15)");
        return;
    }

    int value = analogRead(pin);
    float voltage = (value * 5.0) / 1024.0; // Assuming 5V reference

    Serial.print("Analog pin ");
    Serial.print(pin);
    Serial.print(": ");
    Serial.print(value);
    Serial.print(" (");
    Serial.print(voltage, 2);
    Serial.println("V)");
}

void cmdDigital(const char *args)
{
    String argStr = String(args);
    int spaceIndex = argStr.indexOf(' ');

    if (spaceIndex == -1)
    {
        Serial.println("Usage: digital <pin> [value]");
        return;
    }

    int pin = argStr.substring(0, spaceIndex).toInt();
    String valueStr = argStr.substring(spaceIndex + 1);

    if (pin < 0 || pin > 13)
    { // Adjust based on your board
        Serial.println("Error: Invalid pin number (0-13)");
        return;
    }

    if (valueStr.length() == 0)
    {
        // Read mode
        pinMode(pin, INPUT);
        int value = digitalRead(pin);
        Serial.print("Digital pin ");
        Serial.print(pin);
        Serial.print(": ");
        Serial.println(value ? "HIGH" : "LOW");
    }
    else
    {
        // Write mode
        pinMode(pin, OUTPUT);
        int value = valueStr.toInt();
        digitalWrite(pin, value);
        Serial.print("Digital pin ");
        Serial.print(pin);
        Serial.print(" set to ");
        Serial.println(value ? "HIGH" : "LOW");
    }
}

void cmdTime(const char *args)
{
    unsigned long uptime = millis();
    unsigned long seconds = uptime / 1000;
    unsigned long minutes = seconds / 60;
    unsigned long hours = minutes / 60;

    seconds = seconds % 60;
    minutes = minutes % 60;

    Serial.print("Uptime: ");
    if (hours > 0)
    {
        Serial.print(hours);
        Serial.print("h ");
    }
    if (minutes > 0)
    {
        Serial.print(minutes);
        Serial.print("m ");
    }
    Serial.print(seconds);
    Serial.println("s");
}

void cmdReset(const char *args)
{
    Serial.println("Resetting system... manually reconnect to serial after 20 seconds.");
    delay(1000);

    // Software reset for Teensy 4.1 (ARM Cortex-M7)
    SCB_AIRCR = 0x05FA0004;
}

void cmdHistory(const char *args)
{
    Serial.println("\nCommand History:");
    Serial.println("================");

    for (int i = 0; i < historyIndex && i < 10; i++)
    {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.println(commandHistory[i]);
    }

    if (historyIndex == 0)
    {
        Serial.println("No commands in history");
    }
}

void cmdClear(const char *args)
{
    // Clear screen by printing newlines
    for (int i = 0; i < 50; i++)
    {
        Serial.println();
    }

    // Re-print welcome message with version
    Serial.println("================================================");
    Serial.println("    Artemis Ground Station Command Interpreter");
    Serial.println("================================================");
    Serial.print("Version: ");
    Serial.println(VERSION_STRING);
    Serial.print("Build Date: ");
    Serial.println(VERSION_BUILD);
    Serial.println("================================================\n");
}

void cmdRPIControl(const char *args)
{
    String argStr = String(args);
    argStr.toLowerCase();

    if (argStr == "on")
    {
        digitalWrite(RASPBERRY_PI_GPIO_PIN, HIGH);
        Serial.println("RPI turned ON");
    }
    else if (argStr == "off")
    {
        digitalWrite(RASPBERRY_PI_GPIO_PIN, LOW);
        Serial.println("RPI turned OFF");
    }
    else if (argStr == "status")
    {
        Serial.print("RPI Status: ");
        Serial.println(digitalRead(RASPBERRY_PI_GPIO_PIN));
    }
    else
    {
        Serial.println("Usage: rpi [on|off|status]");
    }
}
