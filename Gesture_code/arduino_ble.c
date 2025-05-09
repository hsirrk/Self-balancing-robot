#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#define MOVEMENT_UUID "69042F04-F8C3-4F4A-A8F4-15CD926DA146"

// Predefined Commands
#define FORWARD "forward"
#define BACKWARD "backward"
#define LEFT "left"
#define RIGHT "right"
#define STOP "stop"

// Track the last state of the button press
int previous_state[5] = {0};  // 'w', 's', 'a', 'd', ' '

// Function to get the bytearray value for a given command
const char* get_movement_value(char command) {
    switch (command) {
        case 'w': return FORWARD;
        case 's': return BACKWARD;
        case 'a': return LEFT;
        case 'd': return RIGHT;
        case ' ': return STOP;
        default: return STOP;
    }
}

// Function to connect to the Bluetooth device
int connect_bluetooth_device() {
    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        perror("Error getting Bluetooth device");
        return -1;
    }
    
    int sock = hci_open_dev(dev_id);
    if (sock < 0) {
        perror("Error opening Bluetooth socket");
        return -1;
    }

    struct sockaddr_rc addr = { 0 };
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;

    str2ba("00:1A:7D:DA:71:13", &addr.rc_bdaddr);  // Replace with your Arduino MAC address
    
    if (connect(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error connecting to Bluetooth device");
        close(sock);
        return -1;
    }

    return sock;
}

// Function to send movement command to Arduino device
void send_movement_command(int sock, const char *command) {
    if (write(sock, command, strlen(command)) < 0) {
        perror("Error writing to Bluetooth device");
    } else {
        printf("Sent movement command: %s\n", command);
    }
}

// Function to detect key presses (non-blocking)
int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    
    ch = getchar();
    
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    
    return 0;
}

// Function to check the key press and send corresponding command
void check_keys(int sock) {
    char key = ' ';
    
    if (kbhit()) {
        key = getchar();
        if (key == 'w' && !previous_state[0]) {
            send_movement_command(sock, get_movement_value('w'));
            previous_state[0] = 1;
        } else if (key == 's' && !previous_state[1]) {
            send_movement_command(sock, get_movement_value('s'));
            previous_state[1] = 1;
        } else if (key == 'a' && !previous_state[2]) {
            send_movement_command(sock, get_movement_value('a'));
            previous_state[2] = 1;
        } else if (key == 'd' && !previous_state[3]) {
            send_movement_command(sock, get_movement_value('d'));
            previous_state[3] = 1;
        } else if (key == ' ' && !previous_state[4]) {
            send_movement_command(sock, get_movement_value(' '));
            previous_state[4] = 1;
        }
    }
    
    // Handle key release (set previous_state back to 0)
    if (!kbhit()) {
        if (previous_state[0]) { previous_state[0] = 0; send_movement_command(sock, get_movement_value(' ')); }
        if (previous_state[1]) { previous_state[1] = 0; send_movement_command(sock, get_movement_value(' ')); }
        if (previous_state[2]) { previous_state[2] = 0; send_movement_command(sock, get_movement_value(' ')); }
        if (previous_state[3]) { previous_state[3] = 0; send_movement_command(sock, get_movement_value(' ')); }
        if (previous_state[4]) { previous_state[4] = 0; send_movement_command(sock, get_movement_value(' ')); }
    }
}

int main() {
    printf("ProtoStax Arduino Nano BLE Movement Peripheral Central Service\n");
    
    // Connect to the Bluetooth device
    int sock = connect_bluetooth_device();
    if (sock == -1) {
        return -1;
    }
    
    printf("Connected to Arduino Nano 33 BLE Sense\n");

    while (1) {
        check_keys(sock);
        usleep(100000);  // Sleep to prevent excessive CPU usage
    }

    close(sock);
    return 0;
}
