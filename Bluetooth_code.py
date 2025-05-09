import asyncio
from bleak import BleakClient, BleakScanner
import keyboard  # Import the keyboard library

# UUID for the movement characteristic
MOVEMENT_UUID = '69042F04-F8C3-4F4A-A8F4-15CD926DA146'

# Predefined commands
'''
COMMANDS = {
    'w': bytearray(b'forward'),
    's': bytearray(b'backward'),
    'a': bytearray(b'left'),
    'd': bytearray(b'right'),
    ' ': bytearray(b'stop')
}
'''
COMMANDS = {
    'w': bytearray([1]),
    's': bytearray([2]),
    'a': bytearray([3]),
    'd': bytearray([4]),
    ' ': bytearray([0]),
    'o': bytearray([5]),
    'c': bytearray([6]),
    'z': bytearray([7]),
    'r': bytearray([8]),
    't': bytearray([9])
}

# Track the last state of the button press
previous_state = {
    'w': False,
    's': False,
    'a': False,
    'd': False,
    ' ': False,
    'o': False,
    'c': False,
    'z': False,
    'r': False,
    't': False
}

def get_movement_value(command):
    """Get the bytearray value for a given command"""
    return COMMANDS.get(command, bytearray([0]))

async def send_movement_command(client, command):
    """Send movement command to Arduino device"""
    if command in COMMANDS:
        await client.write_gatt_char(MOVEMENT_UUID, get_movement_value(command))
        print(f"Sent movement command: {command} ({COMMANDS[command][0]})")
    else:
        print("Invalid command.")

async def run():
    """Main function to discover and connect to the Arduino Nano 33 BLE Sense"""
    global previous_state
    
    print('ProtoStax Arduino Nano BLE Movement Peripheral Central Service')
    print('Looking for Arduino Nano 33 BLE Sense Peripheral Device...')
    
    devices = await BleakScanner.discover()  # Updated discovery function
    found_device = None
    
    for d in devices:
        if d.name and "Arduino Nano 33 BLE Sense" in d.name:
            print(f'Found Arduino Nano 33 BLE Sense Peripheral at {d.address}')
            found_device = d
            break
    
    if not found_device:
        print('Could not find Arduino Nano 33 BLE Sense Peripheral')
        return
    
    async with BleakClient(found_device.address) as client:
        print(f'Connected to {found_device.address}')
        
        async def read_and_print_current_state():
            """Read and print the current movement state"""
            try:
                val = await client.read_gatt_char(MOVEMENT_UUID)
                current_command = val.decode()
                print(f"Current movement: {current_command}")
                return current_command
            except Exception as e:
                print(f"Error reading initial state: {e}")
                return None
        
        # Read initial state
        current_command = await read_and_print_current_state()
        
        # Start a continuous loop to listen to keyboard input
        while True:
            # Check each button's state and send the corresponding command when pressed
            for key, command in COMMANDS.items():
                if keyboard.is_pressed(key):  # Button is pressed
                    if not previous_state[key]:  # State change from not pressed to pressed
                        await send_movement_command(client, key)
                        previous_state[key] = True  # Update the state to pressed
                else:  # Button is not pressed
                    if previous_state[key]:  # State change from pressed to not pressed
                        await send_movement_command(client, ' ')  # Send 'stop' command when released
                        previous_state[key] = False  # Update the state to not pressed

            await asyncio.sleep(0.1)  # Sleep to prevent excessive CPU usage

if __name__ == "__main__":
    loop = asyncio.new_event_loop()  # Fixed event loop issue
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print('\nReceived Keyboard Interrupt')
    finally:
        print('Program finished')
