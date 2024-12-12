import asyncio
import websockets
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

class InverterController:
    def __init__(self, port, baudrate):
        self.client = ModbusSerialClient(
            method="rtu",
            port=port,
            baudrate=baudrate,
            parity="N",
            stopbits=1,
            timeout=3,
        )
        if self.client.connect():
            print("Modbus connection successful!")
        else:
            print("Modbus connection failed!")
            raise ConnectionError("Failed to connect to the Modbus device.")

    def set_frequency(self, frequency_hz):
        """Set frequency (Hz)"""
        frequency_value = int(frequency_hz * 100)  # Convert to 0.01Hz unit
        try:
            response = self.client.write_register(4, frequency_value, unit=1)
            print(f"Frequency set response: {response}")
            return response
        except Exception as e:
            print(f"Error setting frequency: {e}")
            return None

    def set_direction(self, direction):
        """Set direction and stop"""
        if direction == 1:
            command_value = 0x13C2  # Forward direction
        elif direction == 2:
            command_value = 0x13C4  # Reverse direction
        elif direction == 0:
            command_value = 0x13C1  # Stop
        else:
            raise ValueError("Direction must be 0 (Stop), 1 (Forward), or 2 (Reverse).")

        try:
            response = self.client.write_register(5, command_value, unit=1)
            print(f"Direction set response: {response}")
            return response
        except Exception as e:
            print(f"Error setting direction: {e}")
            return None

    def close(self):
        """Close the Modbus connection"""
        self.client.close()

async def handle_client(websocket, path, controller):
    try:
        async for message in websocket:
            message = message.strip().lower()

            if message == 'q':  # Exit the loop
                print("Exiting the program.")
                await websocket.send("Program terminated.")
                break
            try:
                direction_input = int(message)  # Convert to integer
                if direction_input not in [0, 1, 2]:
                    raise ValueError("Invalid direction value. Please enter 0, 1, or 2.")

                # Set direction
                if controller.set_direction(direction_input) is None:
                    await websocket.send("Direction setting failed.")
                else:
                    await websocket.send(f"Direction set to {direction_input}.")
            except ValueError as ve:
                await websocket.send(f"Error: {ve}")

    except Exception as e:
        print(f"Error occurred: {e}")
        await websocket.send(f"Error occurred: {e}")

async def main():
    port = "COM4"  # Example: Change this to your port
    baudrate = 9600
    try:
        controller = InverterController(port, baudrate)

        # Set an initial frequency (optional)
        if controller.set_frequency(25) is None:
            print("Failed to set frequency.")

        # Start the WebSocket server, allowing external access
        print("Starting WebSocket server...")
        server = await websockets.serve(lambda ws, path: handle_client(ws, path, controller), "0.0.0.0", 8765)

        # Keep the server running
        await server.wait_closed()
    except Exception as e:
        print(f"Error occurred: {e}")
    finally:
        controller.close()

if __name__ == "__main__":
    asyncio.run(main())
