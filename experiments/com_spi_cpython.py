import spidev
import time

# Open the SPI device
# You may need to change the bus and device number depending on your setup
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 1000000

# A function to read data from the device
def read_data():
    # Read the data as bytes
    raw = spi.readbytes(1)
    # Decode the bytes as UTF-8 string
    text = bytes(raw).decode("utf-8")
    # Return the text
    return text

# A function to send data to the device
def send_data(text):
    # Encode the text as UTF-8 bytes
    raw = text.encode("utf-8")
    # Write the bytes to the SPI device
    spi.writebytes(raw)

# A loop to send and receive data
while True:
    # Read the data from the device
    data = read_data()
    # If there is any data, print it and send it back
    if data is not None:
        print("Received:", data)
        send_data("Echo: " + data)
    # Otherwise, send a hello message every second
    else:
        send_data("Hello from Python!\n")
        time.sleep(1)
