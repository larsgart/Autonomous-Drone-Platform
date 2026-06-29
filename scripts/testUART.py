import serial

# Define UART
uart2 = serial.Serial(port="/dev/ttyS0", baudrate=115200)

# Main function to receive motor speeds and output them
def main():
   while True:
      chars = input("Enter chars: ")
      char_arr = chars.encode()
      print(char_arr)
      # uart2.write(char_arr)

if __name__ == '__main__':
   main()