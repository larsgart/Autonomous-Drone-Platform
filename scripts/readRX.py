from models.rx import RX

rx = RX()

def main():
    while True:
        print(rx.read())

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rx.close()
