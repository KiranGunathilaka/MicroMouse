import telnetlib
import time

HOST = "192.168.8.194"
PORT = 23  # Default Telnet port

with telnetlib.Telnet(HOST, PORT) as tn, open("output.txt", "w") as f:
    while True:
        try:
            data = tn.read_some()
            if data:
                asciiData = data.decode('ascii')
                f.write(asciiData)
                print(asciiData)
                f.flush()
            time.sleep(0.1)
        except KeyboardInterrupt:
            break