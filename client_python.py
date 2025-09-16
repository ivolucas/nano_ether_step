import socket
import time


class ArduinoUdpAngle:
    def __init__(self, ip, port=8888, local_port=9999, timeout=2.0):
        """
        Simple UDP client for Arduino communication.
        :param ip: Arduino IP address (string, e.g. "192.168.1.177")
        :param port: Arduino UDP port (default 8888)
        :param local_port: Local UDP port to listen on (default 9999)
        :param timeout: Socket timeout in seconds
        """
        self.arduino_ip = ip
        self.arduino_port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", local_port))
        self.sock.settimeout(timeout)

    def send_cmd(self, cmd):
        """Send a command (string) to Arduino."""
        self.sock.sendto(cmd.encode(), (self.arduino_ip, self.arduino_port))

    def request(self, cmd):
        """Send a command and wait for a reply."""
        self.send_cmd(cmd)
        try:
            data, _ = self.sock.recvfrom(1024)
            return data.decode().strip()
        except socket.timeout:
            return None

    def listen(self):
        """Wait for a UDP packet (streaming mode)."""
        try:
            data, _ = self.sock.recvfrom(1024)
            return data.decode().strip()
        except socket.timeout:
            return None

    # ---- Convenience wrappers ----
    def get_position(self):
        return self.request("P")

    def start_stream(self):
        return self.request("O")

    def stop_stream(self):
        return self.request("F")

    def reset(self):
        return self.request("R")


# ----------------- Example usage -----------------
if __name__ == "__main__":
    ard = ArduinoUdpAngle("192.168.0.17")

    print("Reset:", ard.reset())
    time.sleep(2)

    print("Single position:", ard.get_position())

    print("Start streaming:", ard.start_stream())
    while True:
        msg = ard.listen()
        if msg:
            print("Stream:", msg)

 
